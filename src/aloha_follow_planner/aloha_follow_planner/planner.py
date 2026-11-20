"""Shared serial path selection and feedback-gated execution for either follower."""
import copy
import json
import math
import threading
import time
import uuid
import numpy as np

import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped, Pose
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger
from interbotix_xs_msgs.msg import JointGroupCommand
from interbotix_xs_msgs.srv import RegisterValues, RobotInfo
from moveit.planning import MoveItPy
from moveit.core.robot_state import RobotState
from moveit_msgs.msg import CollisionObject, DisplayTrajectory, RobotTrajectory
from shape_msgs.msg import SolidPrimitive
from trajectory_msgs.msg import JointTrajectoryPoint

from .tracking import JointPath, StallMonitor
from .actuator_units import SUPPORTED_MODELS, quantized_target, count_to_radians
from .scoring import JOINTS, UrdfModel, ScoreWeights, expand_path, score_path
from .rest_paths import moving_shoulder_load, retreat_matrices, matrix_quaternion
from .rest_clearance import RestClearance


class TargetRejected(ValueError):
    """No command sent: a changed target can be attempted without restarting."""


class RestRequested(ValueError):
    """Priority rest supersedes the tracking worker without clearing faults."""


class AlohaGraspPlanner:
    def __init__(self, arm):
        if arm not in ('left', 'right'):
            raise ValueError('arm must be left or right')
        self.arm, self.prefix = arm, f'follower_{arm}'
        self.node = rclpy.create_node(f'{arm}_grasp_subscriber',
            cli_args=['--ros-args', '-r', f'__node:={arm}_grasp_subscriber'])
        defaults = dict(execute_motion=False, compare_orientations=False,
            planning_budget=12.,
            ik_timeout=.08, plans_per_orientation=2, waypoint_step=.04,
            waypoint_tolerance=.035, waypoint_timeout=20., execution_timeout=240., start_bounds_tolerance=.05,
            nominal_preview_speed=.08, payload_mass_kg=0., diagnostic_period=1.,
            tracking_speed=.08, tracking_period=.1, tracking_pause_error=.06,
            tracking_error_limit=.08, stall_timeout=5., stall_min_progress=.003,
            stall_error_threshold=.048,
            score_joint_weights=[1., 3., 2., 1.5, 1.5, 1.5],
            score_excursion_weight=2., score_margin_weight=2.,
            score_shoulder_gravity_weight=3., score_elbow_gravity_weight=1.5,
            robot_description='')
        for name, default in defaults.items():
            self.node.declare_parameter(name, default)
        self.options = {name: self.node.get_parameter(name).value for name in defaults}
        self.execute_motion = self.options['execute_motion']
        if self.options['compare_orientations'] and self.execute_motion:
            raise ValueError('compare_orientations is preview-only; use its default false for automatic execution')
        for name in ('planning_budget', 'ik_timeout',
                     'waypoint_step', 'waypoint_tolerance', 'waypoint_timeout',
                     'execution_timeout', 'nominal_preview_speed', 'start_bounds_tolerance', 'diagnostic_period', 'tracking_speed', 'tracking_period',
                     'tracking_pause_error', 'tracking_error_limit', 'stall_timeout',
                     'stall_min_progress', 'stall_error_threshold'):
            if not math.isfinite(self.options[name]) or self.options[name] <= 0:
                raise ValueError(f'{name} must be positive and finite')
        if not 1 <= self.options['plans_per_orientation'] <= 4:
            raise ValueError('plans_per_orientation must be between 1 and 4')
        if self.options['waypoint_step'] > .05 or self.options['waypoint_tolerance'] > .035:
            raise ValueError('waypoint_step must be <= .05 rad and tolerance <= .035 rad')
        if self.options['start_bounds_tolerance'] > .05:
            raise ValueError('start_bounds_tolerance must be <= .05 rad')
        if not .05 <= self.options['tracking_period'] <= .2 or self.options['tracking_speed'] > .08:
            raise ValueError('tracking_period must be .05-.2 s and tracking_speed <= .08 rad/s')
        if not (self.options['waypoint_tolerance'] <= self.options['stall_error_threshold'] <=
                self.options['tracking_pause_error'] < self.options['tracking_error_limit'] <=
                .08):
            raise ValueError('Require final tolerance <= stall threshold <= pause error < hard tracking bound <= .08 rad')
        if not .0015 <= self.options['stall_min_progress'] <= .01 or self.options['stall_timeout'] < 1.:
            raise ValueError('Require stall progress .0015-.01 rad and stall timeout >= 1s')
        if self.options['stall_error_threshold'] >= (self.options['tracking_pause_error'] -
                self.options['tracking_speed']*self.options['tracking_period'] - .0016):
            raise ValueError('Stall threshold must leave one command step plus encoder resolution below soft pause')
        weights = self.options['score_joint_weights']
        factors = [self.options[name] for name in ('score_excursion_weight', 'score_margin_weight',
                   'score_shoulder_gravity_weight', 'score_elbow_gravity_weight', 'payload_mass_kg')]
        if len(weights) != 6 or any(not math.isfinite(x) or x < 0 for x in [*weights, *factors]):
            raise ValueError('Require six finite nonnegative joint weights and nonnegative scoring/payload settings')
        self.weights = ScoreWeights(tuple(weights), *factors[:4])
        self.urdf = UrdfModel(self.options['robot_description'])
        self.tip = f'{self.prefix}/gripper_link'
        if self.tip not in self.urdf.links:
            raise ValueError('Robot description does not match selected arm')
        self.stop_event = threading.Event()
        self.rest_requested = threading.Event()
        self.returning_rest = False
        self.rest_state, self.rest_id, self.rest_message = 'idle', '', ''
        self.rest_worker = None
        self.rest_clearance = None
        self.feedback_lock, self.execution_lock = threading.Lock(), threading.Lock()
        self.feedback, self.worker = None, None
        self.feedback_stats = dict(received=0, accepted=0, rejected=0, last_rejection=None, last_received=None)
        self.execution_context = {}
        self.is_executing, self.follow_state, self.goal_stamp = False, 'ready', None
        self.last_goal = None
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.traj_pub = self.node.create_publisher(DisplayTrajectory,
            f'/{self.prefix}/display_planned_path', latched)
        # Keep the existing single-arm RViz topic working. Use namespaced topics for two viewers.
        self.legacy_traj_pub = self.node.create_publisher(DisplayTrajectory, '/display_planned_path', latched)
        self.report_pub = self.node.create_publisher(String, f'/{self.prefix}/path_selection', latched)
        status_topic = '/arm_follow/status' if arm == 'left' else '/follower_right/arm_follow/status'
        self.status_pub = self.node.create_publisher(String, status_topic, latched)
        self.arm_pub = self.node.create_publisher(JointGroupCommand,
            f'/{self.prefix}/commands/joint_group', 1)
        self.register_client = self.node.create_client(RegisterValues, f'/{self.prefix}/get_motor_registers')
        self.info_client = self.node.create_client(RobotInfo, f'/{self.prefix}/get_robot_info')
        self.joint_subscription = self.node.create_subscription(JointState,
            f'/{self.prefix}/joint_states', self.joint_callback, qos_profile_sensor_data)
        target_topic = '/grasp_pose' if arm == 'left' else '/follower_right/grasp_pose'
        self.subscription = self.node.create_subscription(PoseStamped, target_topic, self.grasp_callback, 1)
        self.status_timer = self.node.create_timer(.2, self.publish_status)
        self.rest_service = self.node.create_service(Trigger, f'/{self.prefix}/return_to_rest', self.request_rest)
        self.aloha_moveit = MoveItPy(node_name=f'{arm}_moveit_py')
        self.arm_planner = self.aloha_moveit.get_planning_component('interbotix_arm')
        self.scene_monitor = self.aloha_moveit.get_planning_scene_monitor()
        # Apply directly under the scene lock instead of hoping a startup topic is received.
        ground = CollisionObject()
        ground.header.frame_id = self.aloha_moveit.get_robot_model().model_frame
        ground.id, ground.operation = 'table_surface', CollisionObject.ADD
        box = SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[2., 2., .05])
        pose = Pose()
        pose.orientation.w, pose.position.z = 1., -.025
        ground.primitives.append(box)
        ground.primitive_poses.append(pose)
        with self.scene_monitor.read_write() as scene:
            scene.apply_collision_object(ground)
        self.node.get_logger().info(f'Grasp Planner Ready. {target_topic}; execute_motion={self.execute_motion}. '
            f'Automatic A-D/multiple-seed path scoring enabled; executor_revision=11; execution_mode=timed_bounded; tracking_speed={self.options["tracking_speed"]}rad/s; waypoint_tolerance={self.options["waypoint_tolerance"]}rad. Detector Ctrl+C can request priority rest; planner shutdown sends no motion.')

    def joint_callback(self, msg):
        received = time.monotonic()
        stamp = msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
        age = self.node.get_clock().now().nanoseconds/1e9-stamp
        reason, positions = None, {}
        if len(msg.name) != len(msg.position) or len(set(msg.name)) != len(msg.name):
            reason = 'joint name/position lengths differ or names are duplicated'
        else:
            positions = dict(zip(msg.name, msg.position))
            if any(name not in positions for name in JOINTS):
                reason = 'required arm joints missing'
            elif not all(math.isfinite(v) for v in positions.values()):
                reason = 'nonfinite position values'
            elif stamp <= 0 or not -.1 <= age <= .5:
                reason = f'invalid timestamp or header age {age:.6f}s'
        # Counters only: do not print or query registers in this 100-Hz callback.
        with self.feedback_lock:
            self.feedback_stats['received'] += 1
            self.feedback_stats['last_received'] = received
            if reason is not None:
                self.feedback_stats['rejected'] += 1
                self.feedback_stats['last_rejection'] = reason
                return
            self.feedback_stats['accepted'] += 1
            self.feedback = (positions, stamp, received)

    def feedback_diagnostics(self):
        if not hasattr(self, 'feedback_lock'):
            return {}  # Also supports offline execution tests without a live subscriber.
        with self.feedback_lock:
            feedback = self.feedback
            stats = dict(getattr(self, 'feedback_stats', {}))
        now = time.monotonic()
        last_received = stats.pop('last_received', None)
        stats['last_message_age_s'] = None if last_received is None else round(now-last_received, 6)
        if feedback is not None:
            _, stamp, received = feedback
            stats['valid_receipt_age_s'] = round(now-received, 6)
            stats['valid_header_age_s'] = round(self.node.get_clock().now().nanoseconds/1e9-stamp, 6)
        return stats

    def execution_log(self, event, **details):
        record = dict(event=event, arm=self.arm, goal_stamp=getattr(self, 'goal_stamp', None),
                      context=dict(getattr(self, 'execution_context', {})))
        record.update(details)
        self.node.get_logger().info('EXEC_DIAG '+json.dumps(record))

    def measured_joints(self):
        with self.feedback_lock:
            feedback = self.feedback
        if feedback is None:
            raise ValueError(f'No valid {self.arm} joint feedback; diagnostics={self.feedback_diagnostics()}')
        positions, stamp, received = feedback
        age = self.node.get_clock().now().nanoseconds/1e9-stamp
        if time.monotonic()-received > .5 or not -.1 <= age <= .5:
            raise ValueError(f'{self.arm} joint feedback stale; diagnostics={self.feedback_diagnostics()}')
        return dict(positions)

    def measured_positions(self):
        positions = self.measured_joints()
        return [positions[name] for name in JOINTS]

    def publish_status(self):
        self.status_pub.publish(String(data=json.dumps(dict(
            state='moving' if self.is_executing else self.follow_state, goal_stamp=self.goal_stamp,
            execute_motion=self.execute_motion,
            rest_state=self.rest_state, rest_id=self.rest_id, rest_message=self.rest_message))))

    def guard_priority(self):
        if getattr(self, 'rest_requested', None) is not None and self.rest_requested.is_set() and not self.returning_rest:
            raise RestRequested('Tracking superseded by priority return to rest')

    def request_rest(self, request, response):
        with self.execution_lock:
            if self.follow_state == 'failed' or self.stop_event.is_set():
                response.success = False
                response.message = 'Rest unavailable: fault latched or planner stopping'
                return response
            if not self.rest_requested.is_set():
                self.rest_requested.set()  # Blocks every future tracking publish under the same lock.
                self.rest_id = uuid.uuid4().hex
                self.rest_state, self.rest_message = 'requested', 'Cancelling tracking before rest'
                old_worker = self.worker
                self.rest_worker = threading.Thread(target=self.return_to_rest, args=(old_worker,), daemon=True)
                self.rest_worker.start()
            response.success = True
            response.message = json.dumps(dict(request_id=self.rest_id))
        self.publish_status()
        return response

    def plan_rest(self, positions):
        info = self.service_call(self.info_client, RobotInfo.Request(cmd_type='group', name='arm'))
        if list(info.joint_names) != list(JOINTS) or len(info.joint_sleep_positions) != 6:
            raise ValueError('Driver did not supply six ordered rest positions')
        self.check_driver()
        measured = [positions[name] for name in JOINTS]
        bounded = self.urdf.bounded_start(measured, self.options['start_bounds_tolerance'])
        goal = []
        for name, sleep, (lo_count, hi_count) in zip(JOINTS, info.joint_sleep_positions, self.motor_limits):
            lo, hi = self.urdf.limits[name]
            lo, hi = max(lo, count_to_radians(lo_count)), min(hi, count_to_radians(hi_count))
            if not math.isfinite(sleep) or hi-lo <= .30:
                raise ValueError('Invalid rest position or actuator limits')
            # Factory sleep lies beyond this model's shoulder/wrist limits.
            # Use an inset folded pose, with geometric and arrival margin.
            goal.append(min(hi-.15, max(lo+.15, sleep)))
        initial_transforms,_ = self.urdf.transforms(positions)
        base_inverse = np.linalg.inv(initial_transforms[self.prefix+'/base_link'])
        initial_xy = (base_inverse@initial_transforms[self.tip])[:2,3]
        protected_xy = [initial_xy] if float(np.linalg.norm(initial_xy)) > RestClearance.inner_radius else []
        if self.last_goal is not None:
            target_transforms,_ = self.urdf.transforms(dict(positions,**dict(zip(JOINTS,self.last_goal))))
            protected_xy.append((base_inverse@target_transforms[self.tip])[:2,3])
        self.rest_clearance = RestClearance(self.options['robot_description'],self.urdf,self.prefix,protected_xy)
        guard = self.rest_clearance
        if guard.violation(goal,positions) > 0:
            raise ValueError('Folded goal does not satisfy padded rest clearance; no motion sent')
        if not self.check_path([measured],positions):
            raise ValueError('Measured rest start is in modeled collision; automatic recovery refused')
        deadline = time.monotonic()+20.
        transforms,_ = self.urdf.transforms(dict(positions,**dict(zip(JOINTS,bounded))))
        base = transforms[self.prefix+'/base_link']
        start_tool = transforms[self.tip]
        relative = np.linalg.inv(base)@start_tool
        height = max(.40,float(relative[2,3])+.08)
        candidates = []
        # Already folded: a short, fully checked adjustment is sufficient.
        if guard.metrics(measured,positions)['tool_radius'] <= guard.inner_radius-.02:
            direct = expand_path([measured,bounded,goal],.02)
            if guard.valid_path(direct,positions) and self.check_path(direct,positions):
                candidates.append((0.,direct,'already_retracted'))
        for radius in (.12,.16,.20):
            if candidates and candidates[0][2] == 'already_retracted':
                break
            if time.monotonic() >= deadline or self.stop_event.is_set():
                break
            prefix = expand_path([measured,bounded],.005)
            control_prefix = [measured,bounded]
            current = list(bounded)
            feasible = guard.valid_path(prefix,positions)
            for matrix in retreat_matrices(start_tool,base,radius,height):
                if not feasible or time.monotonic() >= deadline or self.stop_event.is_set():
                    feasible = False
                    break
                state = self.make_state(dict(positions,**dict(zip(JOINTS,current))))
                pose = Pose()
                pose.position.x,pose.position.y,pose.position.z = map(float,matrix[:3,3])
                pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w = map(float,matrix_quaternion(matrix))
                if not state.set_from_ik('interbotix_arm',pose,self.tip,timeout=.08):
                    feasible = False
                    break
                q = list(state.get_joint_group_positions('interbotix_arm'))
                if not self.urdf.in_bounds(q) or max(abs(a-b) for a,b in zip(q,current)) > .18:
                    feasible = False
                    break
                actual = self.urdf.transforms(dict(positions,**dict(zip(JOINTS,q))))[0][self.tip]
                if max(abs(float(a-b)) for a,b in zip(actual[:3,3],matrix[:3,3])) > .003:
                    feasible = False
                    break
                edge = expand_path([current,q],.005)
                if not guard.valid_path([*prefix,*edge[1:]],positions) or not self.check_path(edge,positions):
                    feasible = False
                    break
                prefix.extend(edge[1:]);control_prefix.append(q);current=q
            if not feasible:
                self.execution_log('REST_ROUTE_REJECTED',radius_m=radius,reason='upward/inward retreat infeasible or insufficient clearance')
                continue
            # Once above the base, try the efficient joint fold first. No low
            # distal-first or unguarded direct route is allowed as a fallback.
            options = [expand_path([current,goal],.02)]
            self.arm_planner.set_start_state(robot_state=self.make_state(dict(positions,**dict(zip(JOINTS,current)))))
            self.arm_planner.set_goal_state(robot_state=self.make_state(dict(positions,**dict(zip(JOINTS,goal)))))
            for attempt in range(2):
                leg = options.pop(0) if options else None
                if leg is None:
                    result = self.arm_planner.plan()
                    if not result:continue
                    trajectory = result.trajectory.get_robot_trajectory_msg().joint_trajectory
                    if len(trajectory.joint_names)!=6 or set(trajectory.joint_names)!=set(JOINTS):
                        raise ValueError('Rest plan returned unexpected joints')
                    order=[trajectory.joint_names.index(name) for name in JOINTS]
                    raw=[[point.positions[i] for i in order] for point in trajectory.points]
                    if not raw or max(abs(a-b) for a,b in zip(raw[0],current))>.01 or max(abs(a-b) for a,b in zip(raw[-1],goal))>.01:
                        continue
                    leg=expand_path(raw,.02)
                points=expand_path([*control_prefix,*leg[1:]],.02)
                if guard.valid_path(expand_path(points,.005),positions) and self.check_path(points,positions):
                    travel=sum(abs(b-a) for p,q in zip(points,points[1:]) for a,b in zip(p,q))
                    load=moving_shoulder_load(points,self.urdf,positions,self.tip)
                    candidates.append((travel+.2*load[0],points,'up_in_then_fold'))
                    self.execution_log('REST_CANDIDATE',route='up_in_then_fold',radius_m=radius,
                        moving_shoulder_peak_nm=load[0],minimum_geometry_z_m=min(guard.metrics(q,positions)['minimum_z'] for q in points))
                    break
        if not candidates:
            raise ValueError('No upward/inward rest path with full tool/table clearance; no motion sent')
        rank,points,label=min(candidates,key=lambda row:row[0])
        self.execution_log('REST_PLANNED',goal_rad=goal,route=label,
            floor_clearance_m=guard.floor,outer_tool_clearance_m=guard.tool_floor,
            retraction_radius_m=guard.inner_radius)
        return points

    def return_to_rest(self, old_worker):
        try:
            if old_worker is not None:
                old_worker.join(timeout=15.)
                if old_worker.is_alive():
                    raise ValueError('Tracking worker did not stop; no competing rest commands sent')
            if self.follow_state == 'failed' or self.stop_event.is_set():
                raise ValueError('Tracking fault or planner shutdown prevents automatic rest')
            self.returning_rest, self.is_executing = True, True
            self.rest_state = 'planning'
            self.publish_status()
            if self.execute_motion:
                self.check_motor_health()
            positions = self.measured_joints()
            points = self.plan_rest(positions)
            self.display_path(points, positions)
            if not self.execute_motion:
                self.rest_state, self.rest_message = 'previewed', 'Clearance-checked rest displayed; no motor command sent'
                self.execution_log('REST_PREVIEW')
                return
            self.rest_state = 'moving'
            self.publish_status()
            self.execute_path(points, positions)
            self.rest_state, self.rest_message = 'completed', 'Folded rest reached and verified'
            self.execution_log('REST_COMPLETED')
        except Exception as exc:
            self.rest_state, self.rest_message = 'failed', str(exc)
            self.follow_state = 'failed'
            self.node.get_logger().error('RETURN TO REST FAILED: '+str(exc))
        finally:
            self.returning_rest, self.is_executing = False, False
            # Keep rest requested latched: no late target can restart motion.
            if rclpy.ok():
                self.publish_status()

    def grasp_callback(self, msg):
        if msg.header.frame_id != f'{self.prefix}/base_link':
            self.node.get_logger().error(f'Require {self.prefix}/base_link target frame')
            return
        p, q = msg.pose.position, msg.pose.orientation
        values = [p.x, p.y, p.z, q.x, q.y, q.z, q.w]
        age = self.node.get_clock().now().nanoseconds/1e9-(msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9)
        if (not all(math.isfinite(v) for v in values) or
                abs(sum(v*v for v in values[3:])-1.) > .01 or not -.1 <= age <= 1.):
            self.node.get_logger().error('Rejected invalid/stale target')
            return
        if self.follow_state == 'failed' or self.stop_event.is_set():
            self.node.get_logger().error('Fault latched; inspect hardware before restarting planner')
            return
        with self.execution_lock:
            if self.is_executing or self.rest_requested.is_set():
                return
            self.is_executing, self.follow_state = True, 'moving'
            self.goal_stamp = [msg.header.stamp.sec, msg.header.stamp.nanosec]
            self.worker = threading.Thread(target=self.execute_sequence, args=(copy.deepcopy(msg),), daemon=True)
            self.worker.start()

    def make_state(self, positions):
        state = RobotState(self.aloha_moveit.get_robot_model())
        state.set_to_default_values()
        state.joint_positions = dict(positions)
        state.update()
        return state

    def check_path(self, points, positions):
        clearance = getattr(self, 'rest_clearance', None)
        if getattr(self, 'returning_rest', False) and clearance is not None:
            points = expand_path(points,.005)
            if not clearance.valid_path(points,positions):
                return False
        # Explicitly check dense samples; OMPL alone may fix an invalid start via adapters.
        if not self.urdf.valid_start_connection(points, self.options['start_bounds_tolerance']):
            return False
        state = self.make_state(positions)
        with self.scene_monitor.read_only() as scene:
            for point in points:
                self.guard_priority()
                if self.stop_event.is_set():
                    raise TargetRejected('Planner stopping')
                state.set_joint_group_positions('interbotix_arm', point)
                state.update()
                if scene.is_state_colliding(state, 'interbotix_arm', False):
                    return False
        return True

    def select_path(self, msg, positions):
        msg = copy.deepcopy(msg)
        if abs(msg.pose.position.z-.300) > 1e-6:
            self.node.get_logger().warning(f'Ignoring incoming target Z={msg.pose.position.z:.3f}; tracking Z is fixed at 0.300 m in {self.prefix}/base_link')
        msg.pose.position.z = .300
        measured = [positions[name] for name in JOINTS]
        bounded = self.urdf.bounded_start(measured, self.options['start_bounds_tolerance'])
        if bounded != measured:
            self.node.get_logger().warning('Measured rest is slightly outside URDF limits; including an explicit checked connection into bounds: '+str(bounded))
        if not self.check_path([measured], positions):
            raise TargetRejected('Measured start is in modeled collision; no automatic start-state repair')
        start = self.make_state(dict(positions, **dict(zip(JOINTS, bounded))))
        deadline = time.monotonic()+self.options['planning_budget']
        seeds = [bounded]
        if self.last_goal is not None:
            seeds.append(self.last_goal)
        for roll in (0., math.pi-.01, -math.pi+.01):
            seed = list(measured)
            seed[3], seed[4], seed[5] = roll, 0., 0.
            if self.urdf.in_bounds(seed):
                seeds.append(seed)
        seeds.append([(a+b)/2 for a, b in (self.urdf.limits[n] for n in JOINTS)])
        candidates, summary = [], []
        quaternion = msg.pose.orientation
        for label, yaw in zip('ABCD', (0., math.pi, math.pi/2, -math.pi/2)):
            pose = copy.deepcopy(msg.pose)
            # Left-multiply requested orientation by a yaw in this arm's base frame.
            s, c = math.sin(yaw/2), math.cos(yaw/2)
            pose.orientation.x = c*quaternion.x-s*quaternion.y
            pose.orientation.y = c*quaternion.y+s*quaternion.x
            pose.orientation.z = c*quaternion.z+s*quaternion.w
            pose.orientation.w = c*quaternion.w-s*quaternion.z
            solutions = []
            for seed in seeds:
                self.guard_priority()
                if self.stop_event.is_set() or time.monotonic() >= deadline:
                    break
                state = self.make_state(positions)
                state.set_joint_group_positions('interbotix_arm', seed)
                state.update()
                if not state.set_from_ik('interbotix_arm', pose, self.tip, timeout=self.options['ik_timeout']):
                    continue
                goal = list(state.get_joint_group_positions('interbotix_arm'))
                if not self.urdf.in_bounds(goal) or any(max(abs(a-b) for a, b in zip(goal, old)) < .01 for old in solutions):
                    continue
                solutions.append(goal)
            # Endpoint heuristic only orders planning attempts; final ranking uses the complete path.
            solutions.sort(key=lambda goal: score_path([measured, goal], self.urdf, positions,
                self.tip, self.weights, self.options['payload_mass_kg'])['score'])
            for branch, goal in enumerate(solutions[:self.options['plans_per_orientation']], 1):
                self.guard_priority()
                remaining = deadline-time.monotonic()
                if self.stop_event.is_set() or remaining <= 0:
                    break
                row = dict(candidate=label, branch=branch, planned=False,
                           target_deg=[round(math.degrees(v), 2) for v in goal])
                self.arm_planner.set_start_state(robot_state=start)
                self.arm_planner.set_goal_state(robot_state=self.make_state(dict(positions, **dict(zip(JOINTS, goal)))))
                result = self.arm_planner.plan()  # Uses launch/YAML plan_request_params; compatible with this Humble binding.
                if result:
                    trajectory = result.trajectory.get_robot_trajectory_msg().joint_trajectory
                    if set(trajectory.joint_names) != set(JOINTS) or len(trajectory.joint_names) != 6:
                        raise ValueError('Planner returned unexpected trajectory joints')
                    order = [trajectory.joint_names.index(n) for n in JOINTS]
                    raw = [[p.positions[i] for i in order] for p in trajectory.points]
                    if (not raw or max(abs(a-b) for a, b in zip(raw[0], bounded)) > .01 or
                            max(abs(a-b) for a, b in zip(raw[-1], goal)) > .01):
                        row['reason'] = 'Planner changed start or goal'
                    else:
                        # Include the precise measured start instead of an adapter-adjusted start.
                        points = expand_path([measured, bounded, *raw], self.options['waypoint_step'])
                        dense = expand_path(points, min(.02, self.options['waypoint_step']))
                        if self.check_path(dense, positions):
                            row.update(score_path(dense, self.urdf, positions, self.tip,
                                self.weights, self.options['payload_mass_kg']))
                            row['planned'] = True
                            candidates.append((row['score'], points, row))
                        else:
                            row['reason'] = 'Bounds/collision check failed'
                summary.append(row)
                self.node.get_logger().info('CANDIDATE '+json.dumps(row))
        if not candidates:
            raise TargetRejected('No feasible path from tested orientations/seeds within planning budget')
        _, points, selected = min(candidates, key=lambda candidate: candidate[0])
        self.report_pub.publish(String(data=json.dumps(dict(arm=self.arm, goal_stamp=self.goal_stamp,
            target_xyz=[msg.pose.position.x, msg.pose.position.y, msg.pose.position.z],
            selected=selected, candidates=summary,
            scoring_note='URDF static gravity estimates, not measured/per-servo torque or a safety guarantee'))))
        self.node.get_logger().info('SELECTED '+json.dumps(selected))
        return points

    def display_path(self, points, positions):
        display = DisplayTrajectory()
        display.model_id = self.aloha_moveit.get_robot_model().name
        display.trajectory_start.joint_state.name = list(positions)
        display.trajectory_start.joint_state.position = list(positions.values())
        trajectory = RobotTrajectory()
        trajectory.joint_trajectory.joint_names = list(JOINTS)
        schedule = self.timed_path(points,[positions[name] for name in JOINTS])
        for point, elapsed in zip(schedule.points,schedule.times):
            waypoint = JointTrajectoryPoint(positions=list(point))
            waypoint.time_from_start.sec = int(elapsed)
            waypoint.time_from_start.nanosec = int((elapsed-int(elapsed))*1e9)
            trajectory.joint_trajectory.points.append(waypoint)
        display.trajectory.append(trajectory)
        self.traj_pub.publish(display)
        self.legacy_traj_pub.publish(display)

    def service_call(self, client, request):
        if not client.service_is_ready():
            raise ValueError('Motor service unavailable')
        future = client.call_async(request)
        deadline = time.monotonic()+1.
        while not future.done() and time.monotonic() < deadline and not self.stop_event.is_set():
            time.sleep(.01)
        if not future.done():
            client.remove_pending_request(future)
            raise ValueError('Motor service read timed out/stopped')
        response = future.result()
        if response is None:
            raise ValueError('Motor service returned no data')
        return response

    def read_register(self, cmd_type, name, register, count):
        request = RegisterValues.Request(cmd_type=cmd_type, name=name, reg=register)
        values = self.service_call(self.register_client, request).values
        if len(values) != count:
            raise ValueError(f'Invalid register response for {name}/{register}: {values}')
        return list(values)

    def check_motor_health(self):
        for register, expected in (('Hardware_Error_Status', 0), ('Torque_Enable', 1)):
            values = self.read_register('group', 'arm', register, 6)
            if any(v != expected for v in values):
                raise ValueError(f'{self.arm} arm {register}={values}; no further waypoints')
            for shadow in ('shoulder_shadow', 'elbow_shadow'):
                value = self.read_register('single', shadow, register, 1)[0]
                if value != expected:
                    raise ValueError(f'{self.arm} {shadow} {register}={value}; no automatic fault reset')

    def check_driver(self):
        response = self.service_call(self.info_client, RobotInfo.Request(cmd_type='group', name='arm'))
        if list(response.joint_names) != list(JOINTS) or response.mode != 'position' or response.profile_type != 'velocity':
            raise ValueError('Driver arm order/mode/profile does not match position waypoint executor')
        models = self.read_register('group', 'arm', 'Model_Number', 6)
        if any(model not in SUPPORTED_MODELS for model in models):
            raise ValueError(f'Unsupported actuator conversion for model numbers {models}; no commands sent')
        lower = self.read_register('group', 'arm', 'Min_Position_Limit', 6)
        upper = self.read_register('group', 'arm', 'Max_Position_Limit', 6)
        if any(lo > hi for lo, hi in zip(lower, upper)):
            raise ValueError('Invalid motor position limits')
        self.motor_limits = list(zip(lower,upper))
        velocity_max, acceleration_max = (6, 3) if self.arm == 'right' else (5, 2)
        for register, maximum in (('Profile_Velocity', velocity_max), ('Profile_Acceleration', acceleration_max)):
            values = self.read_register('group', 'arm', register, 6)
            if any(not 0 < v <= maximum for v in values):
                raise ValueError(f'{register}={values}: require nonzero values <= {maximum}; no settings changed')

    def log_command_readback(self, stage, goal):
        """Read through the existing driver; never open the serial port here."""
        counts, actuator_goal = quantized_target(goal)
        snapshot = dict(stage=stage, joint_order=list(JOINTS), commanded_rad=list(goal),
                        expected_goal_counts=counts, actuator_goal_rad=actuator_goal)
        for register in ('Goal_Position', 'Present_Position', 'Present_Current'):
            snapshot[register] = self.read_register('group', 'arm', register, 6)
        snapshot['measured_rad'] = self.measured_positions()
        self.node.get_logger().info('COMMAND_READBACK '+json.dumps(snapshot))
        return snapshot

    def timed_path(self, points, current):
        if not points or not self.urdf.in_bounds(points[-1]):
            raise TargetRejected('Final path target is outside joint bounds')
        remaining = list(points[1:]) if len(points) > 1 else [points[-1]]
        # Rebuild the short initial connection from current feedback. Initial
        # out-of-bounds animation samples are never motor targets.
        while remaining and not self.urdf.in_bounds(remaining[0]):
            remaining.pop(0)
        return JointPath([list(current), *remaining], self.options['tracking_speed'],
                         self.options['tracking_period'])

    def execute_path(self, points, positions):
        started = time.monotonic()
        self.execution_context = dict(stage='preflight', waypoint=0)
        self.execution_log('EXECUTION_START', execute_motion=self.execute_motion,
            settings={name: self.options[name] for name in ('tracking_speed', 'tracking_period',
                'tracking_pause_error', 'tracking_error_limit', 'stall_timeout',
                'waypoint_tolerance', 'waypoint_timeout', 'execution_timeout')},
            joint_order=list(JOINTS), feedback=self.feedback_diagnostics())
        if not self.execute_motion:
            raise ValueError('Hardware execution disabled')
        self.check_driver()
        self.check_motor_health()
        current = self.measured_positions()
        clearance = getattr(self, 'rest_clearance', None) if getattr(self, 'returning_rest', False) else None
        clearance_best = list(clearance.violations(current,positions,margin=.03)) if clearance is not None else None
        if max(abs(a-b) for a,b in zip(current,points[0])) > .02:
            raise TargetRejected('Arm moved during planning; no command sent')
        endpoints = self.node.get_subscriptions_info_by_topic(f'/{self.prefix}/commands/joint_group')
        drivers = [endpoint for endpoint in endpoints if endpoint.node_name == 'xs_sdk'
                   and endpoint.node_namespace == f'/{self.prefix}']
        if len(drivers) != 1:
            raise ValueError('Require exactly one matching XS driver; passive observers are allowed')
        schedule = self.timed_path(points,current)
        if not self.check_path(expand_path(schedule.points,.02),self.measured_joints()):
            raise TargetRejected('Initial connection or scene is invalid; no command sent')
        self.execution_log('STREAM_READY', nominal_duration_s=schedule.duration,
            vertices=len(schedule.points), measured_start_rad=current,
            final_goal_rad=schedule.points[-1])
        period = self.options['tracking_period']
        tolerance = self.options['waypoint_tolerance']
        monitor = StallMonitor(self.options['stall_timeout'], self.options['stall_min_progress'],
                               self.options['stall_error_threshold'])
        deadline = time.monotonic()+self.options['execution_timeout']
        path_time = 0.
        previous_tick = time.monotonic()-period
        next_health, next_log = time.monotonic()+1., 0.
        last_counts, last_goal, last_ideal = None, None, None
        sent_count, paused, final_started, settled = 0, False, None, None

        def readback_failure(stage):
            if last_ideal is not None:
                try:
                    self.log_command_readback(stage,last_ideal)
                except Exception as exc:
                    self.node.get_logger().error('Failure register readback unavailable: '+str(exc))

        def check_rest_clearance(point):
            if clearance is None:
                return
            violation = clearance.violations(point,self.measured_joints(),margin=.03)
            if any(v > best+.003 for v,best in zip(violation,clearance_best)):
                readback_failure('rest_clearance')
                raise ValueError('Rest clearance decreased below its measured recovery/clearance bound; no further commands')
            clearance_best[:] = [min(a,b) for a,b in zip(clearance_best,violation)]

        while True:
            self.guard_priority()
            tick = time.monotonic()
            self.execution_context = dict(stage='tracking', path_time_s=round(path_time,6),
                nominal_duration_s=schedule.duration, command_number=sent_count)
            if self.stop_event.is_set() or not rclpy.ok():
                raise ValueError('Execution stopped; no further setpoints sent')
            if tick >= deadline:
                readback_failure('execution_timeout')
                raise ValueError('Total execution timeout; no further setpoints sent')
            current = self.measured_positions()
            check_rest_clearance(current)
            if last_goal is not None:
                error = max(abs(a-b) for a,b in zip(current,last_goal))
                if error > self.options['tracking_error_limit']:
                    readback_failure('tracking_error')
                    raise ValueError(f'Tracking error {error:.6f}rad exceeds hard limit; no further setpoints')
            if tick >= next_health:
                self.execution_context['stage'] = 'health_check'
                health_started = time.monotonic()
                self.check_motor_health()
                self.execution_log('HEALTH_CHECK_PASSED',duration_s=time.monotonic()-health_started,
                                   feedback=self.feedback_diagnostics())
                next_health = time.monotonic()+1.
                current = self.measured_positions()
                check_rest_clearance(current)
            proposed_time = schedule.advance(path_time,max(0.,tick-previous_tick))
            previous_tick = tick
            proposed, vertex = schedule.sample(proposed_time)
            self.execution_context.update(stage='tracking', waypoint=vertex,
                total_waypoints=len(schedule.points)-1, commanded_rad=proposed)
            did_pause = False
            if not self.urdf.in_bounds(proposed):
                # The checked measured-start prefix can be outside the model;
                # advance its nominal clock but never publish those samples.
                path_time = proposed_time
            else:
                counts, actuator_goal = quantized_target(proposed)
                if any(not lo <= value <= hi for value,(lo,hi) in zip(counts,self.motor_limits)):
                    raise ValueError('Setpoint would violate stored motor position limits; no command sent')
                candidate_error = max(abs(a-b) for a,b in zip(current,actuator_goal))
                # The first in-bounds recovery command can span the allowed
                # start deviation. All commands remain within the hard envelope.
                if candidate_error > self.options['tracking_error_limit']:
                    readback_failure('candidate_tracking_error')
                    raise ValueError('Next setpoint exceeds hard tracking bound; no further setpoints')
                if last_goal is not None and candidate_error > self.options['tracking_pause_error']:
                    did_pause = True
                else:
                    if counts != last_counts:
                        self.execution_context['stage'] = 'collision_check'
                        if not self.check_path(expand_path([current,proposed],.02), self.measured_joints()):
                            raise ValueError('Upcoming measured-to-target segment is in modeled collision')
                        first = last_goal is None
                        if first:
                            self.log_command_readback('before_first_command',proposed)
                        # Read again after any slow collision/service work.
                        current = self.measured_positions()
                        check_rest_clearance(current)
                        if self.stop_event.is_set() or not rclpy.ok() or time.monotonic() >= deadline:
                            raise ValueError('Execution stopped/timed out before next setpoint')
                        fresh_error = max(abs(a-b) for a,b in zip(current,actuator_goal))
                        if fresh_error > self.options['tracking_error_limit'] or (last_goal is not None and
                                max(abs(a-b) for a,b in zip(current,last_goal)) > self.options['tracking_error_limit']):
                            raise ValueError('Arm moved outside hard tracking bound before publish')
                        if not first and fresh_error > self.options['tracking_pause_error']:
                            did_pause = True
                        else:
                            with self.execution_lock:
                                self.guard_priority()
                                self.arm_pub.publish(JointGroupCommand(name='arm',cmd=list(proposed)))
                            sent_count += 1
                            last_counts, last_goal, last_ideal = counts, actuator_goal, list(proposed)
                            self.execution_context.update(stage='tracking',command_number=sent_count)
                            self.execution_log('SETPOINT_SENT', expected_counts=counts,
                                actuator_goal_rad=actuator_goal,measured_rad=current,
                                nominal_path_time_s=proposed_time)
                            if first:
                                time.sleep(.05)
                                self.log_command_readback('after_first_command',proposed)
                    if not did_pause:
                        path_time = proposed_time
            if did_pause != paused:
                self.execution_log('TRACKING_PAUSED' if did_pause else 'TRACKING_RESUMED',
                    reason='bounded lag; path clock pauses without catch-up jumps' if did_pause else 'feedback caught up')
            paused = did_pause
            current = self.measured_positions()
            if last_goal is not None:
                stalled = monitor.update(time.monotonic(),current,last_goal)
                if stalled:
                    self.execution_log('TRACKING_STALL', joints=[JOINTS[i] for i in stalled],
                        measured_rad=current,actuator_goal_rad=last_goal)
                    readback_failure('tracking_stall')
                    raise ValueError('Stalled joints: '+', '.join(JOINTS[i] for i in stalled))
            finished_path = path_time >= schedule.duration-1e-9
            if finished_path:
                if final_started is None:
                    final_started = time.monotonic()
                    self.execution_log('FINAL_SETTLING', tolerance_rad=tolerance)
                _, final_goal = quantized_target(schedule.points[-1])
                if max(abs(a-b) for a,b in zip(current,final_goal)) <= tolerance:
                    settled = time.monotonic() if settled is None else settled
                    if time.monotonic()-settled >= .04:
                        self.check_motor_health()
                        current = self.measured_positions()
                        check_rest_clearance(current)
                        if max(abs(a-b) for a,b in zip(current,final_goal)) > tolerance:
                            settled = None
                            continue
                        self.execution_context['stage'] = 'finished'
                        self.execution_log('EXECUTION_FINISHED',commands_sent=sent_count,
                            duration_s=time.monotonic()-started,measured_rad=current)
                        return
                else:
                    settled = None
                if time.monotonic()-final_started >= self.options['waypoint_timeout']:
                    readback_failure('final_goal_timeout')
                    raise ValueError('Final goal did not converge within tolerance; no further setpoints')
            if time.monotonic() >= next_log:
                target = last_goal if last_goal is not None else current
                errors = [b-a for a,b in zip(current,target)]
                self.execution_log('TRACKING_SAMPLE', nominal_path_time_s=path_time,
                    nominal_duration_s=schedule.duration,paused=paused,measured_rad=current,
                    actuator_goal_rad=target,error_rad=errors,
                    error_deg=[math.degrees(e) for e in errors],
                    final_error_rad=[b-a for a,b in zip(current,quantized_target(schedule.points[-1])[1])],
                    feedback=self.feedback_diagnostics())
                next_log = time.monotonic()+self.options['diagnostic_period']
            time.sleep(max(.001,period-(time.monotonic()-tick)))

    def execute_sequence(self, msg):
        try:
            self.guard_priority()
            positions = self.measured_joints()
            points = self.select_path(msg, positions)
            self.display_path(points, positions)
            self.last_goal = list(points[-1])
            if self.execute_motion:
                self.execute_path(points, positions)
                self.node.get_logger().info('Selected timed path completed with bounded tracking and final convergence')
            else:
                self.node.get_logger().info('PREVIEW: selected path displayed; no hardware commands. '
                    'Playback uses the nominal stream schedule; hardware pauses its path clock if lag grows.')
        except RestRequested as exc:
            self.execution_log('TRACKING_CANCELLED_FOR_REST', reason=str(exc))
        except TargetRejected as exc:
            self.execution_log('EXECUTION_REJECTED', reason=str(exc), feedback=self.feedback_diagnostics())
            self.follow_state = 'rejected'
            self.node.get_logger().warning(str(exc)+'; waiting for changed target')
        except Exception as exc:
            self.execution_log('EXECUTION_FAILED', reason=str(exc), feedback=self.feedback_diagnostics())
            self.follow_state = 'failed'
            self.node.get_logger().error(str(exc))
        finally:
            if self.follow_state not in ('failed', 'rejected'):
                self.follow_state = 'ready'
            self.is_executing = False
            if rclpy.ok():
                self.publish_status()

    def stop(self):
        self.stop_event.set()
        if self.worker is not None:
            self.worker.join(timeout=5.)
        if self.rest_worker is not None:
            self.rest_worker.join(timeout=5.)
        # Avoid tearing down MoveIt while a worker still uses its C++ objects.
        if (self.worker is None or not self.worker.is_alive()) and (self.rest_worker is None or not self.rest_worker.is_alive()):
            self.arm_planner = None
            self.scene_monitor = None
            self.aloha_moveit.shutdown()


def main(arm='right', args=None):
    rclpy.init(args=args)
    planner = AlohaGraspPlanner(arm)
    try:
        rclpy.spin(planner.node)
    except KeyboardInterrupt:
        pass
    finally:
        planner.stop()
        planner.node.destroy_node()
        rclpy.try_shutdown()
