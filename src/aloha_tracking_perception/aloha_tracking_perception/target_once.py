"""Serial left/right target following with explicit experimental right mapping."""
import json
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from std_srvs.srv import Trigger


def select_point(payload, frame, now, max_age):
    """Return XYZ only for an unambiguous, fresh measurement in metres."""
    items = json.loads(payload)
    if not isinstance(items, list) or len(items) != 1:
        raise ValueError('Waiting for exactly one detection')
    item = items[0]
    if not isinstance(item, dict):
        raise ValueError('Detection must be an object')
    if item.get('label') not in ('car', 'truck', 'train'):
        raise ValueError('Waiting for a car/truck/train detection')
    if item.get('frame_id') != frame or item.get('units') != 'm':
        raise ValueError('Require robot-base coordinates in metres')
    values = [item.get(key) for key in ('x', 'y', 'z', 'stamp')]
    if any(type(v) not in (int, float) or not math.isfinite(v) for v in values):
        raise ValueError('Require finite XYZ and timestamp')
    if not -.1 <= now - values[3] <= max_age:
        raise ValueError('Detection timestamp is stale or in the future')
    return values


def map_target(x, y, arm):
    """Input: filtered LEFT-base metres. Right mapping is an experiment, not TF."""
    if arm == 'left':
        return float(x), float(y), 0.300
    if arm == 'right':
        return 0.9652 - float(x), -float(y), 0.300
    raise ValueError('arm must be left or right')


class TargetOnce(Node):
    def __init__(self, arm='left'):
        if arm not in ('left', 'right'):
            raise ValueError('arm must be left or right')
        super().__init__('target_once' if arm == 'left' else 'right_target')
        self.arm = arm
        default_topic = '/grasp_pose' if arm == 'left' else '/follower_right/grasp_pose'
        default_status = '/arm_follow/status' if arm == 'left' else '/follower_right/arm_follow/status'
        defaults = dict(
            input_topic='/aloha/perception/detections_base_filtered',
            output_topic=default_topic, status_topic=default_status,
            orientation_x=0.0, orientation_y=0.7071067811865476,
            orientation_z=0.0, orientation_w=0.7071067811865476,
            reference_frame='follower_left/base_link', max_age=1.0,
            preview_only=(arm == 'right'), minimum_target_change=0.02, acknowledgement_timeout=300.0)
        for key, value in defaults.items():
            self.declare_parameter(key, value)
        self.frame = self.get_parameter('reference_frame').value
        if arm == 'right' and self.frame != 'follower_left/base_link':
            raise ValueError('Experimental right mapping requires filtered LEFT-base input')
        self.output_frame = self.frame if arm == 'left' else 'follower_right/base_link'
        self.max_age = self.get_parameter('max_age').value
        self.acknowledgement_timeout = self.get_parameter('acknowledgement_timeout').value
        if not math.isfinite(self.acknowledgement_timeout) or self.acknowledgement_timeout <= 0:
            raise ValueError('Acknowledgement timeout must be positive and finite')
        self.minimum_target_change = self.get_parameter('minimum_target_change').value
        if any(not math.isfinite(v) or v <= 0 for v in (self.max_age, self.minimum_target_change)):
            raise ValueError('Age and movement threshold must be positive and finite')
        self.preview_only = self.get_parameter('preview_only').value
        q = [self.get_parameter('orientation_' + axis).value for axis in 'xyzw']
        if any(not math.isfinite(v) for v in q) or sum(v*v for v in q) < 1e-12:
            raise ValueError('Orientation must be a finite nonzero quaternion')
        norm = math.sqrt(sum(v*v for v in q))
        self.orientation = [v/norm for v in q]
        self.latest = None
        self.last_commanded_xy = None
        self.planner_status = None
        self.status_received = 0.0
        self.pending_stamp = None
        self.pending_since = 0.0
        self.failed = False
        self.shutting_down = False
        self.follow_notice = None
        self.rest_client = self.create_client(Trigger, f'/follower_{arm}/return_to_rest')
        outgoing = self.get_parameter('output_topic').value
        if self.preview_only:
            outgoing = '/aloha/perception/target_preview' if arm == 'left' else '/follower_right/grasp_pose_preview'
        self.publisher = self.create_publisher(PoseStamped, outgoing, 10)
        self.subscription = self.create_subscription(String, self.get_parameter('input_topic').value, self.receive, 1)
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.status_subscription = self.create_subscription(String, self.get_parameter('status_topic').value, self.receive_status, qos)
        self.timer = self.create_timer(0.1, self.publish_next)
        self.get_logger().info(f'{arm} targets on {outgoing}; preview_only={self.preview_only}; fixed Z=0.300 m')
        if arm == 'right':
            self.get_logger().warning('MEASURED-SEPARATION APPROXIMATION: right X=0.9652-left-base X, right Y=-left-base Y; assumes facing bases aligned along left X. Verify before movement.')

    def receive(self, msg):
        try:
            self.latest = select_point(msg.data, self.frame, self.get_clock().now().nanoseconds/1e9, self.max_age)
        except (ValueError, TypeError) as exc:
            self.latest = None
            self.get_logger().info('FOLLOW WAIT: '+str(exc), throttle_duration_sec=5.)

    def receive_status(self, msg):
        try:
            status = json.loads(msg.data)
            if not isinstance(status, dict) or status.get('state') not in ('ready', 'moving', 'rejected', 'failed'):
                return
            self.planner_status = status
            self.status_received = time.monotonic()
        except (ValueError, TypeError):
            return

    def publish_next(self):
        if self.shutting_down is True:
            return
        if self.failed:
            return
        if not self.preview_only:
            if self.planner_status is None or time.monotonic()-self.status_received > 1.0:
                self.get_logger().info('FOLLOW WAIT: planner status unavailable/stale', throttle_duration_sec=5.)
                return
            if self.planner_status.get('rest_state') in ('requested', 'planning', 'moving', 'completed', 'previewed'):
                return
            state = self.planner_status['state']
            if state == 'failed':
                self.failed = True
                self.get_logger().error('Planner failed. Following stopped; resolve the cause before restarting.')
                return
            if self.pending_stamp is not None:
                completed = state in ('ready', 'rejected') and self.planner_status.get('goal_stamp') == self.pending_stamp
                if completed:
                    self.get_logger().info('FOLLOW: previous target '+state+'; checking latest detection for movement')
                    if state == 'rejected':
                        self.get_logger().warning('Target/path rejected; waiting for a changed position.')
                    self.pending_stamp = None
                else:
                    if time.monotonic()-self.pending_since > self.acknowledgement_timeout:
                        self.failed = True
                        self.get_logger().error('No target completion acknowledgement. Following stopped.')
                    return
            if state not in ('ready', 'rejected'):
                return
        if self.latest is None:
            return
        # Preview logs work even without a subscriber; motion requires a receiver.
        if not self.preview_only and self.publisher.get_subscription_count() == 0:
            return
        x, y, z, stamp = self.latest
        if not -0.1 <= self.get_clock().now().nanoseconds/1e9-stamp <= self.max_age:
            return
        if self.last_commanded_xy is not None:
            if math.hypot(x-self.last_commanded_xy[0], y-self.last_commanded_xy[1]) < self.minimum_target_change:
                return
        tx, ty, tz = map_target(x, y, self.arm)
        point = PoseStamped()
        point.header.frame_id = self.output_frame
        point.header.stamp.sec = int(stamp)
        point.header.stamp.nanosec = int((stamp-int(stamp))*1e9)
        point.pose.position.x, point.pose.position.y, point.pose.position.z = tx, ty, tz
        q = point.pose.orientation
        q.x, q.y, q.z, q.w = self.orientation
        if not self.preview_only:
            self.pending_stamp = [point.header.stamp.sec, point.header.stamp.nanosec]
            self.pending_since = time.monotonic()
        self.last_commanded_xy = (x, y)
        self.publisher.publish(point)
        self.get_logger().info(f'Input LEFT-base XY=({x:.3f}, {y:.3f}); {self.arm} target=({tx:.3f}, {ty:.3f}, {tz:.3f}) m; preview={self.preview_only}')

    def return_to_rest_on_exit(self, abort, timeout=300.):
        self.shutting_down = True
        self.timer.cancel()
        if self.preview_only:
            self.get_logger().info('Preview-only detector stopped; no return-to-rest motion requested.')
            return False
        self.get_logger().warning('Ctrl+C: tracking stopped. Requesting priority return to rest; keep planner and bringup running. Press Ctrl+C again to stop waiting.')
        deadline = time.monotonic()+min(timeout, 5.)
        while not self.rest_client.service_is_ready() and time.monotonic() < deadline and not abort.is_set() and rclpy.ok():
            rclpy.spin_once(self, executor=self.executor, timeout_sec=.1)
        if abort.is_set() or not self.rest_client.service_is_ready() or not rclpy.ok():
            self.get_logger().error('Rest service unavailable or shutdown interrupted; rest was not confirmed.')
            return False
        future = self.rest_client.call_async(Trigger.Request())
        deadline = time.monotonic()+5.
        while not future.done() and time.monotonic() < deadline and not abort.is_set() and rclpy.ok():
            rclpy.spin_once(self, executor=self.executor, timeout_sec=.1)
        if not future.done() or abort.is_set() or not rclpy.ok():
            self.get_logger().error('Rest request was not acknowledged; do not assume the arm is at rest.')
            return False
        try:
            response = future.result()
            if not response.success:
                raise ValueError(response.message)
            request_id = json.loads(response.message)['request_id']
        except Exception as exc:
            self.get_logger().error('Rest request refused: '+str(exc))
            return False
        deadline, last_state = time.monotonic()+timeout, None
        while time.monotonic() < deadline and not abort.is_set() and rclpy.ok():
            rclpy.spin_once(self, executor=self.executor, timeout_sec=.1)
            status = self.planner_status or {}
            if status.get('rest_id') != request_id:
                continue
            state = status.get('rest_state')
            if state != last_state:
                self.get_logger().info('RETURN TO REST: '+str(state))
                last_state = state
            if state == 'completed':
                self.get_logger().info('Rest reached and verified; detector exiting.')
                return True
            if state == 'previewed':
                self.get_logger().info('Rest preview displayed in RViz; no hardware command sent; detector exiting.')
                return True
            if state == 'failed':
                self.get_logger().error('Rest failed: '+status.get('rest_message', 'unknown reason'))
                return False
            if time.monotonic()-self.status_received > 2.:
                break
        self.get_logger().error('Rest wait interrupted/timed out; rest is not confirmed. Planner may still be finishing its last request.')
        return False


def main(args=None):
    from .target_runtime import main_for_arm
    main_for_arm('left', args)
