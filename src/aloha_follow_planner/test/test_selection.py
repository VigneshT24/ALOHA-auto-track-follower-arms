"""Exercise automatic candidate selection independently of live ROS/hardware."""
from contextlib import nullcontext
import math
import threading
from types import SimpleNamespace as S
from unittest.mock import Mock, patch

import pytest
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from aloha_follow_planner.planner import AlohaGraspPlanner as Planner, TargetRejected
from aloha_follow_planner.scoring import JOINTS, ScoreWeights


class Model:
    limits = dict.fromkeys(JOINTS, (-math.pi, math.pi))
    def in_bounds(self, q):
        return len(q) == 6 and all(-math.pi <= x <= math.pi for x in q)
    def bounded_start(self, q, tolerance):
        return list(q)
    def valid_start_connection(self, points, tolerance):
        return all(self.in_bounds(q) for q in points)
    def gravity_loads(self, positions, tip, payload):
        return [0., 0.]


class State:
    def __init__(self, positions):
        self.q = [positions[n] for n in JOINTS]
    def update(self): pass
    def set_joint_group_positions(self, group, seed): self.q = list(seed)
    def set_from_ik(self, group, pose, tip, timeout):
        z, w = pose.orientation.z, pose.orientation.w
        value = .1 if abs(z) < .01 else (.01 if abs(w) < .01 else (.2 if z > 0 else .3))
        self.q = [value]*6
        return True
    def get_joint_group_positions(self, group): return self.q


def selector():
    n = Planner.__new__(Planner)
    n.arm, n.tip = 'right', 'follower_right/gripper_link'
    n.prefix = 'follower_right'
    n.options = dict(planning_budget=10., start_bounds_tolerance=.05, plans_per_orientation=2,
        planning_time_per_candidate=.75, ik_timeout=.08, waypoint_step=.04, payload_mass_kg=0.)
    n.urdf, n.weights = Model(), ScoreWeights(shoulder_load=0., elbow_load=0.)
    n.node, n.aloha_moveit, n.report_pub = Mock(), Mock(), Mock()
    n.stop_event, n.last_goal, n.goal_stamp = threading.Event(), None, [100, 0]
    n.make_state = lambda positions: State(positions)
    n.check_path = Mock(return_value=True)
    n.arm_planner = Mock()
    def plan():
        start = n.arm_planner.set_start_state.call_args.kwargs['robot_state'].q
        goal = n.arm_planner.set_goal_state.call_args.kwargs['robot_state'].q
        msg = RobotTrajectory()
        # Deliberately reversed joint names verify trajectory mapping by name.
        msg.joint_trajectory.joint_names = list(reversed(JOINTS))
        points = [start, goal]
        if goal[0] == .01:
            points.insert(1, [1.]*6)  # Closest endpoint, but long planned detour.
        msg.joint_trajectory.points = [JointTrajectoryPoint(positions=list(reversed(q))) for q in points]
        return S(trajectory=S(get_robot_trajectory_msg=lambda: msg))
    n.arm_planner.plan.side_effect = plan
    msg = PoseStamped(); msg.pose.orientation.w = 1.
    return n, msg


def test_ranks_full_planned_paths_and_publishes_report():
    n, msg = selector()
    with nullcontext():
        points = n.select_path(msg, dict.fromkeys(JOINTS, 0.))
    assert points[-1] == pytest.approx([.1]*6)  # A wins despite B's closer endpoint.
    assert n.arm_planner.plan.call_count == 4
    assert max(abs(a-b) for p, q in zip(points, points[1:]) for a, b in zip(p, q)) <= .04000001
    import json
    report = json.loads(n.report_pub.publish.call_args.args[0].data)
    assert report['selected']['candidate'] == 'A'
    assert {row['candidate'] for row in report['candidates']} == set('ABCD')


def test_no_feasible_plan_rejects_without_latching_hardware_fault():
    n, msg = selector()
    n.arm_planner.plan.side_effect = None
    n.arm_planner.plan.return_value = None
    with nullcontext():
        with pytest.raises(TargetRejected, match='No feasible path'):
            n.select_path(msg, dict.fromkeys(JOINTS, 0.))


def test_scene_collision_rejects_before_any_planning():
    n, msg = selector()
    n.check_path.return_value = False
    with pytest.raises(TargetRejected, match='collision'):
        n.select_path(msg, dict.fromkeys(JOINTS, 0.))
    n.arm_planner.plan.assert_not_called()


@pytest.mark.parametrize('incoming_z',[.1,.6,-.2])
def test_tracking_ik_always_uses_fixed_base_height_without_mutating_input(incoming_z):
    n,msg=selector();msg.pose.position.z=incoming_z
    captured=[]
    original=n.make_state
    def make(positions):
        state=original(positions);ik=state.set_from_ik
        def solve(group,pose,tip,timeout):
            captured.append(pose.position.z)
            return ik(group,pose,tip,timeout)
        state.set_from_ik=solve
        return state
    n.make_state=make
    n.select_path(msg,dict.fromkeys(JOINTS,0.))
    assert captured and all(z==.300 for z in captured)
    assert msg.pose.position.z==incoming_z
    import json
    assert json.loads(n.report_pub.publish.call_args.args[0].data)['target_xyz'][2]==.300
