"""Unreachable targets must not disable subsequent reachable targets."""
import time
from unittest.mock import Mock

from aloha_tracking_perception.target_once import TargetOnce


def tracker():
    node = Mock()
    node.acknowledgement_timeout = 300.
    node.arm = 'left'
    node.preview_only = False
    node.output_frame = 'follower_left/base_link'
    node.failed = False
    node.planner_status = {'state': 'rejected', 'goal_stamp': [100, 0]}
    node.status_received = time.monotonic()
    node.pending_stamp = [100, 0]
    node.pending_since = time.monotonic()
    node.last_commanded_xy = (.8, .02)
    node.latest = [.8, .02, .1, 100.]
    node.minimum_target_change = .02
    node.frame = 'follower_left/base_link'
    node.max_age = 1.
    node.orientation = [0., 2**-.5, 0., 2**-.5]
    node.publisher.get_subscription_count.return_value = 1
    node.get_clock.return_value.now.return_value.nanoseconds = 100100000000
    return node


def test_rejection_clears_pending_without_disabling_or_resending_same_target():
    node = tracker()
    TargetOnce.publish_next(node)
    assert node.pending_stamp is None
    assert not node.failed
    node.publisher.publish.assert_not_called()
    TargetOnce.publish_next(node)
    node.publisher.publish.assert_not_called()


def test_new_position_after_rejection_is_submitted_and_waits_for_its_ack():
    node = tracker()
    node.latest = [.5, .02, .1, 100.05]
    TargetOnce.publish_next(node)
    node.publisher.publish.assert_called_once()
    point = node.publisher.publish.call_args.args[0]
    assert point.pose.position.x == .5
    assert point.pose.position.z == .3
    assert not node.failed
    TargetOnce.publish_next(node)
    node.publisher.publish.assert_called_once()


def test_unrelated_rejection_does_not_clear_pending():
    node = tracker()
    node.planner_status['goal_stamp'] = [99, 0]
    node.latest = [.5, .02, .1, 100.05]
    TargetOnce.publish_next(node)
    assert node.pending_stamp == [100, 0]
    node.publisher.publish.assert_not_called()


def test_hardware_failure_still_disables_following():
    node = tracker()
    node.planner_status['state'] = 'failed'
    TargetOnce.publish_next(node)
    assert node.failed
    node.publisher.publish.assert_not_called()


def test_right_following_submits_latest_moved_target_after_completion():
    node = tracker()
    node.arm = 'right'
    node.output_frame = 'follower_right/base_link'
    node.planner_status['state'] = 'moving'
    node.latest = [.6, -.04, .1, 100.05]
    TargetOnce.publish_next(node)
    node.publisher.publish.assert_not_called()
    node.planner_status['state'] = 'ready'
    TargetOnce.publish_next(node)
    point = node.publisher.publish.call_args.args[0]
    assert abs(point.pose.position.x - .3652) < 1e-9
    assert point.pose.position.y == .04
    assert point.header.frame_id == 'follower_right/base_link'
    assert not node.failed


def test_rejected_status_is_accepted():
    node = tracker()
    TargetOnce.receive_status(node, Mock(data='{"state":"rejected","goal_stamp":[100,0]}'))
    assert node.planner_status['state'] == 'rejected'


def test_long_monitored_path_does_not_hit_old_60_second_timeout():
    node = tracker()
    node.planner_status['state'] = 'moving'
    node.pending_since = time.monotonic()-90.
    TargetOnce.publish_next(node)
    assert not node.failed
    node.publisher.publish.assert_not_called()


def test_missing_completion_still_times_out():
    node = tracker()
    node.planner_status['state'] = 'moving'
    node.pending_since = time.monotonic()-301.
    TargetOnce.publish_next(node)
    assert node.failed
