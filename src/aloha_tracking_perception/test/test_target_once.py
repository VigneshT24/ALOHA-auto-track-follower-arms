"""Serial target handling for both arms; no ROS hardware is started."""
import json
import time
from unittest.mock import Mock

import pytest
from aloha_tracking_perception.target_once import TargetOnce, select_point, map_target


def payload(**changes):
    item=dict(label='truck',frame_id='follower_left/base_link',units='m',x=.5,y=.02,z=.1,stamp=100.)
    item.update(changes)
    return json.dumps([item])


def tracker(arm='left', preview=False):
    n=Mock();n.acknowledgement_timeout=300.
    n.arm=arm;n.preview_only=preview;n.failed=False
    n.output_frame=f'follower_{arm}/base_link'
    n.planner_status={'state':'ready','goal_stamp':None}
    n.status_received=time.monotonic();n.pending_stamp=None;n.last_commanded_xy=None
    n.latest=[.5,.02,.1,100.];n.max_age=1.;n.minimum_target_change=.02
    n.orientation=[0.,2**-.5,0.,2**-.5]
    n.get_clock.return_value.now.return_value.nanoseconds=100100000000
    n.publisher.get_subscription_count.return_value=1
    return n


def test_select_point():
    assert select_point(payload(),'follower_left/base_link',100.1,1.)==[.5,.02,.1,100.]


@pytest.mark.parametrize('data',['[]','[{},{}]','{}','invalid',payload(frame_id='table_workspace'),payload(stamp=90.),payload(stamp=110.),payload(x=True),payload(z=float('nan')),payload(units='cm'),payload(label='person')])
def test_invalid(data):
    with pytest.raises((ValueError,TypeError)):select_point(data,'follower_left/base_link',100.1,1.)


def test_mapping_for_facing_bases():
    assert map_target(1.,2.,'right')==pytest.approx((-.0348,-2.,.3))
    assert map_target(1.,2.,'left')==(1.,2.,.3)


def test_right_mapping_not_applied_twice():
    n=tracker('right');TargetOnce.publish_next(n)
    p=n.publisher.publish.call_args.args[0]
    assert (p.pose.position.x,p.pose.position.y,p.pose.position.z)==pytest.approx((.4652,-.02,.3))
    assert p.header.frame_id=='follower_right/base_link'
    assert n.latest[:2]==[.5,.02]
    TargetOnce.publish_next(n);n.publisher.publish.assert_called_once()


def test_left_behavior_preserved():
    n=tracker();TargetOnce.publish_next(n)
    p=n.publisher.publish.call_args.args[0]
    assert (p.pose.position.x,p.pose.position.y,p.pose.position.z)==(.5,.02,.3)
    assert p.header.frame_id=='follower_left/base_link'


def test_preview_requires_no_planner_and_sets_no_pending():
    n=tracker('right',True);n.planner_status=None
    n.publisher.get_subscription_count.return_value=0
    TargetOnce.publish_next(n)
    n.publisher.publish.assert_called_once();assert n.pending_stamp is None


def test_small_change_does_not_publish():
    n=tracker();n.last_commanded_xy=(.501,.02)
    TargetOnce.publish_next(n);n.publisher.publish.assert_not_called()


def test_waits_for_own_completion_then_uses_latest():
    n=tracker();TargetOnce.publish_next(n)
    n.latest=[.55,.02,.1,100.05]
    TargetOnce.publish_next(n);n.publisher.publish.assert_called_once()
    n.planner_status={'state':'ready','goal_stamp':n.pending_stamp.copy()}
    TargetOnce.publish_next(n)
    assert n.publisher.publish.call_count==2


def test_stale_input_no_target():
    n=tracker();n.latest[-1]=90.
    TargetOnce.publish_next(n);n.publisher.publish.assert_not_called()


def test_fault_halts():
    n=tracker();n.planner_status['state']='failed'
    TargetOnce.publish_next(n);assert n.failed
    n.publisher.publish.assert_not_called()


def test_constructor_routes_right_and_left_to_separate_topics():
    # Compile the real class against an in-memory Node stub: no ROS context/socket.
    import ast
    from pathlib import Path
    from types import SimpleNamespace
    import aloha_tracking_perception.target_once as implementation

    class NodeStub:
        def __init__(self, *args):
            self.params = {}
            self.outgoing = []
            self.incoming = []
        def declare_parameter(self, key, value):self.params[key]=value
        def get_parameter(self, key):return SimpleNamespace(value=self.params[key])
        def create_publisher(self, kind, topic, qos):self.outgoing.append(topic);return Mock()
        def create_subscription(self, kind, topic, callback, qos):self.incoming.append(topic);return Mock()
        def create_timer(self,*args):return Mock()
        def create_client(self,*args):return Mock()
        def get_logger(self):return Mock()

    cls=next(n for n in ast.parse(Path(implementation.__file__).read_text()).body if isinstance(n,ast.ClassDef))
    ns=dict(implementation.__dict__);ns['Node']=NodeStub
    exec(compile(ast.Module(body=[cls],type_ignores=[]),'target_constructor_test','exec'),ns)
    right=ns['TargetOnce'](arm='right')
    left=ns['TargetOnce']()
    assert right.preview_only
    assert right.outgoing==['/follower_right/grasp_pose_preview']
    assert right.params['output_topic']=='/follower_right/grasp_pose'
    assert '/follower_right/arm_follow/status' in right.incoming
    assert '/arm_follow/status' not in right.incoming
    assert not left.preview_only
    assert left.outgoing==['/grasp_pose']
    assert '/arm_follow/status' in left.incoming


def test_previous_detection_in_right_base():
    assert map_target(.627,-.079,'right') == pytest.approx((.3382,.079,.3))


def test_midpoint_has_equal_forward_distance_from_both_bases():
    assert map_target(.9652/2,0.,'right') == pytest.approx((.9652/2,0.,.3))


@pytest.mark.parametrize("label", ["car", "truck", "train"])
def test_detector_vehicle_labels_are_accepted(label):
    assert select_point(payload(label=label), "follower_left/base_link", 100.1, 1.) == [.5,.02,.1,100.]
