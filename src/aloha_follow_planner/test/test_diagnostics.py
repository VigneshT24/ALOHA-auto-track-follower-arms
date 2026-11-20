import json
import threading
import pytest
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from aloha_follow_planner.planner import AlohaGraspPlanner as Planner
from test_execution import fake,Plant

pytestmark=pytest.mark.usefixtures('virtual_clock')


def records(n):
    return [json.loads(c.args[0][len('EXEC_DIAG '):])
        for c in n.node.get_logger.return_value.info.call_args_list if c.args[0].startswith('EXEC_DIAG ')]


def test_stream_logs_setpoints_lag_and_final_completion(virtual_clock):
    n=fake(execute=True);Plant(n,virtual_clock)
    n.execute_path([[0.]*6,[.08]*6],{})
    logged=records(n)
    assert len([r for r in logged if r['event']=='SETPOINT_SENT'])>2
    assert any(r['event']=='TRACKING_SAMPLE' for r in logged)
    assert any(r['event']=='FINAL_SETTLING' for r in logged)
    assert logged[-1]['event']=='EXECUTION_FINISHED'


def test_stall_logs_joint_and_keeps_failed_context(virtual_clock):
    n=fake(execute=True);Plant(n,virtual_clock,frozen=1)
    n.select_path=lambda msg,positions:[[0.]*6,[.3]*6]
    n.display_path=lambda *args:None
    n.execute_sequence(PoseStamped())
    logged=records(n)
    stalled=[r for r in logged if r['event']=='TRACKING_STALL']
    assert stalled and stalled[0]['joints']==['shoulder']
    assert n.follow_state=='failed'
    assert logged[-1]['event']=='EXECUTION_FAILED'


def test_feedback_rejection_counts_and_stale_reason():
    n=fake();n.feedback_lock=threading.Lock();n.feedback=None
    n.feedback_stats=dict(received=0,accepted=0,rejected=0,last_rejection=None,last_received=None)
    n.node.get_clock.return_value.now.return_value.nanoseconds=100100000000
    msg=JointState();msg.header.stamp.sec=100
    from aloha_follow_planner.scoring import JOINTS
    msg.name=list(JOINTS);msg.position=[0.]*6
    n.joint_callback(msg);msg.position[1]=float('nan');n.joint_callback(msg)
    snapshot=n.feedback_diagnostics()
    assert snapshot['received']==2 and snapshot['accepted']==1 and snapshot['rejected']==1
    n.node.get_clock.return_value.now.return_value.nanoseconds=101000000000
    with pytest.raises(ValueError,match='valid_header_age_s'):Planner.measured_joints(n)
