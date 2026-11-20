"""Controller tests with a virtual clock and explicit simulated plant behavior."""
from types import SimpleNamespace as S
from unittest.mock import Mock, patch
import math
import threading

import pytest
from geometry_msgs.msg import PoseStamped
from aloha_follow_planner.planner import AlohaGraspPlanner as Planner, TargetRejected
from aloha_follow_planner.scoring import JOINTS
from aloha_follow_planner.actuator_units import quantized_target

pytestmark = pytest.mark.usefixtures('virtual_clock')


def fake(arm='right', execute=False):
    n = Planner.__new__(Planner)
    n.arm, n.prefix, n.tip = arm, 'follower_'+arm, 'follower_'+arm+'/gripper_link'
    n.node, n.execute_motion = Mock(), execute
    n.stop_event = threading.Event()
    n.rest_requested = threading.Event()
    n.execution_lock = threading.Lock()
    n.returning_rest = False
    n.rest_state, n.rest_id, n.rest_message = 'idle', '', ''
    n.rest_worker, n.worker = None, None
    n.follow_state, n.is_executing = 'moving', True
    n.measured_joints = Mock(return_value=dict.fromkeys(JOINTS,0.))
    n.measured_positions = Mock(return_value=[0.]*6)
    n.arm_pub, n.traj_pub, n.legacy_traj_pub = Mock(), Mock(), Mock()
    n.log_command_readback = Mock()
    n.motor_limits = [(0,4095)]*6
    n.urdf = Mock(); n.urdf.in_bounds.return_value = True
    n.check_driver, n.check_motor_health, n.check_path = Mock(),Mock(),Mock(return_value=True)
    n.node.get_subscriptions_info_by_topic.return_value = [S(node_name='xs_sdk',node_namespace='/'+n.prefix)]
    n.options = dict(tracking_speed=.06,tracking_period=.1,tracking_pause_error=.04,
        tracking_error_limit=.06,stall_timeout=2.,stall_min_progress=.003,
        stall_error_threshold=.025,waypoint_tolerance=.02,waypoint_timeout=1.,
        execution_timeout=30.,diagnostic_period=1.)
    n.publish_status = Mock()
    return n


class Plant:
    def __init__(self,n,clock,lag=0.,frozen=None,hold_error=False,start=None):
        self.q = list(start if start is not None else [0.]*6)
        self.initial = list(self.q)
        self.goal = list(self.q)
        self.clock,self.lag,self.frozen,self.hold_error = clock,lag,frozen,hold_error
        self.last_command = -100.
        self.commands=[]
        def publish(msg):
            self.commands.append((clock.now,list(msg.cmd)))
            self.goal = quantized_target(msg.cmd)[1]
            self.last_command = clock.now
            if not lag:
                for i in range(6):
                    if i != frozen: self.q[i] = self.goal[i]
        n.arm_pub.publish.side_effect = publish
        n.measured_positions.side_effect = lambda: list(self.q)
        n.measured_joints.side_effect = lambda: dict(zip(JOINTS,self.q))
        clock.callbacks.append(self.update)

    def update(self,dt):
        for i in range(6):
            if i == self.frozen:
                self.q[i]=self.initial[i]
                continue
            target=self.goal[i]
            # Simulate the observed ~.023-rad shoulder lag during streaming.
            # In the settling plant it disappears after the final command is held.
            if i==1 and self.lag and (self.hold_error or self.clock.now-self.last_command < .3):
                error=target-self.q[i]
                if abs(error)<=self.lag:
                    target=self.q[i]
                else:
                    target-=math.copysign(self.lag,error)
            self.q[i]+=(target-self.q[i])*min(1.,dt/.05)


@pytest.mark.parametrize('arm',['left','right'])
def test_preview_never_commands_hardware(arm):
    n=fake(arm)
    points=[[0.]*6,[.02]*6]
    n.select_path,n.display_path,n.execute_path=Mock(return_value=points),Mock(),Mock()
    n.execute_sequence(PoseStamped())
    n.display_path.assert_called_once_with(points,dict.fromkeys(JOINTS,0.))
    n.execute_path.assert_not_called()
    n.arm_pub.publish.assert_not_called()
    assert n.follow_state=='ready'


@pytest.mark.parametrize('arm',['left','right'])
def test_execute_uses_selected_geometry_without_new_ik(arm):
    n=fake(arm,True)
    points=[[0.]*6,[.02]*6]
    n.select_path,n.display_path,n.execute_path=Mock(return_value=points),Mock(),Mock()
    n.execute_sequence(PoseStamped())
    assert n.execute_path.call_args.args[0] is points
    assert n.display_path.call_args.args[0] is points
    n.select_path.assert_called_once()


def test_unreachable_target_recoverable():
    n=fake();n.select_path=Mock(side_effect=TargetRejected('no IK'))
    n.execute_sequence(PoseStamped())
    assert n.follow_state=='rejected'
    n.arm_pub.publish.assert_not_called()


def test_motor_fault_latches_before_command():
    n=fake(execute=True)
    n.select_path=Mock(return_value=[[0.]*6,[.1]*6]);n.display_path=Mock()
    n.check_motor_health.side_effect=ValueError('shoulder_shadow overload 32')
    n.execute_sequence(PoseStamped())
    assert n.follow_state=='failed'
    n.arm_pub.publish.assert_not_called()


def test_timed_commands_follow_geometry_at_bounded_speed(virtual_clock):
    n=fake(execute=True);plant=Plant(n,virtual_clock)
    corner=[.03,0.,0.,0.,0.,0.];goal=[.03,.09,0.,0.,0.,0.]
    n.execute_path([[0.]*6,corner,goal],{})
    commands=[q for _,q in plant.commands]
    assert commands[-1]==pytest.approx(goal)
    assert any(q==pytest.approx(corner) for q in commands)
    # The shoulder never starts before the waist reaches its planned corner.
    assert all(q[1]==pytest.approx(0.) or q[0]==pytest.approx(.03) for q in commands)
    for a,b in zip([[0.]*6]+commands,commands):
        assert max(abs(x-y) for x,y in zip(a,b))<=.00600001
    assert len(commands)>3


def test_observed_moving_shoulder_lag_no_longer_blocks_tiny_samples(virtual_clock):
    n=fake(execute=True);plant=Plant(n,virtual_clock,lag=.023009777)
    points=[[0.,i*.003,0.,0.,0.,0.] for i in range(81)]
    n.execute_path(points,{})
    assert plant.commands[-1][1][1]==pytest.approx(.24)
    assert abs(plant.q[1]-quantized_target(points[-1])[1][1])<=.02
    assert n.check_motor_health.call_count>2


def test_frozen_shoulder_stops_instead_of_pushing_through(virtual_clock):
    n=fake(execute=True);plant=Plant(n,virtual_clock,frozen=1)
    with pytest.raises(ValueError,match='Stalled joints: shoulder'):
        n.execute_path([[0.]*6,[.3]*6],{})
    assert virtual_clock.now<6.
    assert max(q[1] for _,q in plant.commands)<=.0416
    # Other joints moving must not mask the stuck shoulder.
    assert any(plant.q[i]>0 for i in (0,2,3,4,5))


def test_observed_final_offset_completes_with_configured_arrival_tolerance(virtual_clock):
    n=fake(execute=True)
    n.options.update(waypoint_tolerance=.035, stall_error_threshold=.04,
                     tracking_pause_error=.05, stall_timeout=5.)
    plant=Plant(n,virtual_clock,lag=.030679613,hold_error=True)
    goal=[0.,.18,0.,0.,0.,0.]
    n.execute_path([[0.]*6,goal],{})
    assert .02 < abs(plant.q[1]-quantized_target(goal)[1][1]) <= .035


@pytest.mark.parametrize('tolerance,lag', [(.02,.023009777), (.035,.036)])
def test_permanent_final_error_does_not_report_success(virtual_clock,tolerance,lag):
    n=fake(execute=True)
    n.options.update(waypoint_tolerance=tolerance, stall_error_threshold=.04,
                     tracking_pause_error=.05, stall_timeout=5.)
    plant=Plant(n,virtual_clock,lag=lag,hold_error=True)
    goal=[0.,.18,0.,0.,0.,0.]
    with pytest.raises(ValueError,match='Final goal did not converge'):
        n.execute_path([[0.]*6,goal],{})
    assert plant.commands[-1][1]==pytest.approx(goal)
    assert abs(plant.q[1]-quantized_target(goal)[1][1])>tolerance


def test_stale_feedback_stops_further_commands(virtual_clock):
    n=fake(execute=True);plant=Plant(n,virtual_clock)
    def lost(dt):
        if len(plant.commands)>=3:
            n.measured_positions.side_effect=ValueError('stale feedback')
    virtual_clock.callbacks.append(lost)
    with pytest.raises(ValueError,match='stale'):
        n.execute_path([[0.]*6,[.2]*6],{})
    assert len(plant.commands)==3


def test_dynamic_fault_stops_stream(virtual_clock):
    n=fake(execute=True);plant=Plant(n,virtual_clock)
    n.check_motor_health.side_effect=[None,ValueError('overload')]
    with pytest.raises(ValueError,match='overload'):
        n.execute_path([[0.]*6,[.3]*6],{})
    assert max(q[1] for _,q in plant.commands)<.1


def test_external_deviation_stops_without_new_command(virtual_clock):
    n=fake(execute=True);plant=Plant(n,virtual_clock)
    def displaced(dt):
        if len(plant.commands)==3: plant.q[1]-=.15
    virtual_clock.callbacks.append(displaced)
    with pytest.raises(ValueError,match='Tracking error'):
        n.execute_path([[0.]*6,[.2]*6],{})
    assert len(plant.commands)==3


def test_changed_start_and_disabled_execution_prevent_commands():
    n=fake(execute=True);n.measured_positions.return_value=[.1]*6
    with pytest.raises(TargetRejected): n.execute_path([[0.]*6,[.2]*6],{})
    n.arm_pub.publish.assert_not_called()
    n=fake()
    with pytest.raises(ValueError,match='disabled'): n.execute_path([[0.]*6],{})
    n.check_driver.assert_not_called()


def test_observers_allowed_but_real_driver_required(virtual_clock):
    n=fake(execute=True);Plant(n,virtual_clock)
    n.node.get_subscriptions_info_by_topic.return_value.append(S(node_name='echo',node_namespace='/'))
    n.execute_path([[0.]*6,[.02]*6],{})
    n=fake(execute=True)
    n.node.get_subscriptions_info_by_topic.return_value=[S(node_name='echo',node_namespace='/')]
    with pytest.raises(ValueError,match='matching XS driver'): n.execute_path([[0.]*6,[.02]*6],{})
    n.arm_pub.publish.assert_not_called()


def test_outside_start_prefix_is_checked_but_never_commanded(virtual_clock):
    n=fake(execute=True)
    start=[0.,-.04,0.,0.,0.,0.]
    plant=Plant(n,virtual_clock,start=start)
    n.urdf.in_bounds.side_effect=lambda q:q[1]>=0.
    n.execute_path([start,[0.,-.02,0.,0.,0.,0.],[0.]*6,[.02]*6],{})
    assert all(q[1]>=0 for _,q in plant.commands)
    assert any(q[1]==pytest.approx(0.) for _,q in plant.commands)
    assert n.check_path.call_count>1


def test_stop_event_aborts_stream(virtual_clock):
    n=fake(execute=True);plant=Plant(n,virtual_clock)
    virtual_clock.callbacks.append(lambda dt:n.stop_event.set() if len(plant.commands)>=2 else None)
    with pytest.raises(ValueError,match='stopped'): n.execute_path([[0.]*6,[.2]*6],{})
    assert len(plant.commands)==2


def test_health_checks_primary_and_both_shadow_pairs():
    n=fake();n.read_register=Mock(side_effect=[[0]*6,[0],[0],[1]*6,[1],[1]])
    Planner.check_motor_health(n)
    assert [c.args[1] for c in n.read_register.call_args_list]==[
        'arm','shoulder_shadow','elbow_shadow','arm','shoulder_shadow','elbow_shadow']


def test_partial_register_response_is_not_healthy():
    n=fake();n.register_client=Mock();n.service_call=Mock(return_value=S(values=[]))
    with pytest.raises(ValueError,match='Invalid register'): n.read_register('group','arm','Hardware_Error_Status',6)


def test_model_check_and_readback_remain_read_only():
    n=fake();n.info_client=Mock()
    n.service_call=Mock(return_value=S(joint_names=list(JOINTS),mode='position',profile_type='velocity'))
    n.read_register=Mock(return_value=[9999]*6)
    with pytest.raises(ValueError,match='Unsupported actuator'): Planner.check_driver(n)
    n.read_register=Mock(side_effect=[[841]*6,[826]*6,[0]*6])
    result=Planner.log_command_readback(n,'after_first_command',[.1]*6)
    assert result['Goal_Position']==[841]*6 and result['Present_Position']==[826]*6
    n.arm_pub.publish.assert_not_called()


def test_recorded_folded_start_and_shoulder_lag_can_follow_into_workspace(virtual_clock):
    n=fake(execute=True)
    start=[-.00306796,-1.87145662,1.61374784,.01994175,-1.90673816,.03834952]
    bounded=[start[0],-1.850049007,1.605702911,start[3],-1.867502299,start[5]]
    second=[start[0],-1.847161666,1.603477445,start[3],-1.862502299,start[5]]
    goal=[0.,-1.3,1.,.02,-1.3,.04]
    plant=Plant(n,virtual_clock,lag=.023009777,start=start)
    lower=[-3.14158265,-1.850049008,-1.76278255,-3.14158265,-1.867502300,-3.14158265]
    upper=[3.14158265,1.25663707,1.605702912,3.14158265,2.23402145,3.14158265]
    n.urdf.in_bounds.side_effect=lambda q:all(lo<=x<=hi for lo,x,hi in zip(lower,q,upper))
    n.execute_path([start,bounded,second,goal],{})
    assert plant.commands[-1][1]==pytest.approx(goal)
    assert all(all(lo-1e-7<=x<=hi+1e-7 for lo,x,hi in zip(lower,q,upper)) for _,q in plant.commands)
    motor_lower=[0,841,898,0,830,0];motor_upper=[4095,2867,3094,4095,3504,4095]
    assert all(all(lo<=count<=hi for lo,count,hi in zip(motor_lower,quantized_target(q)[0],motor_upper)) for _,q in plant.commands)
    assert max(abs(a-b) for a,b in zip(plant.q,quantized_target(goal)[1]))<=.02


def test_hardware_count_limits_reject_before_publish(virtual_clock):
    n=fake(execute=True);Plant(n,virtual_clock)
    n.motor_limits=[(2048,2048)]*6
    with pytest.raises(ValueError,match='stored motor position limits'):
        n.execute_path([[0.]*6,[.1]*6],{})
    n.arm_pub.publish.assert_not_called()
