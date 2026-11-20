import math
import pytest
from aloha_follow_planner.tracking import JointPath,StallMonitor


def test_nominal_schedule_caps_speed_and_preserves_every_corner():
    a=[0.]*6;b=[.1,0.,0.,0.,0.,0.];c=[.1,.1,0.,0.,0.,0.]
    path=JointPath([a,b,b,c],.06,.1)
    assert path.points==[a,b,c]
    assert path.sample(path.times[1])[0]==pytest.approx(b)
    assert path.duration==pytest.approx(.2/.06)
    assert path.sample(path.duration)[0]==c


def test_large_clock_delay_does_not_jump_ahead():
    path=JointPath([[0.]*6,[.3]*6],.06,.1)
    t=path.advance(0.,100.)
    assert t==pytest.approx(.1)
    assert max(path.sample(t)[0])==pytest.approx(.006)


def test_advance_never_skips_vertex():
    path=JointPath([[0.]*6,[.003]*6,[.1]*6],.06,.1)
    assert path.advance(.05,.1)==pytest.approx(path.times[1])


def test_stall_is_per_joint_and_requires_persistent_error():
    monitor=StallMonitor(2.,.003,.025)
    assert monitor.update(0.,[0.]*6,[.04]*6)==[]
    assert monitor.update(1.,[0.,.01,.01,.01,.01,.01],[.04]*6)==[]
    assert monitor.update(2.1,[0.,.03,.03,.03,.03,.03],[.04]*6)==[0]


def test_realistic_lag_is_not_a_stall_when_position_advances():
    monitor=StallMonitor(2.,.003,.025)
    for step in range(100):
        current=[step*.004]*6;goal=[q+.03 for q in current]
        assert monitor.update(step*.1,current,goal)==[]


def test_invalid_schedule_rejected():
    with pytest.raises(ValueError): JointPath([[math.nan]*6],.06,.1)


def test_rviz_uses_the_same_nominal_schedule_as_execution():
    from unittest.mock import Mock
    from test_execution import fake
    from aloha_follow_planner.scoring import JOINTS
    n=fake()
    n.aloha_moveit=Mock();n.aloha_moveit.get_robot_model.return_value.name='aloha_vx300s'
    points=[[0.]*6,[.03]*6,[.06]*6]
    n.display_path(points,dict.fromkeys(JOINTS,0.))
    message=n.traj_pub.publish.call_args.args[0]
    actual=[p.time_from_start.sec+p.time_from_start.nanosec*1e-9
            for p in message.trajectory[0].joint_trajectory.points]
    schedule=JointPath(points,n.options['tracking_speed'],n.options['tracking_period'])
    assert actual==pytest.approx(schedule.times,abs=1e-8)
    assert [list(p.positions) for p in message.trajectory[0].joint_trajectory.points]==points
