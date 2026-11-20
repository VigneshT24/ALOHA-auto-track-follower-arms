"""Priority rest, real recorded lag, and fault-preserving cancellation."""
import threading
from types import SimpleNamespace as S
from unittest.mock import Mock

import pytest
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Trigger

from aloha_follow_planner.planner import AlohaGraspPlanner as Planner, RestRequested
from aloha_follow_planner.scoring import JOINTS
from test_execution import fake, Plant

pytestmark = pytest.mark.usefixtures('virtual_clock')


def current_options(n):
    n.options.update(tracking_speed=.08, tracking_pause_error=.06,
        tracking_error_limit=.08, stall_error_threshold=.048,
        stall_timeout=5., waypoint_tolerance=.035)


@pytest.mark.parametrize('arm', ['left', 'right'])
def test_recorded_second_move_lag_can_progress_with_new_envelope(virtual_clock, arm):
    n=fake(arm, execute=True); current_options(n)
    plant=Plant(n,virtual_clock,lag=.045)
    n.execute_path([[0.]*6,[0.,-.3,.2,0.,0.,0.]],{})
    assert plant.commands[-1][1][1] == pytest.approx(-.3)
    assert abs(plant.q[1]+.3)<.035


def test_new_envelope_still_stops_a_frozen_shoulder(virtual_clock):
    n=fake(execute=True);current_options(n)
    plant=Plant(n,virtual_clock,frozen=1)
    with pytest.raises(ValueError,match='Stalled joints: shoulder'):
        n.execute_path([[0.]*6,[.3]*6],{})
    assert max(q[1] for _,q in plant.commands)<.0616


def test_paused_fractional_steps_cannot_hide_a_stall(virtual_clock):
    n=fake(execute=True);current_options(n)
    plant=Plant(n,virtual_clock,frozen=1)
    # Shoulder advances more slowly than the leading wrist; its exact last
    # setpoint can land below the pause threshold by an arbitrary fraction.
    with pytest.raises(ValueError,match='Stalled joints: shoulder'):
        n.execute_path([[0.]*6,[0.,.23,0.,0.,.4,0.]],{})
    assert virtual_clock.now < 9.


def test_priority_rest_interrupts_stream_before_another_publish(virtual_clock):
    n=fake(execute=True);plant=Plant(n,virtual_clock)
    def interrupt(dt):
        if len(plant.commands)>=3: n.rest_requested.set()
    virtual_clock.callbacks.append(interrupt)
    with pytest.raises(RestRequested):n.execute_path([[0.]*6,[.3]*6],{})
    assert len(plant.commands)==3


def test_rest_interruption_does_not_latch_a_hardware_fault():
    n=fake(execute=True);n.rest_requested.set()
    n.select_path=Mock()
    n.execute_sequence(PoseStamped())
    assert n.follow_state=='ready'
    n.select_path.assert_not_called()


@pytest.mark.parametrize('enabled,fault',[(False,True),(True,True)])
def test_fault_refuses_rest(enabled,fault):
    n=fake(execute=enabled)
    if fault:n.follow_state='failed'
    response=n.request_rest(Trigger.Request(),Trigger.Response())
    assert not response.success
    assert not n.rest_requested.is_set()
    n.arm_pub.publish.assert_not_called()


def test_preview_rest_displays_without_executing_or_claiming_arrival():
    n=fake(execute=False);n.follow_state='ready'
    n.plan_rest=Mock(return_value=[[0.]*6,[.1]*6])
    n.display_path,n.execute_path=Mock(),Mock()
    n.return_to_rest(None)
    n.display_path.assert_called_once()
    n.execute_path.assert_not_called()
    n.check_motor_health.assert_not_called()
    assert n.rest_state=='previewed'


def test_rest_waits_for_tracking_then_checks_and_executes():
    n=fake(execute=True);old=Mock();old.is_alive.return_value=False
    n.plan_rest=Mock(return_value=[[0.]*6,[.1]*6])
    n.display_path,n.execute_path=Mock(),Mock()
    n.return_to_rest(old)
    old.join.assert_called_once()
    n.check_motor_health.assert_called_once()
    n.execute_path.assert_called_once()
    assert n.rest_state=='completed'
    assert not n.returning_rest


def test_unstopped_old_worker_cannot_compete_with_rest():
    n=fake(execute=True);old=Mock();old.is_alive.return_value=True
    n.execute_path=Mock()
    n.return_to_rest(old)
    assert n.rest_state=='failed'
    n.execute_path.assert_not_called()


def test_rest_request_is_idempotent_and_keeps_targets_blocked():
    import json
    n=fake(execute=True);n.follow_state='ready'
    n.plan_rest=Mock(return_value=[[0.]*6,[.1]*6])
    n.display_path,n.execute_path=Mock(),Mock()
    first=n.request_rest(Trigger.Request(),Trigger.Response())
    n.rest_worker.join(timeout=2.)
    second=n.request_rest(Trigger.Request(),Trigger.Response())
    assert first.success and second.success
    assert json.loads(first.message)==json.loads(second.message)
    assert n.rest_state=='completed' and n.rest_requested.is_set()
    n.execute_path.assert_called_once()


def test_rest_failure_is_acknowledged_without_sending_positions():
    n=fake(execute=True);n.plan_rest=Mock(side_effect=ValueError('blocked rest path'))
    n.execute_path=Mock()
    n.return_to_rest(None)
    assert n.rest_state=='failed' and n.follow_state=='failed'
    n.execute_path.assert_not_called()


def test_measured_clearance_loss_stops_further_rest_commands(virtual_clock):
    n=fake(execute=True);n.returning_rest=True
    n.rest_clearance=Mock()
    n.rest_clearance.violations.side_effect=lambda q,positions,margin:(max(0.,q[1]-.01),0.)
    plant=Plant(n,virtual_clock)
    with pytest.raises(ValueError,match='Rest clearance decreased'):
        n.execute_path([[0.]*6,[.2]*6],{})
    assert len(plant.commands)<5


