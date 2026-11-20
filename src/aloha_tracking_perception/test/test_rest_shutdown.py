"""Detector shutdown waits for the specific planner rest completion."""
import json
import threading
from types import SimpleNamespace as S
from unittest.mock import Mock, patch

import pytest
from aloha_tracking_perception.target_once import TargetOnce


def node():
    n=Mock()
    n.preview_only=False
    n.planner_status=None
    n.rest_client.service_is_ready.return_value=True
    future=Mock()
    future.done.return_value=True
    future.result.return_value=S(success=True,message=json.dumps(dict(request_id='new')))
    n.rest_client.call_async.return_value=future
    return n


def test_shutdown_waits_for_matching_completion_not_old_status():
    n=node();sequence=iter([
        dict(rest_id='old',rest_state='completed'),
        dict(rest_id='new',rest_state='moving'),
        dict(rest_id='new',rest_state='completed')])
    def spin(*args,**kw):
        n.planner_status=next(sequence)
        import time
        n.status_received=time.monotonic()
    with patch('rclpy.ok',return_value=True),patch('rclpy.spin_once',side_effect=spin) as calls:
        assert TargetOnce.return_to_rest_on_exit(n,threading.Event())
    assert calls.call_count==3
    n.timer.cancel.assert_called_once()
    assert n.shutting_down is True


def test_preview_shutdown_sends_no_rest_request():
    n=node();n.preview_only=True
    assert not TargetOnce.return_to_rest_on_exit(n,threading.Event())
    n.rest_client.call_async.assert_not_called()


def test_fault_refusal_is_reported_without_waiting():
    n=node()
    n.rest_client.call_async.return_value.result.return_value=S(success=False,message='fault latched')
    with patch('rclpy.ok',return_value=True):
        assert not TargetOnce.return_to_rest_on_exit(n,threading.Event())
    assert 'fault latched' in n.get_logger.return_value.error.call_args.args[0]


def test_reported_rest_failure_does_not_claim_success():
    n=node();n.planner_status=dict(rest_id='new',rest_state='failed',rest_message='collision')
    with patch('rclpy.ok',return_value=True),patch('rclpy.spin_once'):
        assert not TargetOnce.return_to_rest_on_exit(n,threading.Event())


def test_preview_acknowledgement_is_not_reported_as_physical_arrival():
    n=node();n.planner_status=dict(rest_id='new',rest_state='previewed')
    with patch('rclpy.ok',return_value=True),patch('rclpy.spin_once'):
        assert TargetOnce.return_to_rest_on_exit(n,threading.Event())
    assert 'no hardware command' in n.get_logger.return_value.info.call_args.args[0]


def test_already_requested_rest_blocks_all_new_detection_targets():
    from test_target_once import tracker
    n=tracker('right')
    n.planner_status.update(rest_state='requested')
    TargetOnce.publish_next(n)
    n.publisher.publish.assert_not_called()
