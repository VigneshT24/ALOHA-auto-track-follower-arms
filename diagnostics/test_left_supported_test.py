"""Offline checks only; these tests never initialize ROS or open hardware."""
from copy import deepcopy
import importlib.util
from pathlib import Path

import pytest
from builtin_interfaces.msg import Time

spec=importlib.util.spec_from_file_location('left_trial',Path(__file__).with_name('run_left_supported_test.py'))
trial=importlib.util.module_from_spec(spec);spec.loader.exec_module(trial)


def healthy():
    result={name:dict(Hardware_Error_Status=0,Torque_Enable=1,Operating_Mode=3,Drive_Mode=0) for name in trial.MOTORS}
    result['shoulder']['Drive_Mode']=1
    return result


def test_healthy_preflight_is_read_only():
    values=healthy();before=deepcopy(values)
    trial.validate_preflight(values)
    assert values==before


@pytest.mark.parametrize('motor',['shoulder','shoulder_shadow'])
def test_either_shoulder_overload_aborts_before_target(motor):
    values=healthy();values[motor]['Hardware_Error_Status']=32
    with pytest.raises(RuntimeError,match='NO target sent'):trial.validate_preflight(values)


def test_disabled_torque_is_not_automatically_enabled():
    values=healthy();values['shoulder_shadow']['Torque_Enable']=0
    with pytest.raises(RuntimeError,match='NO settings changed'):trial.validate_preflight(values)


def test_single_target_uses_fresh_publish_stamp_and_fixed_height():
    stamp=Time(sec=123,nanosec=456)
    message=trial.make_target([.4,-.02,.1,122.9],stamp)
    assert message.header.frame_id=='follower_left/base_link'
    assert message.header.stamp==stamp
    assert (message.pose.position.x,message.pose.position.y,message.pose.position.z)==(.4,-.02,.3)


def test_child_python_is_system_python_even_with_conda_first():
    import subprocess
    environment=trial.ros_environment(dict(PATH='/home/vignesh/miniconda3/bin:/usr/bin',
        PYTHONHOME='/home/vignesh/miniconda3'))
    assert 'PYTHONHOME' not in environment
    result=subprocess.run(['/usr/bin/env','python3','-c','import sys; print(sys.executable); print(sys.version_info.minor)'],
        env=environment,text=True,capture_output=True,check=True)
    assert result.stdout.splitlines()==['/usr/bin/python3','10']


def test_recorded_3905_shoulder_reference_refuses_motion():
    values=healthy()
    values['shoulder'].update(Present_Position=3905,Min_Position_Limit=841,Max_Position_Limit=2867)
    with pytest.raises(RuntimeError,match='3905 counts outside'):
        trial.validate_preflight(values)


def test_small_known_folded_start_offset_remains_allowed():
    values=healthy()
    values['shoulder'].update(Present_Position=828,Min_Position_Limit=841,Max_Position_Limit=2867)
    trial.validate_preflight(values)
