"""Both arm wrappers must dispatch to the shared implementation."""
import runpy
from pathlib import Path
from unittest.mock import patch
import pytest

@pytest.mark.parametrize('arm', ['left', 'right'])
def test_arm_entry_point(arm):
    root = Path(__file__).resolve().parents[2]
    with patch('aloha_follow_planner.planner.main') as main:
        runpy.run_path(str(root/f'follower_{arm}_moveit_config'/'aloha_grasp_planner.py'), run_name='__main__')
    main.assert_called_once_with(arm)
