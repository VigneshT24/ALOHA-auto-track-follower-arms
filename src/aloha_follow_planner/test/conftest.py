from pathlib import Path
import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import pytest
from unittest.mock import patch

@pytest.fixture
def virtual_clock():
    import aloha_follow_planner.planner as planner_module
    class Clock:
        now=0.
        def __init__(self): self.callbacks=[]
        def monotonic(self): return self.now
        def sleep(self,seconds):
            self.now+=seconds
            for callback in list(self.callbacks): callback(seconds)
    clock=Clock()
    with patch.object(planner_module.time,'monotonic',clock.monotonic), \
         patch.object(planner_module.time,'sleep',clock.sleep), \
         patch.object(planner_module.rclpy,'ok',return_value=True):
        yield clock
