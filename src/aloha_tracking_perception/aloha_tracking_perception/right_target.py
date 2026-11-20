"""Right follower entry point; shares serial target handling with the left path."""
import rclpy
from .target_once import TargetOnce


def main(args=None):
    from .target_runtime import main_for_arm
    main_for_arm('right', args)
