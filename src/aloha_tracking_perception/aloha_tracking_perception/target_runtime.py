"""Keep ROS alive on launch SIGINT until the priority rest request completes."""
import signal
import threading
import time

import rclpy
from rclpy.signals import SignalHandlerOptions
from rclpy.executors import SingleThreadedExecutor

from .target_once import TargetOnce


def main_for_arm(arm, args=None):
    # rclpy's default SIGINT handler invalidates the context before cleanup can
    # call services. Launch still gets SIGINT; this child keeps its context alive.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    requested, abort = threading.Event(), threading.Event()
    last_interrupt = [0.]

    def interrupt(signum, frame):
        now = time.monotonic()
        # A foreground SIGINT and launch's forwarded SIGINT can arrive as a
        # pair. They represent one keypress, not an instruction to abandon rest.
        if signum == signal.SIGINT and requested.is_set() and now-last_interrupt[0] < .5:
            return
        last_interrupt[0] = now
        if requested.is_set() or signum == signal.SIGTERM:
            abort.set()
        requested.set()

    handlers = {sig: signal.signal(sig, interrupt) for sig in (signal.SIGINT, signal.SIGTERM)}
    node = None
    executor = None
    exit_code = 1
    try:
        node = TargetOnce(arm=arm)
        executor = SingleThreadedExecutor()
        executor.add_node(node)
        while rclpy.ok() and not requested.is_set():
            executor.spin_once(timeout_sec=.1)
        if requested.is_set() and not abort.is_set():
            completed = node.return_to_rest_on_exit(abort)
            exit_code = 0 if completed or node.preview_only else 1
    finally:
        if executor is not None:
            executor.shutdown(timeout_sec=2.)
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        for sig, handler in handlers.items():
            signal.signal(sig, handler)
    if exit_code:
        raise SystemExit(exit_code)
