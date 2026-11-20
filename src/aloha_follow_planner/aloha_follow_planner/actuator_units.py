"""XM430/XM540 position conversions matching Dynamixel Workbench float32 arithmetic.

Only use after verifying the actual model numbers; these constants do not apply
universally to DYNAMIXEL motors. No I/O or motor register writes occur here.
"""
import math
import numpy as np

SUPPORTED_MODELS = frozenset((1020, 1120))  # XM430-W350 / XM540-W270 in this ALOHA.


def count_to_radians(count):
    return float(np.float32(np.float32(int(count)-2048)*np.float32(math.pi))/np.float32(2048))


def quantized_target(positions):
    if len(positions) != 6 or not all(math.isfinite(q) for q in positions):
        raise ValueError('Require six finite arm targets')
    counts = []
    for position in positions:
        # Workbench truncates the float32 result toward zero, rather than rounding.
        value = int(np.float32(np.float32(np.float32(position)*np.float32(2048)) /
                              np.float32(math.pi))+np.float32(2048))
        if not 0 <= value <= 4095:
            raise ValueError('Target is outside XM single-turn encoder range')
        counts.append(value)
    return counts, [count_to_radians(value) for value in counts]
