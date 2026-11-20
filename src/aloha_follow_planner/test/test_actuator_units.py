import pytest
from aloha_follow_planner.actuator_units import count_to_radians, quantized_target


def test_exact_logged_first_command_maps_to_motor_goal_registers():
    goal = [-.004601942375302315, -1.8500490071139892, 1.6057029118347832,
            .006135923322290182, -1.8675022996339325, .03374757990241051]
    counts, radians = quantized_target(goal)
    assert counts == [2045, 841, 3094, 2052, 830, 2070]
    assert radians[1] == pytest.approx(count_to_radians(841))


def test_real_thirteen_count_shoulder_error_is_inside_existing_tolerance():
    ideal = -1.8500490071139892
    _, executed = quantized_target([0., ideal, 0., 0., 0., 0.])
    measured = count_to_radians(828)
    assert abs(measured-ideal) > .02
    assert abs(measured-executed[1]) < .02
    assert abs(count_to_radians(827)-executed[1]) > .02


def test_zero_and_bounded_full_range():
    counts, _ = quantized_target([0., -3.14158265, 3.14158265, 0., 0., 0.])
    assert counts == [2048, 0, 4095, 2048, 2048, 2048]
