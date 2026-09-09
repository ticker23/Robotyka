import pytest

from anti_drone.controller import PIDController


def test_proportional_positive_error() -> None:
    controller = PIDController(kp=2.0, ki=0.0, kd=0.0)

    output = controller.update(target=10.0, current=5.0, dt=1.0)

    assert output == pytest.approx(10.0)


def test_proportional_negative_error() -> None:
    controller = PIDController(kp=2.0, ki=0.0, kd=0.0)

    output = controller.update(target=5.0, current=10.0, dt=1.0)

    assert output == pytest.approx(-10.0)


def test_proportional_zero_error() -> None:
    controller = PIDController(kp=2.0, ki=0.0, kd=0.0)

    output = controller.update(target=5.0, current=5.0, dt=1.0)

    assert output == pytest.approx(0.0)


def test_zero_proportional_gain_is_valid() -> None:
    controller = PIDController(kp=0.0, ki=0.0, kd=0.0)

    output = controller.update(target=10.0, current=5.0, dt=1.0)

    assert output == pytest.approx(0.0)


def test_integral_accumulates_over_multiple_updates() -> None:
    controller = PIDController(kp=0.0, ki=1.0, kd=0.0)

    first_output = controller.update(target=2.0, current=0.0, dt=0.5)
    second_output = controller.update(target=2.0, current=0.0, dt=0.5)

    assert first_output == pytest.approx(1.0)
    assert second_output == pytest.approx(2.0)


def test_integral_incorporates_different_dt_values() -> None:
    controller = PIDController(kp=0.0, ki=2.0, kd=0.0)

    first_output = controller.update(target=3.0, current=1.0, dt=0.25)
    second_output = controller.update(target=3.0, current=1.0, dt=0.75)

    assert first_output == pytest.approx(1.0)
    assert second_output == pytest.approx(4.0)


def test_zero_integral_gain_disables_integral_contribution() -> None:
    controller = PIDController(kp=0.0, ki=0.0, kd=0.0)

    first_output = controller.update(target=2.0, current=0.0, dt=1.0)
    second_output = controller.update(target=2.0, current=0.0, dt=1.0)

    assert first_output == pytest.approx(0.0)
    assert second_output == pytest.approx(0.0)


def test_derivative_is_zero_on_first_update() -> None:
    controller = PIDController(kp=0.0, ki=0.0, kd=3.0)

    output = controller.update(target=10.0, current=0.0, dt=0.5)

    assert output == pytest.approx(0.0)


def test_changing_error_produces_correct_derivative() -> None:
    controller = PIDController(kp=0.0, ki=0.0, kd=1.0)

    controller.update(target=10.0, current=0.0, dt=1.0)
    output = controller.update(target=6.0, current=0.0, dt=2.0)

    assert output == pytest.approx(-2.0)


def test_increasing_error_produces_positive_derivative() -> None:
    controller = PIDController(kp=0.0, ki=0.0, kd=0.5)

    controller.update(target=4.0, current=0.0, dt=1.0)
    output = controller.update(target=10.0, current=0.0, dt=3.0)

    assert output == pytest.approx(1.0)


def test_decreasing_error_produces_negative_derivative() -> None:
    controller = PIDController(kp=0.0, ki=0.0, kd=0.25)

    controller.update(target=8.0, current=0.0, dt=1.0)
    output = controller.update(target=2.0, current=0.0, dt=0.5)

    assert output == pytest.approx(-3.0)


def test_zero_derivative_gain_disables_derivative_contribution() -> None:
    controller = PIDController(kp=0.0, ki=0.0, kd=0.0)

    controller.update(target=10.0, current=0.0, dt=1.0)
    output = controller.update(target=0.0, current=0.0, dt=0.5)

    assert output == pytest.approx(0.0)


def test_combined_pid_sequence_with_manual_expected_values() -> None:
    controller = PIDController(kp=2.0, ki=0.5, kd=0.25)

    first_output = controller.update(target=10.0, current=6.0, dt=0.5)
    second_output = controller.update(target=8.0, current=5.0, dt=0.5)

    assert first_output == pytest.approx(9.0)
    assert second_output == pytest.approx(7.25)


def test_reset_clears_integral_state() -> None:
    controller = PIDController(kp=0.0, ki=1.0, kd=0.0)

    controller.update(target=2.0, current=0.0, dt=1.0)
    controller.update(target=2.0, current=0.0, dt=1.0)
    controller.reset()
    output = controller.update(target=2.0, current=0.0, dt=1.0)

    assert output == pytest.approx(2.0)


def test_reset_clears_previous_error_state() -> None:
    controller = PIDController(kp=0.0, ki=0.0, kd=1.0)

    controller.update(target=10.0, current=0.0, dt=1.0)
    controller.reset()
    output = controller.update(target=2.0, current=0.0, dt=1.0)

    assert output == pytest.approx(0.0)


def test_first_update_after_reset_has_zero_derivative_contribution() -> None:
    controller = PIDController(kp=1.0, ki=0.0, kd=2.0)

    controller.update(target=10.0, current=0.0, dt=1.0)
    controller.reset()
    output = controller.update(target=4.0, current=1.0, dt=0.5)

    assert output == pytest.approx(3.0)


def test_reset_does_not_modify_gains() -> None:
    controller = PIDController(kp=1.0, ki=2.0, kd=3.0)

    controller.update(target=5.0, current=1.0, dt=1.0)
    controller.reset()

    assert controller.kp == pytest.approx(1.0)
    assert controller.ki == pytest.approx(2.0)
    assert controller.kd == pytest.approx(3.0)


def test_behaviour_after_reset_matches_new_controller() -> None:
    controller = PIDController(kp=1.0, ki=0.5, kd=2.0)
    new_controller = PIDController(kp=1.0, ki=0.5, kd=2.0)

    controller.update(target=10.0, current=0.0, dt=1.0)
    controller.update(target=5.0, current=0.0, dt=1.0)
    controller.reset()

    reset_output = controller.update(target=4.0, current=1.0, dt=2.0)
    new_output = new_controller.update(target=4.0, current=1.0, dt=2.0)

    assert reset_output == pytest.approx(new_output)


def test_controller_instances_maintain_independent_state() -> None:
    first_controller = PIDController(kp=0.0, ki=1.0, kd=1.0)
    second_controller = PIDController(kp=0.0, ki=1.0, kd=1.0)

    first_controller.update(target=4.0, current=0.0, dt=1.0)
    first_output = first_controller.update(target=2.0, current=0.0, dt=1.0)
    second_output = second_controller.update(target=2.0, current=0.0, dt=1.0)

    assert first_output == pytest.approx(4.0)
    assert second_output == pytest.approx(2.0)


@pytest.mark.parametrize(
    ("kp", "ki", "kd", "message"),
    [
        (-1.0, 0.0, 0.0, "kp must be greater than or equal to zero"),
        (0.0, -1.0, 0.0, "ki must be greater than or equal to zero"),
        (0.0, 0.0, -1.0, "kd must be greater than or equal to zero"),
    ],
)
def test_negative_gain_raises_value_error(kp: float, ki: float, kd: float, message: str) -> None:
    with pytest.raises(ValueError, match=message):
        PIDController(kp=kp, ki=ki, kd=kd)


def test_zero_gains_are_valid() -> None:
    controller = PIDController(kp=0.0, ki=0.0, kd=0.0)

    output = controller.update(target=1.0, current=0.0, dt=1.0)

    assert output == pytest.approx(0.0)


@pytest.mark.parametrize("dt", [0.0, -1.0])
def test_invalid_dt_raises_value_error(dt: float) -> None:
    controller = PIDController(kp=1.0, ki=1.0, kd=1.0)

    with pytest.raises(ValueError, match="dt must be greater than zero"):
        controller.update(target=1.0, current=0.0, dt=dt)
