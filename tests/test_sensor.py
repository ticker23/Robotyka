import pytest

from anti_drone import sensor as sensor_module
from anti_drone.sensor import Sensor, TargetTracker


def test_sensor_initializes_with_valid_configuration() -> None:
    sensor = Sensor(position=(-1.0, 2.0), detection_range=5.0, noise_std=0.25)

    assert sensor.position == pytest.approx((-1.0, 2.0))
    assert sensor.detection_range == pytest.approx(5.0)
    assert sensor.noise_std == pytest.approx(0.25)


@pytest.mark.parametrize(
    ("sensor", "target_position", "expected"),
    [
        (Sensor(position=(0.0, 0.0), detection_range=5.0), (0.0, 0.0), True),
        (Sensor(position=(0.0, 0.0), detection_range=5.0), (3.0, 4.0), True),
        (Sensor(position=(0.0, 0.0), detection_range=5.0), (5.0, 0.0), True),
        (Sensor(position=(0.0, 0.0), detection_range=5.0), (5.01, 0.0), False),
        (Sensor(position=(2.0, -1.0), detection_range=3.0), (4.0, 1.0), True),
        (Sensor(position=(2.0, -1.0), detection_range=3.0), (5.0, 2.0), False),
    ],
)
def test_is_detected(sensor: Sensor, target_position: tuple[float, float], expected: bool) -> None:
    assert sensor.is_detected(target_position) is expected


def test_is_detected_does_not_modify_sensor_state() -> None:
    sensor = Sensor(position=(1.0, 2.0), detection_range=3.0, noise_std=0.5)

    first_result = sensor.is_detected((2.0, 3.0))
    second_result = sensor.is_detected((2.0, 3.0))

    assert first_result is True
    assert second_result is True
    assert sensor.position == pytest.approx((1.0, 2.0))
    assert sensor.detection_range == pytest.approx(3.0)
    assert sensor.noise_std == pytest.approx(0.5)


def test_measure_detected_target_returns_true_position_without_noise() -> None:
    sensor = Sensor(position=(0.0, 0.0), detection_range=10.0)
    target_position = (3.0, -4.0)

    measurement = sensor.measure(target_position)

    assert measurement == pytest.approx(target_position)


def test_measure_undetected_target_returns_none() -> None:
    sensor = Sensor(position=(0.0, 0.0), detection_range=1.0)

    assert sensor.measure((2.0, 0.0)) is None


def test_measure_does_not_modify_input_position() -> None:
    sensor = Sensor(position=(0.0, 0.0), detection_range=10.0)
    target_position = (1.0, 2.0)

    sensor.measure(target_position)

    assert target_position == pytest.approx((1.0, 2.0))


def test_measure_applies_gaussian_noise(monkeypatch: pytest.MonkeyPatch) -> None:
    noise_values = iter([0.25, -0.5])

    def fake_gauss(mean: float, standard_deviation: float) -> float:
        assert mean == pytest.approx(0.0)
        assert standard_deviation == pytest.approx(0.1)
        return next(noise_values)

    monkeypatch.setattr(sensor_module, "gauss", fake_gauss)
    sensor = Sensor(position=(0.0, 0.0), detection_range=10.0, noise_std=0.1)

    measurement = sensor.measure((2.0, 3.0))

    assert measurement == pytest.approx((2.25, 2.5))


def test_measure_applies_independent_noise_to_each_coordinate(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    calls: list[tuple[float, float]] = []
    noise_values = iter([1.0, 2.0])

    def fake_gauss(mean: float, standard_deviation: float) -> float:
        calls.append((mean, standard_deviation))
        return next(noise_values)

    monkeypatch.setattr(sensor_module, "gauss", fake_gauss)
    sensor = Sensor(position=(0.0, 0.0), detection_range=10.0, noise_std=0.3)

    measurement = sensor.measure((5.0, 6.0))

    assert measurement == pytest.approx((6.0, 8.0))
    assert calls == [(0.0, 0.3), (0.0, 0.3)]


def test_measure_with_noise_does_not_modify_input_position(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.setattr(sensor_module, "gauss", lambda _mean, _standard_deviation: 1.0)
    sensor = Sensor(position=(0.0, 0.0), detection_range=10.0, noise_std=0.5)
    target_position = (1.0, 2.0)

    sensor.measure(target_position)

    assert target_position == pytest.approx((1.0, 2.0))


@pytest.mark.parametrize("detection_range", [0.0, -1.0])
def test_invalid_detection_range_raises_value_error(detection_range: float) -> None:
    with pytest.raises(ValueError, match="detection_range must be greater than zero"):
        Sensor(position=(0.0, 0.0), detection_range=detection_range)


def test_negative_noise_std_raises_value_error() -> None:
    with pytest.raises(ValueError, match="noise_std must be greater than or equal to zero"):
        Sensor(position=(0.0, 0.0), detection_range=1.0, noise_std=-0.1)


def test_zero_noise_std_is_valid() -> None:
    sensor = Sensor(position=(0.0, 0.0), detection_range=1.0, noise_std=0.0)

    assert sensor.noise_std == pytest.approx(0.0)


def test_target_tracker_initial_state_has_no_positions_or_velocity() -> None:
    tracker = TargetTracker()

    assert tracker.current_position is None
    assert tracker.previous_position is None
    assert tracker.velocity is None


def test_target_tracker_first_update_stores_current_position() -> None:
    tracker = TargetTracker()

    tracker.update((1.0, -2.0), dt=0.1)

    assert tracker.current_position == pytest.approx((1.0, -2.0))


def test_target_tracker_first_update_keeps_previous_position_none() -> None:
    tracker = TargetTracker()

    tracker.update((1.0, -2.0), dt=0.1)

    assert tracker.previous_position is None


def test_target_tracker_first_update_does_not_calculate_velocity() -> None:
    tracker = TargetTracker()

    tracker.update((1.0, -2.0), dt=0.1)

    assert tracker.velocity is None


def test_target_tracker_second_update_stores_previous_position() -> None:
    tracker = TargetTracker()
    tracker.update((1.0, 1.0), dt=0.1)

    tracker.update((1.2, 1.1), dt=0.1)

    assert tracker.previous_position == pytest.approx((1.0, 1.0))


def test_target_tracker_second_update_stores_new_current_position() -> None:
    tracker = TargetTracker()
    tracker.update((1.0, 1.0), dt=0.1)

    tracker.update((1.2, 1.1), dt=0.1)

    assert tracker.current_position == pytest.approx((1.2, 1.1))


def test_target_tracker_second_update_calculates_velocity() -> None:
    tracker = TargetTracker()
    tracker.update((1.0, 1.0), dt=0.1)

    tracker.update((1.2, 1.1), dt=0.1)

    assert tracker.velocity == pytest.approx((2.0, 1.0))


def test_target_tracker_update_calculates_velocity_for_negative_positions() -> None:
    tracker = TargetTracker()
    tracker.update((-2.0, -1.0), dt=0.5)

    tracker.update((-1.0, -3.0), dt=0.5)

    assert tracker.velocity == pytest.approx((2.0, -4.0))


def test_target_tracker_update_calculates_zero_velocity_for_unchanged_position() -> None:
    tracker = TargetTracker()
    tracker.update((0.0, 0.0), dt=1.0)

    tracker.update((0.0, 0.0), dt=1.0)

    assert tracker.velocity == pytest.approx((0.0, 0.0))


def test_target_tracker_repeated_updates_shift_previous_position() -> None:
    tracker = TargetTracker()
    tracker.update((0.0, 0.0), dt=1.0)
    tracker.update((1.0, 2.0), dt=1.0)

    tracker.update((3.0, 1.0), dt=2.0)

    assert tracker.previous_position == pytest.approx((1.0, 2.0))


def test_target_tracker_repeated_updates_replace_current_position() -> None:
    tracker = TargetTracker()
    tracker.update((0.0, 0.0), dt=1.0)
    tracker.update((1.0, 2.0), dt=1.0)

    tracker.update((3.0, 1.0), dt=2.0)

    assert tracker.current_position == pytest.approx((3.0, 1.0))


def test_target_tracker_repeated_updates_recalculate_velocity() -> None:
    tracker = TargetTracker()
    tracker.update((0.0, 0.0), dt=1.0)
    tracker.update((1.0, 2.0), dt=1.0)

    tracker.update((3.0, 1.0), dt=2.0)

    assert tracker.velocity == pytest.approx((1.0, -0.5))


@pytest.mark.parametrize("dt", [0.0, -0.1])
def test_target_tracker_invalid_dt_raises_value_error(dt: float) -> None:
    tracker = TargetTracker()

    with pytest.raises(ValueError, match="dt must be greater than zero"):
        tracker.update((1.0, 1.0), dt=dt)


def test_target_tracker_prediction_before_velocity_raises_value_error() -> None:
    tracker = TargetTracker()
    tracker.update((12.0, 8.0), dt=1.0)

    with pytest.raises(ValueError, match="current_position and velocity must be available"):
        tracker.prediction(1.0)


def test_target_tracker_prediction_zero_time_returns_current_position() -> None:
    tracker = TargetTracker()
    tracker.update((10.0, 5.0), dt=1.0)
    tracker.update((12.0, 8.0), dt=1.0)

    prediction = tracker.prediction(0.0)

    assert prediction == pytest.approx((12.0, 8.0))


def test_target_tracker_prediction_uses_current_position_and_velocity() -> None:
    tracker = TargetTracker()
    tracker.update((10.0, 5.0), dt=1.0)
    tracker.update((12.0, 8.0), dt=1.0)

    prediction = tracker.prediction(2.0)

    assert prediction == pytest.approx((16.0, 14.0))


def test_target_tracker_negative_prediction_time_raises_value_error() -> None:
    tracker = TargetTracker()
    tracker.update((10.0, 5.0), dt=1.0)
    tracker.update((12.0, 8.0), dt=1.0)

    with pytest.raises(ValueError, match="prediction_time must not be negative"):
        tracker.prediction(-0.1)
