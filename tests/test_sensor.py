import pytest

from anti_drone import sensor as sensor_module
from anti_drone.sensor import Sensor


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
