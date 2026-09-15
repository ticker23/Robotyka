import math

import pytest

from anti_drone.controller import PIDController
from anti_drone.drone import Drone
from anti_drone.kinematics import forward_kinematics, inverse_kinematics
from anti_drone.robot import Robot
from anti_drone.sensor import Sensor, TargetTracker
from anti_drone.simulation import Simulation, SimulationState, SimulationStep
from anti_drone.trajectory import LinearTrajectory


class RecordingTrajectory:
    def __init__(self) -> None:
        self.calls: list[float] = []

    def get_position(self, time: float) -> tuple[float, float]:
        self.calls.append(time)
        return time, 0.0


class FixedMeasurementSensor:
    def __init__(self, measurement: tuple[float, float] | None) -> None:
        self.measurement = measurement
        self.calls: list[tuple[float, float]] = []

    def measure(self, target_position: tuple[float, float]) -> tuple[float, float] | None:
        self.calls.append(target_position)
        return self.measurement


class SequenceMeasurementSensor:
    def __init__(self, measurements: list[tuple[float, float] | None]) -> None:
        self.measurements = measurements
        self.calls: list[tuple[float, float]] = []

    def measure(self, target_position: tuple[float, float]) -> tuple[float, float] | None:
        self.calls.append(target_position)
        return self.measurements.pop(0)


class RecordingPID:
    def __init__(self, output: float) -> None:
        self.output = output
        self.calls: list[tuple[float, float, float]] = []

    def update(self, target: float, current: float, dt: float) -> float:
        self.calls.append((target, current, dt))
        return self.output


class RecordingTracker:
    def __init__(self, prediction: tuple[float, float] | None) -> None:
        self.prediction_position = prediction
        self.update_calls: list[tuple[tuple[float, float], float]] = []
        self.prediction_calls: list[float] = []

    def update(self, measured_position: tuple[float, float], dt: float) -> None:
        self.update_calls.append((measured_position, dt))

    def prediction(self, prediction_time: float) -> tuple[float, float] | None:
        self.prediction_calls.append(prediction_time)
        return self.prediction_position


def make_simulation(
    *,
    drone: Drone | None = None,
    sensor: Sensor | FixedMeasurementSensor | SequenceMeasurementSensor | None = None,
    tracker: TargetTracker | RecordingTracker | None = None,
    robot: Robot | None = None,
    pid_q1: PIDController | RecordingPID | None = None,
    pid_q2: PIDController | RecordingPID | None = None,
) -> Simulation:
    return Simulation(
        drone=drone or Drone(LinearTrajectory(start_position=(0.0, 0.0), velocity=(0.0, 0.0))),
        sensor=sensor or Sensor(position=(0.0, 0.0), detection_range=10.0),
        tracker=tracker or TargetTracker(),
        robot=robot or Robot(link_1=1.0, link_2=1.0),
        pid_q1=pid_q1 or PIDController(kp=1.0, ki=0.0, kd=0.0),
        pid_q2=pid_q2 or PIDController(kp=1.0, ki=0.0, kd=0.0),
    )


def assert_position(actual: tuple[float, float] | None, expected: tuple[float, float]) -> None:
    assert actual is not None
    assert actual[0] == pytest.approx(expected[0])
    assert actual[1] == pytest.approx(expected[1])


def test_simulation_starts_at_zero_and_uses_supplied_instances() -> None:
    drone = Drone(LinearTrajectory(start_position=(0.0, 0.0), velocity=(0.0, 0.0)))
    sensor = Sensor(position=(0.0, 0.0), detection_range=10.0)
    tracker = TargetTracker()
    robot = Robot(link_1=1.0, link_2=1.0)
    pid_q1 = PIDController(kp=1.0, ki=0.0, kd=0.0)
    pid_q2 = PIDController(kp=1.0, ki=0.0, kd=0.0)

    simulation = Simulation(
        drone=drone,
        sensor=sensor,
        tracker=tracker,
        robot=robot,
        pid_q1=pid_q1,
        pid_q2=pid_q2,
    )

    assert simulation.time == pytest.approx(0.0)
    assert simulation.drone is drone
    assert simulation.sensor is sensor
    assert simulation.tracker is tracker
    assert simulation.robot is robot
    assert simulation.pid_q1 is pid_q1
    assert simulation.pid_q2 is pid_q2


def test_valid_steps_accumulate_simulation_time() -> None:
    simulation = make_simulation(sensor=FixedMeasurementSensor(None))

    first_step = simulation.step(0.1, prediction_time=0.2)
    second_step = simulation.step(0.1, prediction_time=0.2)
    third_step = simulation.step(0.3, prediction_time=0.2)

    assert first_step.time == pytest.approx(0.1)
    assert second_step.time == pytest.approx(0.2)
    assert third_step.time == pytest.approx(0.5)
    assert simulation.time == pytest.approx(0.5)


@pytest.mark.parametrize("dt", [0.0, -0.1])
def test_invalid_dt_raises_before_state_changes(dt: float) -> None:
    trajectory = RecordingTrajectory()
    drone = Drone(trajectory)
    sensor = FixedMeasurementSensor((1.0, 0.0))
    tracker = RecordingTracker(prediction=(1.0, 0.0))
    robot = Robot(link_1=1.0, link_2=1.0, q1=0.25, q2=-0.5)
    pid_q1 = RecordingPID(output=1.0)
    pid_q2 = RecordingPID(output=1.0)
    simulation = make_simulation(
        drone=drone,
        sensor=sensor,
        tracker=tracker,
        robot=robot,
        pid_q1=pid_q1,
        pid_q2=pid_q2,
    )

    with pytest.raises(ValueError, match="dt must be greater than zero"):
        simulation.step(dt, prediction_time=0.2)

    assert simulation.time == pytest.approx(0.0)
    assert trajectory.calls == [0.0]
    assert sensor.calls == []
    assert tracker.update_calls == []
    assert tracker.prediction_calls == []
    assert pid_q1.calls == []
    assert pid_q2.calls == []
    assert robot.q1 == pytest.approx(0.25)
    assert robot.q2 == pytest.approx(-0.5)


def test_drone_updates_using_new_absolute_simulation_time() -> None:
    trajectory = RecordingTrajectory()
    drone = Drone(trajectory)
    simulation = make_simulation(drone=drone, sensor=FixedMeasurementSensor(None))

    first_step = simulation.step(0.5, prediction_time=0.1)
    second_step = simulation.step(0.5, prediction_time=0.1)

    assert trajectory.calls == [0.0, 0.5, 1.0]
    assert_position(first_step.drone_position, (0.5, 0.0))
    assert_position(second_step.drone_position, (1.0, 0.0))


def test_no_target_skips_tracker_prediction_and_tracking_pipeline() -> None:
    tracker = RecordingTracker(prediction=(1.0, 1.0))
    robot = Robot(link_1=1.0, link_2=1.0, q1=0.5, q2=-0.25)
    pid_q1 = RecordingPID(output=10.0)
    pid_q2 = RecordingPID(output=10.0)
    simulation = make_simulation(
        sensor=FixedMeasurementSensor(None),
        tracker=tracker,
        robot=robot,
        pid_q1=pid_q1,
        pid_q2=pid_q2,
    )

    step = simulation.step(0.2, prediction_time=0.5)

    assert step.state is SimulationState.NO_TARGET
    assert step.measured_position is None
    assert step.target_joint_angles is None
    assert tracker.update_calls == []
    assert tracker.prediction_calls == []
    assert_position(step.robot_joint_angles, (0.5, -0.25))
    assert pid_q1.calls == []
    assert pid_q2.calls == []
    assert_position(step.end_effector_position, forward_kinematics(0.5, -0.25, 1.0, 1.0))


def test_first_measurement_updates_tracker_and_falls_back_to_measured_position() -> None:
    tracker = TargetTracker()
    simulation = make_simulation(
        sensor=FixedMeasurementSensor((1.0, 1.0)),
        tracker=tracker,
        pid_q1=RecordingPID(output=0.0),
        pid_q2=RecordingPID(output=0.0),
    )

    step = simulation.step(0.5, prediction_time=2.0)

    assert step.state is SimulationState.TRACKING
    assert_position(step.measured_position, (1.0, 1.0))
    assert tracker.current_position == pytest.approx((1.0, 1.0))
    assert tracker.previous_position is None
    assert tracker.velocity is None
    assert_position(step.target_joint_angles, inverse_kinematics(1.0, 1.0, 1.0, 1.0))


def test_subsequent_measurements_use_predicted_position_for_inverse_kinematics() -> None:
    tracker = TargetTracker()
    sensor = SequenceMeasurementSensor([(1.0, 0.0), (1.5, 0.0)])
    simulation = make_simulation(
        sensor=sensor,
        tracker=tracker,
        pid_q1=RecordingPID(output=0.0),
        pid_q2=RecordingPID(output=0.0),
    )

    simulation.step(1.0, prediction_time=1.0)
    step = simulation.step(1.0, prediction_time=1.0)

    assert tracker.velocity == pytest.approx((0.5, 0.0))
    assert_position(step.measured_position, (1.5, 0.0))
    assert_position(step.target_joint_angles, inverse_kinematics(2.0, 0.0, 1.0, 1.0))
    assert step.target_joint_angles != pytest.approx(inverse_kinematics(1.5, 0.0, 1.0, 1.0))


def test_predicted_unreachable_position_skips_ik_pid_and_robot_update() -> None:
    tracker = RecordingTracker(prediction=(3.0, 0.0))
    robot = Robot(link_1=1.0, link_2=1.0, q1=0.5, q2=-0.25)
    pid_q1 = RecordingPID(output=10.0)
    pid_q2 = RecordingPID(output=10.0)
    simulation = make_simulation(
        sensor=FixedMeasurementSensor((1.0, 0.0)),
        tracker=tracker,
        robot=robot,
        pid_q1=pid_q1,
        pid_q2=pid_q2,
    )

    step = simulation.step(0.2, prediction_time=0.5)

    assert step.state is SimulationState.TARGET_UNREACHABLE
    assert_position(step.measured_position, (1.0, 0.0))
    assert step.target_joint_angles is None
    assert tracker.update_calls == [((1.0, 0.0), 0.2)]
    assert tracker.prediction_calls == [0.5]
    assert_position(step.robot_joint_angles, (0.5, -0.25))
    assert pid_q1.calls == []
    assert pid_q2.calls == []


def test_predicted_reachable_position_is_used_even_when_measurement_is_unreachable() -> None:
    tracker = RecordingTracker(prediction=(1.0, 1.0))
    simulation = make_simulation(
        sensor=FixedMeasurementSensor((3.0, 0.0)),
        tracker=tracker,
        pid_q1=RecordingPID(output=0.0),
        pid_q2=RecordingPID(output=0.0),
    )

    step = simulation.step(0.2, prediction_time=0.5)

    assert step.state is SimulationState.TRACKING
    assert_position(step.measured_position, (3.0, 0.0))
    assert tracker.update_calls == [((3.0, 0.0), 0.2)]
    assert tracker.prediction_calls == [0.5]
    assert_position(step.target_joint_angles, inverse_kinematics(1.0, 1.0, 1.0, 1.0))


def test_tracking_updates_pid_and_robot_from_predicted_position() -> None:
    tracker = RecordingTracker(prediction=(1.0, 1.0))
    robot = Robot(link_1=1.0, link_2=1.0)
    pid_q1 = RecordingPID(output=0.5)
    pid_q2 = RecordingPID(output=-0.25)
    simulation = make_simulation(
        sensor=FixedMeasurementSensor((0.5, 0.5)),
        tracker=tracker,
        robot=robot,
        pid_q1=pid_q1,
        pid_q2=pid_q2,
    )

    step = simulation.step(0.4, prediction_time=0.75)
    expected_targets = inverse_kinematics(1.0, 1.0, 1.0, 1.0)

    assert isinstance(step, SimulationStep)
    assert step.state is SimulationState.TRACKING
    assert step.time == pytest.approx(0.4)
    assert_position(step.target_joint_angles, expected_targets)
    assert pid_q1.calls == [(expected_targets[0], 0.0, 0.4)]
    assert pid_q2.calls == [(expected_targets[1], 0.0, 0.4)]
    assert robot.q1 == pytest.approx(0.2)
    assert robot.q2 == pytest.approx(-0.1)
    assert_position(step.robot_joint_angles, (0.2, -0.1))
    assert_position(
        step.end_effector_position,
        forward_kinematics(robot.q1, robot.q2, robot.link_1, robot.link_2),
    )


def test_simulation_step_snapshot_contains_completed_step_data() -> None:
    tracker = RecordingTracker(prediction=(1.0, 1.0))
    robot = Robot(link_1=1.0, link_2=1.0)
    simulation = make_simulation(
        sensor=FixedMeasurementSensor((0.5, 0.5)),
        tracker=tracker,
        robot=robot,
        pid_q1=RecordingPID(output=0.5),
        pid_q2=RecordingPID(output=0.25),
    )

    step = simulation.step(0.5, prediction_time=0.25)

    assert step.time == pytest.approx(0.5)
    assert step.state is SimulationState.TRACKING
    assert_position(step.drone_position, (0.0, 0.0))
    assert_position(step.measured_position, (0.5, 0.5))
    assert_position(step.target_joint_angles, inverse_kinematics(1.0, 1.0, 1.0, 1.0))
    assert_position(step.robot_joint_angles, (0.25, 0.125))
    assert_position(step.end_effector_position, forward_kinematics(0.25, 0.125, 1.0, 1.0))


def test_pid_state_changes_only_during_tracking() -> None:
    pid = PIDController(kp=0.0, ki=1.0, kd=0.0)

    tracking_simulation = make_simulation(
        sensor=FixedMeasurementSensor((1.0, 1.0)),
        tracker=RecordingTracker(prediction=None),
        pid_q2=pid,
    )
    tracking_simulation.step(0.5, prediction_time=0.2)
    tracking_output = pid.update(target=0.0, current=0.0, dt=1.0)
    assert tracking_output == pytest.approx(math.pi / 4)

    no_target_pid = PIDController(kp=0.0, ki=1.0, kd=0.0)
    no_target_simulation = make_simulation(
        sensor=FixedMeasurementSensor(None),
        tracker=RecordingTracker(prediction=(1.0, 1.0)),
        pid_q2=no_target_pid,
    )
    no_target_simulation.step(0.5, prediction_time=0.2)
    no_target_output = no_target_pid.update(target=0.0, current=0.0, dt=1.0)
    assert no_target_output == pytest.approx(0.0)

    unreachable_pid = PIDController(kp=0.0, ki=1.0, kd=0.0)
    unreachable_simulation = make_simulation(
        sensor=FixedMeasurementSensor((1.0, 1.0)),
        tracker=RecordingTracker(prediction=(3.0, 0.0)),
        pid_q2=unreachable_pid,
    )
    unreachable_simulation.step(0.5, prediction_time=0.2)
    unreachable_output = unreachable_pid.update(target=0.0, current=0.0, dt=1.0)
    assert unreachable_output == pytest.approx(0.0)


def test_simulation_step_snapshot_remains_unchanged_after_later_steps() -> None:
    simulation = make_simulation(
        sensor=FixedMeasurementSensor((1.0, 1.0)),
        tracker=RecordingTracker(prediction=None),
    )

    first_step = simulation.step(0.5, prediction_time=0.2)
    first_robot_joint_angles = first_step.robot_joint_angles
    second_step = simulation.step(0.5, prediction_time=0.2)

    assert first_step.time == pytest.approx(0.5)
    assert first_step.robot_joint_angles == first_robot_joint_angles
    assert second_step.time == pytest.approx(1.0)
    assert first_step.robot_joint_angles != pytest.approx(second_step.robot_joint_angles)


def test_deterministic_end_to_end_tracking_pipeline_with_real_components() -> None:
    drone = Drone(LinearTrajectory(start_position=(0.0, 0.0), velocity=(1.0, 1.0)))
    sensor = Sensor(position=(0.0, 0.0), detection_range=10.0, noise_std=0.0)
    tracker = TargetTracker()
    robot = Robot(link_1=1.0, link_2=1.0)
    pid_q1 = PIDController(kp=1.0, ki=0.0, kd=0.0)
    pid_q2 = PIDController(kp=1.0, ki=0.0, kd=0.0)
    simulation = Simulation(
        drone=drone,
        sensor=sensor,
        tracker=tracker,
        robot=robot,
        pid_q1=pid_q1,
        pid_q2=pid_q2,
    )

    step = simulation.step(1.0, prediction_time=0.25)

    assert step.state is SimulationState.TRACKING
    assert_position(step.drone_position, (1.0, 1.0))
    assert_position(step.measured_position, (1.0, 1.0))
    assert_position(step.target_joint_angles, (0.0, math.pi / 2))
    assert_position(step.robot_joint_angles, (0.0, math.pi / 2))
    assert_position(step.end_effector_position, (1.0, 1.0))
