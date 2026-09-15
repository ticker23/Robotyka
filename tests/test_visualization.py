import math

import matplotlib

matplotlib.use("Agg")

import matplotlib.pyplot as plt
import pytest

from anti_drone.controller import PIDController
from anti_drone.drone import Drone
from anti_drone.robot import Robot
from anti_drone.sensor import Sensor, TargetTracker
from anti_drone.simulation import Simulation, SimulationState, SimulationStep
from anti_drone.trajectory import LinearTrajectory
from anti_drone.visualization import Visualization


@pytest.fixture(autouse=True)
def close_figures() -> None:
    yield
    plt.close("all")


def make_step(
    *,
    state: SimulationState = SimulationState.TRACKING,
    time: float = 1.25,
    drone_position: tuple[float, float] = (2.0, 1.5),
    measured_position: tuple[float, float] | None = (1.8, 1.25),
    target_joint_angles: tuple[float, float] | None = (0.2, 0.3),
    robot_joint_angles: tuple[float, float] = (0.0, math.pi / 2),
    end_effector_position: tuple[float, float] = (1.0, 1.0),
) -> SimulationStep:
    return SimulationStep(
        time=time,
        state=state,
        drone_position=drone_position,
        measured_position=measured_position,
        target_joint_angles=target_joint_angles,
        robot_joint_angles=robot_joint_angles,
        end_effector_position=end_effector_position,
    )


def assert_artist_position(artist, expected: tuple[float, float]) -> None:
    assert list(artist.get_xdata()) == pytest.approx([expected[0]])
    assert list(artist.get_ydata()) == pytest.approx([expected[1]])


@pytest.mark.parametrize(
    ("link_1", "q1", "expected_joint"),
    [
        (1.0, 0.0, (1.0, 0.0)),
        (1.0, math.pi / 2, (0.0, 1.0)),
        (2.0, math.pi, (-2.0, 0.0)),
    ],
)
def test_intermediate_joint_geometry(
    link_1: float,
    q1: float,
    expected_joint: tuple[float, float],
) -> None:
    robot = Robot(link_1=link_1, link_2=1.0)
    visualization = Visualization(
        robot=robot, sensor=Sensor(position=(0.0, 0.0), detection_range=3.0)
    )

    visualization.update(make_step(robot_joint_angles=(q1, 0.0)))

    assert_artist_position(visualization.joint_marker, expected_joint)


def test_tracking_update_sets_expected_artist_data() -> None:
    robot = Robot(link_1=1.0, link_2=1.0)
    visualization = Visualization(
        robot=robot, sensor=Sensor(position=(0.0, 0.0), detection_range=3.0)
    )
    step = make_step(
        state=SimulationState.TRACKING,
        time=2.4,
        drone_position=(2.0, 1.0),
        measured_position=(1.5, 0.5),
        target_joint_angles=(math.pi, math.pi),
        robot_joint_angles=(0.0, math.pi / 2),
        end_effector_position=(1.0, 1.0),
    )

    visualization.update(step)

    assert list(visualization.robot_line.get_xdata()) == pytest.approx([0.0, 1.0, 1.0])
    assert list(visualization.robot_line.get_ydata()) == pytest.approx([0.0, 0.0, 1.0])
    assert_artist_position(visualization.end_effector_marker, (1.0, 1.0))
    assert_artist_position(visualization.drone_marker, (2.0, 1.0))
    assert_artist_position(visualization.measurement_marker, (1.5, 0.5))
    assert visualization.measurement_marker.get_visible() is True
    assert visualization.time_text.get_text() == "Time: 2.40 s"
    assert visualization.state_text.get_text() == "State: TRACKING"


def test_no_target_hides_stale_measurement_marker() -> None:
    robot = Robot(link_1=1.0, link_2=1.0)
    visualization = Visualization(
        robot=robot, sensor=Sensor(position=(0.0, 0.0), detection_range=3.0)
    )

    visualization.update(make_step(measured_position=(1.0, 1.0)))
    visualization.update(
        make_step(
            state=SimulationState.NO_TARGET,
            measured_position=None,
            target_joint_angles=None,
            drone_position=(2.0, -1.0),
        )
    )

    assert_artist_position(visualization.drone_marker, (2.0, -1.0))
    assert visualization.measurement_marker.get_visible() is False
    assert list(visualization.measurement_marker.get_xdata()) == []
    assert list(visualization.measurement_marker.get_ydata()) == []
    assert visualization.state_text.get_text() == "State: NO_TARGET"
    assert list(visualization.robot_line.get_xdata())


def test_target_unreachable_shows_measurement_and_current_robot_state() -> None:
    robot = Robot(link_1=1.0, link_2=1.0)
    visualization = Visualization(
        robot=robot, sensor=Sensor(position=(0.0, 0.0), detection_range=5.0)
    )
    step = make_step(
        state=SimulationState.TARGET_UNREACHABLE,
        measured_position=(3.0, 0.0),
        target_joint_angles=None,
        robot_joint_angles=(math.pi / 2, 0.0),
        end_effector_position=(0.0, 2.0),
    )

    visualization.update(step)

    assert_artist_position(visualization.drone_marker, step.drone_position)
    assert_artist_position(visualization.measurement_marker, (3.0, 0.0))
    assert visualization.measurement_marker.get_visible() is True
    assert list(visualization.robot_line.get_xdata()) == pytest.approx([0.0, 0.0, 0.0])
    assert list(visualization.robot_line.get_ydata()) == pytest.approx([0.0, 1.0, 2.0])
    assert visualization.state_text.get_text() == "State: TARGET_UNREACHABLE"


def test_sensor_static_artists_use_sensor_configuration() -> None:
    sensor = Sensor(position=(1.5, -0.5), detection_range=2.25)
    visualization = Visualization(robot=Robot(link_1=1.0, link_2=1.0), sensor=sensor)

    assert_artist_position(visualization.sensor_marker, (1.5, -0.5))
    assert visualization.sensor_range_circle.center == pytest.approx((1.5, -0.5))
    assert visualization.sensor_range_circle.radius == pytest.approx(2.25)


def test_axis_uses_equal_scaling() -> None:
    visualization = Visualization(
        robot=Robot(link_1=1.0, link_2=1.0),
        sensor=Sensor(position=(0.0, 0.0), detection_range=3.0),
    )

    assert visualization.ax.get_aspect() in ("equal", 1.0)


def test_update_smoke_test() -> None:
    visualization = Visualization(
        robot=Robot(link_1=1.0, link_2=1.0),
        sensor=Sensor(position=(0.0, 0.0), detection_range=3.0),
    )

    visualization.update(make_step())


def test_visualization_consumes_real_simulation_step() -> None:
    drone = Drone(LinearTrajectory(start_position=(0.0, 0.0), velocity=(1.0, 1.0)))
    sensor = Sensor(position=(0.0, 0.0), detection_range=10.0, noise_std=0.0)
    robot = Robot(link_1=1.0, link_2=1.0)
    simulation = Simulation(
        drone=drone,
        sensor=sensor,
        tracker=TargetTracker(),
        robot=robot,
        pid_q1=PIDController(kp=1.0, ki=0.0, kd=0.0),
        pid_q2=PIDController(kp=1.0, ki=0.0, kd=0.0),
    )
    visualization = Visualization(robot=robot, sensor=sensor)

    step = simulation.step(1.0, prediction_time=0.25)
    visualization.update(step)

    assert visualization.state_text.get_text() == "State: TRACKING"
    assert_artist_position(visualization.drone_marker, step.drone_position)
