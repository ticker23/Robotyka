"""Simulation orchestration for the anti-drone MVP pipeline."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum, auto

from anti_drone.controller import PIDController
from anti_drone.drone import Drone
from anti_drone.kinematics import forward_kinematics, inverse_kinematics, is_reachable
from anti_drone.robot import Robot
from anti_drone.sensor import Sensor, TargetTracker


class SimulationState(Enum):
    """Logical state of a completed simulation step."""

    NO_TARGET = auto()
    TARGET_UNREACHABLE = auto()
    TRACKING = auto()


@dataclass(frozen=True, slots=True)
class SimulationStep:
    """Immutable snapshot of one completed simulation step."""

    time: float
    state: SimulationState
    drone_position: tuple[float, float]
    measured_position: tuple[float, float] | None
    target_joint_angles: tuple[float, float] | None
    robot_joint_angles: tuple[float, float]
    end_effector_position: tuple[float, float]


class Simulation:
    """Coordinate drone, sensor, kinematics, controllers, and robot state."""

    def __init__(
        self,
        drone: Drone,
        sensor: Sensor,
        tracker: TargetTracker,
        robot: Robot,
        pid_q1: PIDController,
        pid_q2: PIDController,
    ) -> None:
        self.drone = drone
        self.sensor = sensor
        self.tracker = tracker
        self.robot = robot
        self.pid_q1 = pid_q1
        self.pid_q2 = pid_q2
        self.time = 0.0

    def step(self, dt: float, prediction_time: float) -> SimulationStep:
        """Advance the simulation by one positive timestep and return a snapshot."""

        if dt <= 0:
            raise ValueError("dt must be greater than zero")

        self.time += dt
        self.drone.update(self.time)

        drone_position = self.drone.position
        measured_position = self.sensor.measure(drone_position)
        target_joint_angles: tuple[float, float] | None = None

        if measured_position is None:
            state = SimulationState.NO_TARGET
        else:
            self.tracker.update(measured_position, dt)
            prediction_position = self.tracker.prediction(prediction_time)

            if prediction_position is None:
                predicted_x, predicted_y = measured_position

            else:
                predicted_x, predicted_y = prediction_position

            reachable = is_reachable(
                predicted_x,
                predicted_y,
                self.robot.link_1,
                self.robot.link_2,
            )

            if not reachable:
                state = SimulationState.TARGET_UNREACHABLE
            else:
                target_joint_angles = inverse_kinematics(
                    predicted_x,
                    predicted_y,
                    self.robot.link_1,
                    self.robot.link_2,
                )
                q1_target, q2_target = target_joint_angles
                q1_velocity = self.pid_q1.update(
                    target=q1_target,
                    current=self.robot.q1,
                    dt=dt,
                )
                q2_velocity = self.pid_q2.update(
                    target=q2_target,
                    current=self.robot.q2,
                    dt=dt,
                )
                self.robot.update(q1_velocity=q1_velocity, q2_velocity=q2_velocity, dt=dt)
                state = SimulationState.TRACKING

        robot_joint_angles = (self.robot.q1, self.robot.q2)
        end_effector_position = forward_kinematics(
            self.robot.q1,
            self.robot.q2,
            self.robot.link_1,
            self.robot.link_2,
        )

        return SimulationStep(
            time=self.time,
            state=state,
            drone_position=drone_position,
            measured_position=measured_position,
            target_joint_angles=target_joint_angles,
            robot_joint_angles=robot_joint_angles,
            end_effector_position=end_effector_position,
        )
