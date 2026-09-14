"""Matplotlib visualization for simulation steps."""

from __future__ import annotations

from math import cos, sin

import matplotlib.pyplot as plt
from matplotlib.patches import Circle

from anti_drone.robot import Robot
from anti_drone.sensor import Sensor
from anti_drone.simulation import SimulationStep


class Visualization:
    """Passive 2D visualization for Anti-Drone simulation snapshots."""

    def __init__(self, robot: Robot, sensor: Sensor) -> None:
        self.robot = robot
        self.sensor = sensor
        self.drone_history: list[tuple[float, float]] = []

        self.figure, self.ax = plt.subplots()
        self.ax.set_title("Anti-Drone System Simulation")
        self.ax.set_xlabel("x")
        self.ax.set_ylabel("y")
        self.ax.set_aspect("equal", adjustable="box")
        self.ax.grid(True)

        (self.robot_line,) = self.ax.plot([], [], "o-", color="tab:blue", label="Robot")
        (self.base_marker,) = self.ax.plot([0.0], [0.0], "ks", label="Base")
        (self.joint_marker,) = self.ax.plot([], [], "o", color="tab:blue", label="Joint")
        (self.end_effector_marker,) = self.ax.plot(
            [],
            [],
            "o",
            color="tab:cyan",
            label="End effector",
        )
        (self.drone_marker,) = self.ax.plot([], [], "ro", label="Drone")
        (self.measurement_marker,) = self.ax.plot(
            [],
            [],
            "x",
            color="tab:orange",
            label="Measurement",
            visible=False,
        )
        (self.sensor_marker,) = self.ax.plot(
            [sensor.position[0]],
            [sensor.position[1]],
            "D",
            color="tab:green",
            label="Sensor",
        )
        self.sensor_range_circle = Circle(
            sensor.position,
            sensor.detection_range,
            fill=False,
            linestyle="--",
            color="tab:green",
            alpha=0.5,
        )
        self.ax.add_patch(self.sensor_range_circle)
        (self.history_line,) = self.ax.plot(
            [], [], ".", color="tab:red", alpha=0.35, label="History"
        )

        self.time_text = self.ax.text(
            0.02, 0.98, "Time: 0.00 s", transform=self.ax.transAxes, va="top"
        )
        self.state_text = self.ax.text(
            0.02, 0.92, "State: -", transform=self.ax.transAxes, va="top"
        )

        self._set_axis_limits()
        self.ax.legend(loc="upper right")

    def update(self, step: SimulationStep) -> None:
        """Update plotted data from a completed simulation step."""

        q1, _q2 = step.robot_joint_angles
        joint_x = self.robot.link_1 * cos(q1)
        joint_y = self.robot.link_1 * sin(q1)
        end_x, end_y = step.end_effector_position

        self.robot_line.set_data([0.0, joint_x, end_x], [0.0, joint_y, end_y])
        self.joint_marker.set_data([joint_x], [joint_y])
        self.end_effector_marker.set_data([end_x], [end_y])

        drone_x, drone_y = step.drone_position
        self.drone_marker.set_data([drone_x], [drone_y])
        self.drone_history.append(step.drone_position)
        history_x = [position[0] for position in self.drone_history]
        history_y = [position[1] for position in self.drone_history]
        self.history_line.set_data(history_x, history_y)

        if step.measured_position is None:
            self.measurement_marker.set_data([], [])
            self.measurement_marker.set_visible(False)
        else:
            measured_x, measured_y = step.measured_position
            self.measurement_marker.set_data([measured_x], [measured_y])
            self.measurement_marker.set_visible(True)

        self.time_text.set_text(f"Time: {step.time:.2f} s")
        self.state_text.set_text(f"State: {step.state.name}")
        self.figure.canvas.draw_idle()

    def show(self) -> None:
        """Display the Matplotlib figure."""

        plt.show()

    def _set_axis_limits(self) -> None:
        reach = self.robot.link_1 + self.robot.link_2
        sensor_x, sensor_y = self.sensor.position
        sensor_extent = self.sensor.detection_range
        max_extent = max(
            reach,
            abs(sensor_x) + sensor_extent,
            abs(sensor_y) + sensor_extent,
        )
        limit = max_extent * 1.2 if max_extent > 0 else 1.0
        self.ax.set_xlim(-limit, limit)
        self.ax.set_ylim(-limit, limit)
