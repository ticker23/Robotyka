#!/usr/bin/env python3

"""Run a simple animated Anti-Drone System demonstration."""

from __future__ import annotations

import matplotlib.pyplot as plt

from anti_drone.controller import PIDController
from anti_drone.drone import Drone
from anti_drone.robot import Robot
from anti_drone.sensor import Sensor, TargetTracker
from anti_drone.simulation import Simulation
from anti_drone.trajectory import CircularTrajectory
from anti_drone.visualization import Visualization


def main() -> None:
    trajectory = CircularTrajectory(
        center=(0.75, 0.5),
        radius=0.75,
        angular_velocity=0.7,
        phase=0.0,
    )
    drone = Drone(trajectory)
    sensor = Sensor(position=(0.0, 0.0), detection_range=2.5, noise_std=0.01)
    tracker = TargetTracker()
    robot = Robot(link_1=1.0, link_2=1.0)
    pid_q1 = PIDController(kp=3.0, ki=0.0, kd=0.15)
    pid_q2 = PIDController(kp=3.0, ki=0.0, kd=0.15)
    simulation = Simulation(
        drone=drone,
        sensor=sensor,
        tracker=tracker,
        robot=robot,
        pid_q1=pid_q1,
        pid_q2=pid_q2,
    )
    visualization = Visualization(robot=robot, sensor=sensor)

    dt = 0.03
    prediction_time = 0.1
    duration = 20.0

    plt.ion()
    while simulation.time < duration and plt.fignum_exists(visualization.figure.number):
        step = simulation.step(dt, prediction_time=prediction_time)
        visualization.update(step)
        plt.pause(dt)

    plt.ioff()
    visualization.show()


if __name__ == "__main__":
    main()
