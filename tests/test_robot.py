import pytest

from anti_drone.robot import Robot


def test_valid_link_lengths_are_stored() -> None:
    robot = Robot(link_1=2.0, link_2=1.5)

    assert robot.link_1 == pytest.approx(2.0)
    assert robot.link_2 == pytest.approx(1.5)


def test_default_joint_angles_are_zero() -> None:
    robot = Robot(link_1=2.0, link_2=1.5)

    assert robot.q1 == pytest.approx(0.0)
    assert robot.q2 == pytest.approx(0.0)


def test_custom_initial_joint_angles_are_stored() -> None:
    robot = Robot(link_1=2.0, link_2=1.5, q1=0.5, q2=-0.25)

    assert robot.q1 == pytest.approx(0.5)
    assert robot.q2 == pytest.approx(-0.25)


def test_negative_initial_joint_angles_are_valid() -> None:
    robot = Robot(link_1=2.0, link_2=1.5, q1=-1.0, q2=-2.0)

    assert robot.q1 == pytest.approx(-1.0)
    assert robot.q2 == pytest.approx(-2.0)


@pytest.mark.parametrize(
    ("link_1", "link_2", "message"),
    [
        (0.0, 1.0, "link_1 must be greater than zero"),
        (-1.0, 1.0, "link_1 must be greater than zero"),
        (1.0, 0.0, "link_2 must be greater than zero"),
        (1.0, -1.0, "link_2 must be greater than zero"),
    ],
)
def test_invalid_link_lengths_raise_value_error(link_1: float, link_2: float, message: str) -> None:
    with pytest.raises(ValueError, match=message):
        Robot(link_1=link_1, link_2=link_2)


def test_positive_q1_velocity_increases_q1() -> None:
    robot = Robot(link_1=1.0, link_2=1.0)

    robot.update(q1_velocity=2.0, q2_velocity=0.0, dt=0.5)

    assert robot.q1 == pytest.approx(1.0)
    assert robot.q2 == pytest.approx(0.0)


def test_negative_q1_velocity_decreases_q1() -> None:
    robot = Robot(link_1=1.0, link_2=1.0, q1=1.0)

    robot.update(q1_velocity=-2.0, q2_velocity=0.0, dt=0.25)

    assert robot.q1 == pytest.approx(0.5)
    assert robot.q2 == pytest.approx(0.0)


def test_positive_q2_velocity_increases_q2() -> None:
    robot = Robot(link_1=1.0, link_2=1.0)

    robot.update(q1_velocity=0.0, q2_velocity=3.0, dt=0.5)

    assert robot.q1 == pytest.approx(0.0)
    assert robot.q2 == pytest.approx(1.5)


def test_negative_q2_velocity_decreases_q2() -> None:
    robot = Robot(link_1=1.0, link_2=1.0, q2=1.0)

    robot.update(q1_velocity=0.0, q2_velocity=-2.0, dt=0.25)

    assert robot.q1 == pytest.approx(0.0)
    assert robot.q2 == pytest.approx(0.5)


def test_both_joints_update_in_same_call() -> None:
    robot = Robot(link_1=1.0, link_2=1.0)

    robot.update(q1_velocity=2.0, q2_velocity=-1.0, dt=0.5)

    assert robot.q1 == pytest.approx(1.0)
    assert robot.q2 == pytest.approx(-0.5)


def test_zero_q1_velocity_leaves_q1_unchanged() -> None:
    robot = Robot(link_1=1.0, link_2=1.0, q1=0.75, q2=0.25)

    robot.update(q1_velocity=0.0, q2_velocity=1.0, dt=0.5)

    assert robot.q1 == pytest.approx(0.75)
    assert robot.q2 == pytest.approx(0.75)


def test_zero_q2_velocity_leaves_q2_unchanged() -> None:
    robot = Robot(link_1=1.0, link_2=1.0, q1=0.25, q2=0.75)

    robot.update(q1_velocity=1.0, q2_velocity=0.0, dt=0.5)

    assert robot.q1 == pytest.approx(0.75)
    assert robot.q2 == pytest.approx(0.75)


def test_zero_velocities_leave_robot_state_unchanged() -> None:
    robot = Robot(link_1=1.0, link_2=1.0, q1=0.25, q2=-0.75)

    robot.update(q1_velocity=0.0, q2_velocity=0.0, dt=1.0)

    assert robot.q1 == pytest.approx(0.25)
    assert robot.q2 == pytest.approx(-0.75)


def test_different_dt_values_produce_correct_state_changes() -> None:
    robot = Robot(link_1=1.0, link_2=1.0)

    robot.update(q1_velocity=2.0, q2_velocity=4.0, dt=0.25)
    robot.update(q1_velocity=2.0, q2_velocity=4.0, dt=0.75)

    assert robot.q1 == pytest.approx(2.0)
    assert robot.q2 == pytest.approx(4.0)


@pytest.mark.parametrize("dt", [0.0, -0.1])
def test_invalid_dt_raises_value_error(dt: float) -> None:
    robot = Robot(link_1=1.0, link_2=1.0)

    with pytest.raises(ValueError, match="dt must be greater than zero"):
        robot.update(q1_velocity=1.0, q2_velocity=1.0, dt=dt)


def test_repeated_updates_accumulate_state() -> None:
    robot = Robot(link_1=1.0, link_2=1.0)

    robot.update(q1_velocity=1.0, q2_velocity=0.0, dt=0.5)
    assert robot.q1 == pytest.approx(0.5)

    robot.update(q1_velocity=1.0, q2_velocity=0.0, dt=0.5)
    assert robot.q1 == pytest.approx(1.0)


def test_updating_q1_only_does_not_modify_q2() -> None:
    robot = Robot(link_1=1.0, link_2=1.0, q1=0.0, q2=2.0)

    robot.update(q1_velocity=3.0, q2_velocity=0.0, dt=0.5)

    assert robot.q1 == pytest.approx(1.5)
    assert robot.q2 == pytest.approx(2.0)


def test_updating_q2_only_does_not_modify_q1() -> None:
    robot = Robot(link_1=1.0, link_2=1.0, q1=2.0, q2=0.0)

    robot.update(q1_velocity=0.0, q2_velocity=-3.0, dt=0.5)

    assert robot.q1 == pytest.approx(2.0)
    assert robot.q2 == pytest.approx(-1.5)


def test_update_preserves_link_lengths() -> None:
    robot = Robot(link_1=2.0, link_2=1.5)

    robot.update(q1_velocity=1.0, q2_velocity=-1.0, dt=0.5)

    assert robot.link_1 == pytest.approx(2.0)
    assert robot.link_2 == pytest.approx(1.5)


def test_robot_instances_maintain_independent_state() -> None:
    first_robot = Robot(link_1=1.0, link_2=1.0)
    second_robot = Robot(link_1=1.0, link_2=1.0)

    first_robot.update(q1_velocity=2.0, q2_velocity=-1.0, dt=0.5)

    assert first_robot.q1 == pytest.approx(1.0)
    assert first_robot.q2 == pytest.approx(-0.5)
    assert second_robot.q1 == pytest.approx(0.0)
    assert second_robot.q2 == pytest.approx(0.0)
