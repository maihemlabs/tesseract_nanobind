"""Roundtrip tests for JointTrajectory <-> ROS msg conversions.

Two flavors:
  - tesseract_msgs::JointTrajectory (richer, carries description / uuid)
  - trajectory_msgs::JointTrajectory (the standard ROS one, used to publish
    to controllers — needs an initial SceneState for the first point).
"""

import numpy as np
import pytest

pytest.importorskip("tesseract_robotics.tesseract_ros2_rosutils")

from tesseract_robotics import tesseract_ros2_rosutils as ru  # noqa: E402
from tesseract_robotics.tesseract_common import (  # noqa: E402
    GeneralResourceLocator,
    JointState,
    JointTrajectory,
)
from tesseract_robotics.tesseract_environment import Environment  # noqa: E402

SIMPLE_URDF = """
<robot name="traj_test_robot" xmlns:tesseract="http://ros.org/wiki/tesseract" tesseract:make_convex="true">
  <link name="world"/>
  <link name="link1"/>
  <link name="link2"/>
  <joint name="j1" type="revolute">
    <parent link="world"/>
    <child link="link1"/>
    <axis xyz="0 0 1"/>
    <limit effort="100" lower="-3.14" upper="3.14" velocity="1.0"/>
  </joint>
  <joint name="j2" type="revolute">
    <parent link="link1"/>
    <child link="link2"/>
    <axis xyz="0 1 0"/>
    <limit effort="100" lower="-3.14" upper="3.14" velocity="1.0"/>
  </joint>
</robot>
"""


def _make_state(names, positions, time):
    s = JointState(names, np.array(positions, dtype=np.float64))
    s.time = float(time)
    return s


def _make_trajectory():
    states = [
        _make_state(["j1", "j2"], [0.0, 0.0], 0.0),
        _make_state(["j1", "j2"], [0.5, -0.5], 0.5),
        _make_state(["j1", "j2"], [1.0, -1.0], 1.0),
    ]
    return JointTrajectory(states, "test_trajectory")


def test_tesseract_msgs_trajectory_roundtrip():
    traj = _make_trajectory()
    msg = ru.trajectory_to_msg(traj)
    assert msg.description == "test_trajectory"
    assert len(msg.states) == 3

    traj2 = ru.msg_to_trajectory(msg)
    assert traj2.description == "test_trajectory"
    assert len(traj2.states) == 3
    for orig, got in zip(traj.states, traj2.states):
        assert orig.joint_names == got.joint_names
        assert np.allclose(orig.position, got.position)
        assert orig.time == pytest.approx(got.time)


def test_trajectory_msgs_trajectory_roundtrip():
    env = Environment()
    assert env.init(SIMPLE_URDF, GeneralResourceLocator())
    initial_state = env.getState()

    traj = _make_trajectory()
    msg = ru.trajectory_to_legacy_msg(traj, initial_state)
    assert msg.joint_names == ["j1", "j2"]
    assert len(msg.points) == 3
    assert msg.points[0].positions[0] == pytest.approx(0.0)
    assert msg.points[2].positions[0] == pytest.approx(1.0)
    assert msg.points[2].positions[1] == pytest.approx(-1.0)

    traj2 = ru.legacy_msg_to_trajectory(msg)
    assert len(traj2.states) == 3
    assert traj2.states[2].joint_names == ["j1", "j2"]
    assert np.allclose(traj2.states[2].position, [1.0, -1.0])


def test_empty_trajectory_roundtrip():
    traj = JointTrajectory("empty")
    msg = ru.trajectory_to_msg(traj)
    assert msg.description == "empty"
    assert len(msg.states) == 0
    traj2 = ru.msg_to_trajectory(msg)
    assert traj2.description == "empty"
    assert len(traj2.states) == 0
