"""Roundtrip tests for Pose / PoseArray conversions.

Skipped when the ROS extension is absent (default ROS-less builds).
"""

import math

import numpy as np
import pytest

pytest.importorskip("tesseract_robotics.tesseract_ros2_rosutils")

from tesseract_robotics import tesseract_ros2_rosutils as ru  # noqa: E402
from tesseract_robotics.tesseract_common import Isometry3d  # noqa: E402


def _make_iso(tx, ty, tz, rx_deg, ry_deg, rz_deg):
    """Build an Isometry3d from XYZ + extrinsic-rotation Euler angles."""
    m = np.eye(4)
    cz, sz = math.cos(math.radians(rz_deg)), math.sin(math.radians(rz_deg))
    cy, sy = math.cos(math.radians(ry_deg)), math.sin(math.radians(ry_deg))
    cx, sx = math.cos(math.radians(rx_deg)), math.sin(math.radians(rx_deg))
    Rz = np.array([[cz, -sz, 0], [sz, cz, 0], [0, 0, 1]])
    Ry = np.array([[cy, 0, sy], [0, 1, 0], [-sy, 0, cy]])
    Rx = np.array([[1, 0, 0], [0, cx, -sx], [0, sx, cx]])
    m[:3, :3] = Rz @ Ry @ Rx
    m[:3, 3] = [tx, ty, tz]
    return Isometry3d(m)


def test_iso_to_pose_translation():
    iso = _make_iso(1.0, 2.0, 3.0, 0, 0, 0)
    msg = ru.iso_to_pose(iso)
    assert msg.position.x == pytest.approx(1.0)
    assert msg.position.y == pytest.approx(2.0)
    assert msg.position.z == pytest.approx(3.0)
    # Identity rotation: q = (0, 0, 0, 1)
    assert msg.orientation.x == pytest.approx(0.0)
    assert msg.orientation.y == pytest.approx(0.0)
    assert msg.orientation.z == pytest.approx(0.0)
    assert msg.orientation.w == pytest.approx(1.0)


def test_iso_to_pose_rotation():
    # 90deg about Z: q = (0, 0, sin(45), cos(45))
    iso = _make_iso(0, 0, 0, 0, 0, 90)
    msg = ru.iso_to_pose(iso)
    assert msg.orientation.x == pytest.approx(0.0, abs=1e-9)
    assert msg.orientation.y == pytest.approx(0.0, abs=1e-9)
    assert msg.orientation.z == pytest.approx(math.sin(math.pi / 4))
    assert msg.orientation.w == pytest.approx(math.cos(math.pi / 4))


def test_pose_to_iso_roundtrip():
    iso = _make_iso(0.5, -1.5, 2.5, 30, 45, -60)
    msg = ru.iso_to_pose(iso)
    iso2 = ru.pose_to_iso(msg)
    assert np.allclose(iso.matrix, iso2.matrix, atol=1e-9)


def test_isos_to_pose_array():
    isos = [
        _make_iso(0, 0, 0, 0, 0, 0),
        _make_iso(1, 2, 3, 0, 90, 0),
        _make_iso(-1, -2, -3, 0, 0, 180),
    ]
    msg = ru.isos_to_pose_array(isos)
    assert len(msg.poses) == 3
    assert msg.poses[1].position.x == pytest.approx(1.0)
    assert msg.poses[1].position.y == pytest.approx(2.0)
    assert msg.poses[1].position.z == pytest.approx(3.0)
    # Verify roundtrip via per-pose conversion.
    assert np.allclose(ru.pose_to_iso(msg.poses[2]).matrix, isos[2].matrix, atol=1e-9)


def test_empty_pose_array():
    msg = ru.isos_to_pose_array([])
    assert len(msg.poses) == 0
