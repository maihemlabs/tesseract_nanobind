"""Lifecycle smoke tests for tesseract_ros2_monitoring bindings.

The whole module is skipped when the C++ extension is absent — this
matches the default (ROS-less) build where `TESSERACT_NANOBIND_BUILD_ROS=OFF`.
"""

import os

import pytest

pytest.importorskip("tesseract_robotics.tesseract_ros2_monitoring")

from tesseract_robotics import (  # noqa: E402
    tesseract_environment,
    tesseract_ros2_monitoring as tm_ros2,
    tesseract_srdf,
    tesseract_urdf,
)

from ..tesseract_support_resource_locator import TesseractSupportResourceLocator  # noqa: E402


def _make_env():
    """Build an Environment from the bundled iiwa URDF/SRDF."""
    support = os.environ["TESSERACT_SUPPORT_DIR"]
    locator = TesseractSupportResourceLocator()
    sg = tesseract_urdf.parseURDFFile(
        os.path.join(support, "urdf/lbr_iiwa_14_r820.urdf"), locator
    )
    srdf = tesseract_srdf.SRDFModel()
    srdf.initFile(sg, os.path.join(support, "urdf/lbr_iiwa_14_r820.srdf"), locator)
    env = tesseract_environment.Environment()
    assert env.init(sg, srdf)
    return env


@pytest.fixture(scope="module")
def ctx():
    """Module-scoped ROSContext. install_signal_handlers=False avoids rclcpp
    stomping on pytest's own SIGINT handling (matters under pytest-xdist)."""
    c = tm_ros2.ROSContext(
        "tesseract_nanobind_test_node",
        args=[],
        install_signal_handlers=False,
    )
    yield c
    c.shutdown()


def test_ros_context_node_name(ctx):
    assert ctx.node_name() == "tesseract_nanobind_test_node"


def test_construct_with_prebuilt_env(ctx):
    env = _make_env()
    mon = tm_ros2.ROSEnvironmentMonitor(ctx, env, "nb_test_ns")
    assert mon.getNamespace() == "nb_test_ns"
    assert mon.getEnvironment() is not None
    mon.shutdown()


def test_publish_lifecycle(ctx):
    mon = tm_ros2.ROSEnvironmentMonitor(ctx, _make_env(), "nb_test_pub")
    mon.startPublishingEnvironment(False)
    mon.setEnvironmentPublishingFrequency(10.0)
    assert mon.getEnvironmentPublishingFrequency() == pytest.approx(10.0)
    mon.stopPublishingEnvironment()
    mon.shutdown()


def test_state_monitor_lifecycle(ctx):
    mon = tm_ros2.ROSEnvironmentMonitor(ctx, _make_env(), "nb_test_state")
    mon.startStateMonitor("/nb_test_joint_states", False)
    sm = mon.getStateMonitor()
    assert sm.isActive()
    assert sm.getMonitoredTopic() == "/nb_test_joint_states"
    mon.stopStateMonitor()
    assert not sm.isActive()
    mon.shutdown()


def test_wait_for_connection_short_timeout(ctx):
    mon = tm_ros2.ROSEnvironmentMonitor(ctx, _make_env(), "nb_test_wait")
    # Env is already initialised, so waitForConnection returns True promptly.
    assert mon.waitForConnection(0.5) is True
    mon.shutdown()
