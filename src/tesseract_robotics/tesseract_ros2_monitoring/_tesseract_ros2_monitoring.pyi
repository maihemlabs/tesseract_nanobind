"""
Python bindings for tesseract_monitoring (tesseract_ros2, ROS 2 Jazzy+)
"""

from collections.abc import Sequence
import datetime
import enum
from typing import overload

import tesseract_robotics.tesseract_environment._tesseract_environment
import tesseract_robotics.tesseract_state_solver._tesseract_state_solver


class ROSContext:
    """
    Owns rclcpp::init/shutdown lifetime and one user-facing rclcpp::Node. A background SingleThreadedExecutor spins the node so parameter services and other rclcpp callbacks fire. Mirrors the typical environment_monitor_node.cpp construction pattern from tesseract_ros2.
    """

    def __init__(self, node_name: str, args: Sequence[str] = [], install_signal_handlers: bool = False) -> None:
        """
        Initialise rclcpp (if not already up), create a node named `node_name`, and start a background executor thread.
        """

    def node_name(self) -> str:
        """Return the fully-qualified name of the owned rclcpp::Node."""

    def shutdown(self) -> None:
        """
        Stop the executor, join the spin thread, drop the node, and (if this context owned rclcpp::init) call rclcpp::shutdown. Idempotent.
        """

class MonitoredEnvironmentMode(enum.Enum):
    DEFAULT = 0

    SYNCHRONIZED = 1

DEFAULT: MonitoredEnvironmentMode = MonitoredEnvironmentMode.DEFAULT

SYNCHRONIZED: MonitoredEnvironmentMode = MonitoredEnvironmentMode.SYNCHRONIZED

class EnvironmentMonitor:
    """
    Abstract base class for tesseract environment monitors. Concrete subclasses (e.g. ROSEnvironmentMonitor) provide the transport layer.
    """

    def getNamespace(self) -> str:
        """Unique namespace identifying this monitor instance."""

    def getEnvironment(self) -> tesseract_robotics.tesseract_environment._tesseract_environment.Environment:
        """Return the monitored tesseract::environment::Environment."""

    def waitForConnection(self, duration: datetime.timedelta | float = ...) -> bool:
        """
        Block until the environment is connected/initialised. duration=0 waits indefinitely.
        """

    def stopPublishingEnvironment(self) -> None: ...

    def setEnvironmentPublishingFrequency(self, hz: float) -> None: ...

    def getEnvironmentPublishingFrequency(self) -> float: ...

    def startStateMonitor(self, joint_states_topic: str, publish_tf: bool = True) -> None: ...

    def stopStateMonitor(self) -> None: ...

    def setStateUpdateFrequency(self, hz: float = 10.0) -> None: ...

    def getStateUpdateFrequency(self) -> float: ...

    def updateEnvironmentWithCurrentState(self) -> None: ...

    def startMonitoringEnvironment(self, monitored_namespace: str, mode: MonitoredEnvironmentMode = MonitoredEnvironmentMode.DEFAULT) -> None: ...

    def stopMonitoringEnvironment(self) -> None: ...

    def waitForCurrentState(self, duration: datetime.timedelta | float = ...) -> bool: ...

    def shutdown(self) -> None: ...

class ROSEnvironmentMonitor(EnvironmentMonitor):
    """
    Monitors and optionally publishes a tesseract::environment::Environment over ROS 2 topics/services. The URDF-parameter constructor loads the environment from the `robot_description` / `robot_description_semantic` parameters on the context's node; the environment-passing constructor adopts a pre-built Environment.
    """

    @overload
    def __init__(self, context: ROSContext, robot_description: str, monitor_namespace: str) -> None:
        """
        Construct from a ROS-parameter URDF/SRDF. `robot_description` is the *parameter name* (not the URDF text); the parameter must be set on `context`'s node before this runs.
        """

    @overload
    def __init__(self, context: ROSContext, environment: tesseract_robotics.tesseract_environment._tesseract_environment.Environment, monitor_namespace: str) -> None:
        """
        Adopt a pre-built Environment (recommended for Python scripting: build the Environment via tesseract_urdf/tesseract_srdf, then hand it to the monitor).
        """

    def getURDFDescription(self) -> str:
        """
        Return the ROS parameter name that holds the URDF (empty when constructed from a pre-built Environment).
        """

    def startPublishingEnvironment(self, publish_tf: bool) -> None:
        """
        Begin publishing the environment on /<namespace>/tesseract_published_environment. When publish_tf=True, also broadcast TFs for each joint.
        """

    def getStateMonitor(self) -> CurrentStateMonitor:
        """Read-only view of the internal joint-state monitor."""

class CurrentStateMonitor:
    """Monitors a joint_states topic and maintains the current robot state."""

    def isActive(self) -> bool: ...

    def getMonitoredTopic(self) -> str: ...

    def haveCompleteState(self) -> bool:
        """
        True iff every DOF in the kinematic model has been observed at least once.
        """

    def getCurrentState(self) -> tesseract_robotics.tesseract_state_solver._tesseract_state_solver.SceneState: ...

    def getCurrentStateValues(self) -> dict[str, float]:
        """
        Map of joint name -> joint position. Only observed joints appear; check haveCompleteState() first if completeness matters.
        """

    def getBoundsError(self) -> float: ...

    def setBoundsError(self, error: float) -> None: ...

    def enableCopyDynamics(self, enabled: bool) -> None: ...

    def waitForCompleteState(self, wait_time: float) -> bool:
        """Block up to wait_time seconds until every DOF has been observed."""

DEFAULT_JOINT_STATES_TOPIC: str = '/joint_states'

DEFAULT_GET_ENVIRONMENT_CHANGES_SERVICE: str = '/get_tesseract_changes'

DEFAULT_GET_ENVIRONMENT_INFORMATION_SERVICE: str = '/get_tesseract_information'

DEFAULT_MODIFY_ENVIRONMENT_SERVICE: str = '/modify_tesseract'

DEFAULT_SAVE_SCENE_GRAPH_SERVICE: str = '/save_scene_graph'

DEFAULT_PUBLISH_ENVIRONMENT_TOPIC: str = '/tesseract_published_environment'
