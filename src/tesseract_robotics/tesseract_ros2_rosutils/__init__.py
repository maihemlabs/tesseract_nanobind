"""tesseract_ros2_rosutils Python bindings (nanobind).

Exposes a high-leverage subset of conversion functions and message types
from the `tesseract_rosutils` package in the tesseract_ros2 repo (ROS 2).

Module is prefixed `tesseract_ros2_` so a future ROS 1 variant
(`tesseract_ros_rosutils`) can coexist without a naming collision.

Only available when the repo was built with
`-DTESSERACT_NANOBIND_BUILD_ROS=ON`; otherwise the underlying `.so`
is absent and importing this package raises ImportError.

Bound msg types are nanobind classes, NOT rclpy classes. To publish to ROS
or consume rclpy subscriber messages, copy fields between the bound msg
and the rclpy msg of the same shape — the same pattern used elsewhere in
this repo for `Eigen::Isometry3d <-> numpy`.
"""

import tesseract_robotics.tesseract_common  # noqa: F401 - JointTrajectory lives here
import tesseract_robotics.tesseract_environment  # noqa: F401 - Command hierarchy lives here
import tesseract_robotics.tesseract_state_solver  # noqa: F401 - SceneState lives here (used by trajectory_to_legacy_msg)
from tesseract_robotics.tesseract_ros2_rosutils._tesseract_ros2_rosutils import *  # noqa: F401,F403

__all__ = [
    # geometry_msgs
    "Vector3",
    "Point",
    "Quaternion",
    "Pose",
    "PoseArray",
    # std_msgs / builtin_interfaces
    "Header",
    "ColorRGBA",
    "Time",
    "Duration",
    # sensor_msgs (rebound to disambiguate from tesseract_msgs::JointState)
    "SensorJointState",
    # trajectory_msgs (rebound to disambiguate from tesseract_msgs::JointTrajectory)
    "JointTrajectoryPoint",
    "TrajectoryMsgsJointTrajectory",
    # tesseract_msgs leaves
    "JointCalibration",
    "JointDynamics",
    "JointLimits",
    "JointMimic",
    "JointSafety",
    "Material",
    "Mesh",
    "Geometry",
    "Inertial",
    "AllowedCollisionEntry",
    "CollisionMarginData",
    "StringDoublePair",
    "TesseractMsgsJointState",
    "TesseractMsgsJointTrajectory",
    # tesseract_msgs composites
    "VisualGeometry",
    "CollisionGeometry",
    "Link",
    "Joint",
    "SceneGraph",
    # top-level discriminated union
    "EnvironmentCommand",
    # conversion functions
    "iso_to_pose",
    "pose_to_iso",
    "isos_to_pose_array",
    "command_to_msg",
    "commands_to_msg",
    "msg_to_command",
    "msgs_to_commands",
    "trajectory_to_msg",
    "msg_to_trajectory",
    "trajectory_to_legacy_msg",
    "legacy_msg_to_trajectory",
]
