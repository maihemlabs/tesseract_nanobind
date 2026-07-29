# tesseract_robotics.tesseract_ros2_rosutils

Python bindings for a high-leverage subset of the `tesseract_rosutils`
package from
[tesseract_ros2](https://github.com/tesseract-robotics/tesseract_ros2) — a
collection of conversion helpers between Tesseract types and ROS 2 message
types.

!!! note "Optional module"
    This submodule is built automatically when the repository is configured in a
    shell that has sourced a ROS 2 distro (e.g. `/opt/ros/jazzy/setup.bash`) and
    the `tesseract_ros2` overlay is present — no CMake flag needed. Default
    wheels do not include it; `import
    tesseract_robotics.tesseract_ros2_rosutils` raises `ImportError` on a
    ROS-less install.

!!! warning "Bound msg types are NOT rclpy types"
    The ROS message classes exposed by this module (`Pose`, `EnvironmentCommand`,
    `JointTrajectory`, …) are nanobind-bound C++ classes from
    `geometry_msgs::msg::Pose` etc. They are **not** the
    `geometry_msgs.msg.Pose` classes you get from `rclpy`. To publish to ROS
    or process subscriber messages, copy fields between the bound msg and the
    rclpy msg of the same shape — the same pattern this repo already uses for
    `Eigen::Isometry3d` ↔ numpy at the boundary.

## Scope

This first-pass binding covers the conversion functions whose C++ side does
real algorithmic work — Eigen quaternion math and polymorphic
`tesseract_environment::Command` dispatch — and skips the trivial field-copy
helpers (e.g. `JointState` ↔ `unordered_map<string, double>`) which are
shorter to write directly in Python against rclpy than to bridge.

| Function | Maps |
|----------|------|
| `iso_to_pose(iso)` | `Eigen::Isometry3d` → `geometry_msgs::msg::Pose` |
| `pose_to_iso(pose)` | `geometry_msgs::msg::Pose` → `Eigen::Isometry3d` |
| `isos_to_pose_array(isos)` | `list[Isometry3d]` → `geometry_msgs::msg::PoseArray` |
| `command_to_msg(cmd)` | `tesseract_environment::Command` → `tesseract_msgs::msg::EnvironmentCommand` |
| `msg_to_command(msg)` | `tesseract_msgs::msg::EnvironmentCommand` → `Command` |
| `commands_to_msg(cmds, past_revision=0)` | `list[Command]` → `list[EnvironmentCommand]` |
| `msgs_to_commands(msgs)` | `list[EnvironmentCommand]` → `list[Command]` |
| `trajectory_to_msg(traj)` | `tesseract_common::JointTrajectory` → `tesseract_msgs::msg::JointTrajectory` |
| `msg_to_trajectory(msg)` | reverse |
| `trajectory_to_legacy_msg(traj, initial_state)` | `tesseract_common::JointTrajectory` + `SceneState` → `trajectory_msgs::msg::JointTrajectory` |
| `legacy_msg_to_trajectory(msg)` | reverse |

## Quickstart

```python
from tesseract_robotics import tesseract_ros2_rosutils as ru
from tesseract_robotics.tesseract_common import Isometry3d

# 1. Eigen <-> Pose
iso = Isometry3d.Identity()
pose_msg = ru.iso_to_pose(iso)            # nanobind Pose, NOT rclpy
print(pose_msg.position.x, pose_msg.orientation.w)

iso_back = ru.pose_to_iso(pose_msg)       # roundtrip

# 2. Command <-> EnvironmentCommand
from tesseract_robotics.tesseract_environment import (
    AddLinkCommand,
    RemoveJointCommand,
)
from tesseract_robotics.tesseract_scene_graph import Link

cmd = AddLinkCommand(Link("test_link"), False)
env_cmd_msg = ru.command_to_msg(cmd)      # nanobind EnvironmentCommand
assert env_cmd_msg.command == ru.EnvironmentCommand.ADD_LINK

cmd_back = ru.msg_to_command(env_cmd_msg) # polymorphic dispatch
assert isinstance(cmd_back, AddLinkCommand)
```

## Bridging to rclpy at the boundary

Publishing requires copying the bound msg into an rclpy msg of the same
shape. For a single `Pose`, that's eight field assignments:

```python
import geometry_msgs.msg as gm
from tesseract_robotics import tesseract_ros2_rosutils as ru

def to_rclpy_pose(p):
    out = gm.Pose()
    out.position.x = p.position.x
    out.position.y = p.position.y
    out.position.z = p.position.z
    out.orientation.x = p.orientation.x
    out.orientation.y = p.orientation.y
    out.orientation.z = p.orientation.z
    out.orientation.w = p.orientation.w
    return out

iso = ...  # Eigen::Isometry3d
publisher.publish(to_rclpy_pose(ru.iso_to_pose(iso)))
```

For larger messages (`EnvironmentCommand`, `JointTrajectory`), the
boundary copy is recursive — write a helper once per project and reuse it.
A future submodule (`tesseract_ros2_rosutils_rclpy`?) could provide the
copies as Python helpers if the use case scales.

## What is *not* bound

- `JointState` ↔ `unordered_map<string, double>` (rosutils:405/413) and
  `StringDoublePair[]` ↔ `unordered_map<string, double>` (rosutils:421/430)
  — trivial in pure Python.
- The `EnvironmentCommand` discriminator handles 23 command types, but only
  the fields used by the high-leverage subset are exposed via `def_rw`.
  Commands that depend on currently-unbound msg types (kinematics info,
  contact-manager plugin info, joint-limit pair vectors,
  collision-margin-pair data, floating-joint transforms) will not roundtrip
  through these bindings — left for a follow-up PR.
- `processMsg(Environment, …)` (follower-side application of msg deltas to
  a local Environment) — no current consumer.
- Full `Environment` / `EnvironmentState` serialization — not needed for
  current workflows.
- TF helpers (`toTransformMsgs`) — `robot_state_publisher` covers TF.

## API Reference

This page is intentionally manual — auto-generated reference would require
importing the extension at docs-build time, which only works in a shell
that has both pixi and ROS 2 active. For full signatures, consult the type
stub:

`src/tesseract_robotics/tesseract_ros2_rosutils/_tesseract_ros2_rosutils.pyi`

(generated by `./scripts/generate_stubs.sh` on a machine that built the
optional bindings).
