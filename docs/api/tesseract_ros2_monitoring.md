# tesseract_robotics.tesseract_ros2_monitoring

Python bindings for the `tesseract_monitoring` package from
[tesseract_ros2](https://github.com/tesseract-robotics/tesseract_ros2) —
the ROS 2 transport layer for a `tesseract::environment::Environment`.

!!! note "Optional module"
    This submodule is built automatically when the repository is configured in a
    shell that has sourced a ROS 2 distro (e.g. `/opt/ros/jazzy/setup.bash`) and
    the `tesseract_ros2` overlay is present — no CMake flag needed. Default
    wheels do not include it; `import
    tesseract_robotics.tesseract_ros2_monitoring` raises `ImportError` on a
    ROS-less install.

!!! info "Naming convention"
    The module is prefixed `tesseract_ros2_` (not simply `tesseract_monitoring`)
    because `tesseract_ros` (ROS 1) contains an identically-named sibling
    package. The prefix keeps both variants addable side-by-side.

## Overview

The core class is `ROSEnvironmentMonitor`, which mirrors the construction
pattern used by `environment_monitor_node.cpp` in the upstream repo:

1. Create a `ROSContext` — a RAII wrapper owning `rclcpp::init`/`shutdown` and
   a user-facing `rclcpp::Node` spun on a background executor.
2. Construct a `ROSEnvironmentMonitor` against that context, either with a
   `robot_description` ROS-parameter name (the monitor loads URDF/SRDF from
   those parameters) or with a pre-built `Environment` (recommended for
   Python scripting).
3. Drive the lifecycle with `startStateMonitor` / `startPublishingEnvironment` /
   `startMonitoringEnvironment`, then `shutdown()` to clean up.

## Quickstart

```python
from tesseract_robotics import (
    tesseract_environment,
    tesseract_ros2_monitoring as tm_ros2,
    tesseract_srdf,
    tesseract_urdf,
)
from tesseract_robotics.tesseract_common import GeneralResourceLocator

# 1. Build an Environment in Python.
locator = GeneralResourceLocator()
sg = tesseract_urdf.parseURDFFile("path/to/robot.urdf", locator)
srdf = tesseract_srdf.SRDFModel()
srdf.initFile(sg, "path/to/robot.srdf", locator)
env = tesseract_environment.Environment()
assert env.init(sg, srdf)

# 2. Stand up a ROS context and a monitor against it.
ctx = tm_ros2.ROSContext("my_tesseract_node", install_signal_handlers=False)
mon = tm_ros2.ROSEnvironmentMonitor(ctx, env, monitor_namespace="my_robot")

# 3. Publish the environment, subscribe to /joint_states, block until shutdown.
mon.startPublishingEnvironment(publish_tf=True)
mon.startStateMonitor("/joint_states", publish_tf=False)

try:
    # Poll the current state from Python while the monitor runs in background.
    while True:
        values = mon.getStateMonitor().getCurrentStateValues()
        print(values)
except KeyboardInterrupt:
    pass
finally:
    mon.shutdown()
    ctx.shutdown()
```

## `ROSContext`

`ROSContext(node_name, args=[], install_signal_handlers=False)` owns
`rclcpp::init/shutdown` lifetime plus one `rclcpp::Node`. Construction:

| Parameter | Default | Notes |
|-----------|---------|-------|
| `node_name` | — | Name of the user-facing `rclcpp::Node`. |
| `args` | `[]` | Extra argv strings passed to `rclcpp::init` (e.g. `--remap`). |
| `install_signal_handlers` | `False` | When `False`, passes `SignalHandlerOptions::None` so rclcpp does **not** install SIGINT/SIGTERM handlers. Leave `False` under pytest / pytest-xdist. |

The context itself also spins a single-threaded executor on a background
thread so parameter services on its node can respond. The monitor spawns a
*separate* internal node + multi-threaded executor for its own business
logic — the two executors do not share state.

## Constructor overloads

```python
# Overload 1: URDF via ROS parameter
mon = tm_ros2.ROSEnvironmentMonitor(ctx, "robot_description", "my_robot")
# Requires ctx.node() to already have `robot_description` and
# `robot_description_semantic` parameters set (e.g. declared by a launch file).

# Overload 2: adopt a pre-built Environment
mon = tm_ros2.ROSEnvironmentMonitor(ctx, env, "my_robot")
```

Overload 2 is the path most Python users want — build the Environment with
the existing `tesseract_urdf` / `tesseract_srdf` bindings and then hand it
to the monitor.

## Lifetime

The binding holds a `keep_alive` relationship between the monitor and its
context: `ctx` cannot be garbage-collected before the monitor. Calling
`ctx.shutdown()` while a monitor is still alive will cause its internal
executor to observe a shut-down rclcpp and error. Always `mon.shutdown()`
first, then `ctx.shutdown()`.

## What is *not* bound

The first-pass binding deliberately omits:

- `CurrentStateMonitor.addUpdateCallback(...)` — the callback signature uses
  `sensor_msgs::msg::JointState::ConstSharedPtr`, which would require a ROS
  message bridge. Poll `getCurrentStateValues()` from Python instead.
- Methods returning `rclcpp::Time` / taking `rclcpp::Duration`
  (`getCurrentStateTime`, `haveCompleteState(age)` overloads, …). Use the
  `double`-seconds or no-arg variants.
- `ROSEnvironmentMonitorInterface` (multi-namespace apply-command utility)
  and `ContactMonitor`. Tracked as follow-up work.

## API Reference

This page is intentionally manual — auto-generated reference would require
importing the extension at docs-build time, which only works in a shell that
has both pixi and ROS 2 active (and never on CI's ROS-less runners). For the
full signatures, consult the type stub:

`src/tesseract_robotics/tesseract_ros2_monitoring/_tesseract_ros2_monitoring.pyi`

(generated by `./scripts/generate_stubs.sh` on a machine that built the
optional bindings).
