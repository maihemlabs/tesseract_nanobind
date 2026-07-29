# ROS 2 Bindings Build

`tesseract_ros2_monitoring` and `tesseract_ros2_rosutils` build when a ROS 2 distro is
sourced and are skipped when it isn't. There is no flag to pass and no dedicated pixi
environment:

```bash
source /opt/ros/jazzy/setup.bash
pixi run install
pixi run test-ros
```

**Prerequisite:** `sudo apt install python3-colcon-common-extensions`. See
[The overlay builds with the ROS side's toolchain](#the-overlay-builds-with-the-ros-sides-toolchain)
for why it comes from apt rather than pixi.

## Two halves have to line up, and conda ships only one

| Half | Provides | Comes from |
| --- | --- | --- |
| the distro | `rclcpp`, `geometry_msgs`, `sensor_msgs`, `trajectory_msgs`, `std_msgs`, `builtin_interfaces` | `source /opt/ros/<distro>/setup.bash`, which puts them on `CMAKE_PREFIX_PATH` |
| the overlay | `tesseract_msgs`, `tesseract_rosutils`, `tesseract_monitoring` | source-built into `ws/install/` by `scripts/build_tesseract_ros2.sh` |

Conda ships **no** ROS 2 packages, so the overlay is always built from source. That is why
"is ROS available?" cannot be answered by looking at the conda env — it means "is a distro
sourced, and has the overlay been built?".

`scripts/install.sh` (the `install` task) builds the overlay on first use, so the one-time
setup is not a separate command. `pixi run build-ros` rebuilds it on its own, e.g. after
bumping `dependencies_ros.rosinstall`.

## Detection follows the shell, not the build directory

`CMakeLists.txt` gates the whole ROS block on `$ENV{AMENT_PREFIX_PATH}` being non-empty,
and only then puts `ws/install` on `CMAKE_PREFIX_PATH` and calls `find_package`.

!!! warning "Both halves of that ordering are load-bearing — neither is stylistic"

    **Gating on `find_package` results instead of the environment breaks the ROS-free
    build.** `find_package` caches `<pkg>_DIR` in `CMakeCache.txt`, and `build/{wheel_tag}`
    is shared across shells. Once a ROS-sourced configure has run, a plain shell keeps
    "finding" `/opt/ros` — and then fails hard, because `rclcpp`'s extras run ament's
    python helpers, which need the distro's `PYTHONPATH`:

    ```
    ModuleNotFoundError: No module named 'ament_package'
    ```

    `AMENT_PREFIX_PATH` is re-read every configure, so gating on it makes the modules
    follow the shell. Sourcing (or not sourcing) a distro flips them on or off in place,
    with no need to wipe the build directory.

    **Putting `ws/install` on the prefix path before the distro is found also breaks it.**
    `QUIET` only suppresses `find_package`'s own "not found" message; it does not make a
    config file that *is* found harmless. `tesseract_msgsConfig.cmake` calls
    `find_package(ament_cmake_libraries)` internally, which hard-errors with no distro
    sourced. Any machine with an overlay in its tree would fail an ordinary ROS-free
    `pixi run install`.

## The overlay builds with the ROS side's toolchain

`rosidl` and `ament` generators need one interpreter that can import all three of:

1. **colcon** — the build orchestrator
2. **`empy`, `lark`, `catkin_pkg`** — third-party deps the generators import
3. **`rosidl_adapter`, `rosidl_parser`, `rosidl_cmake`, `ament_package`** — the distro's own
   python packages

Item 3 is the constraint. Those packages exist on **neither conda-forge nor PyPI** — only
inside `/opt/ros/<distro>/lib/pythonX.Y/site-packages`, reachable solely because
`setup.bash` puts that directory on `PYTHONPATH`. So pixi cannot supply them, and the
choice is which interpreter borrows them:

- **A pixi interpreter.** pixi can supply items 1 and 2 easily, but the directory it is
  borrowing item 3 from is named `python3.12` — so the pixi python must *be* 3.12. That
  means pinning the workspace python to whatever the distro uses.
- **The distro's interpreter** (what we do). Item 3 works natively, so no pin is needed —
  but items 1 and 2 must be installed for *that* interpreter, and pixi only installs into
  its own env, never into `/usr`. Hence the apt prerequisite.

The apt dependency and a workspace python pin are two ends of the same stick. We take the
apt dependency, because it keeps the pixi env free of ROS entirely: **any** env works,
including Humble/Iron, with none to choose between.

`build_tesseract_ros2.sh` derives the interpreter from the `pythonX.Y` component of the
distro's `PYTHONPATH` entry, checks each import up front, and names the apt package for
whatever is missing. Set `TESSERACT_ROS_PYTHON` to override it when ROS 2 is not a system
apt install.

### What comes from where

| Component | Source |
| --- | --- |
| tesseract C++ libs + CMake configs | pixi (conda `tesseract-robotics` packages), via `CMAKE_PREFIX_PATH` |
| `ros_industrial_cmake_boilerplate` | built in the workspace — it is a `build_depend` of every `tesseract_ros2` package, so colcon orders it first (listed in `dependencies_ros.rosinstall`) |
| `colcon`, `empy`, `lark`, `catkin_pkg` | apt, for the distro's python |
| `rclcpp`, message packages, `rosidl_*`, `ament_*` | the sourced distro |

## The interpreter is pinned, not discovered

CMake's `FindPython3` defaults to `Python3_FIND_VIRTUALENV=FIRST`, so a stray `VIRTUAL_ENV`
in the caller's shell — a leftover `.venv` in the repo is enough — outranks everything else
and silently wins the search. That interpreter has no `empy`, and the overlay build dies
with:

```
ModuleNotFoundError: No module named 'em'
```

...naming a module that *is* installed for the interpreter you expected to be used. The
script therefore `unset`s `VIRTUAL_ENV`, passes both `-DPython3_EXECUTABLE` and
`-DPYTHON_EXECUTABLE` (ament reads the latter, modern CMake modules the former), and
invokes colcon as `python -m colcon` so its interpreter is pinned the same way rather than
resolved off `PATH`.

The bindings build is immune to this — `scikit-build-core` always passes
`Python_EXECUTABLE` explicitly — which is why only the overlay build needed the defence.

## Wheels stay ROS-free

`TESSERACT_NANOBIND_BUILD_ROS` selects the mode:

| Value | Behaviour |
| --- | --- |
| `AUTO` (default) | build if a distro and the overlay are both found, else skip with a status message |
| `ON` | require them; configure fails naming what is missing |
| `OFF` | never build |

`scripts/build_{linux,macos}_wheel.sh` pin `OFF`, so a wheel built from a ROS-sourced shell
cannot link `/opt/ros`. Do not rely on the caller's shell being ROS-free for that
guarantee. The same variable works as an escape hatch for editable installs:

```bash
TESSERACT_NANOBIND_BUILD_ROS=OFF pixi run install   # ROS-free despite a sourced distro
TESSERACT_NANOBIND_BUILD_ROS=ON  pixi run install   # hard-fail instead of silently skipping
```

ROS-free wheels still ship the `tesseract_ros2_*` `__init__.py` and `.pyi` stubs —
`wheel.packages` takes the whole package tree — but no compiled modules, so importing them
raises `ImportError`.

## Tests

`pixi run test-ros` is separate from `test` because it sets
`PYTEST_DISABLE_PLUGIN_AUTOLOAD=1`: a sourced distro puts its `launch_testing` /
`launch_pytest` plugins on the path, and these tests only import `tesseract_robotics.*`
(never `rclpy`). The full suite cannot disable autoload — it needs `xdist`, `benchmark`,
and `testmon` — so the ROS tests get their own task. In a ROS-free build they skip via
`pytest.importorskip`.

## Troubleshooting

| Symptom | Cause |
| --- | --- |
| `ModuleNotFoundError: No module named 'em'` | the build picked the wrong interpreter — check for a stray `VIRTUAL_ENV`, or set `TESSERACT_ROS_PYTHON` |
| `ModuleNotFoundError: No module named 'ament_package'` | an interpreter without the distro's `PYTHONPATH` is running ament's helpers; the distro is not sourced in this shell |
| `Could not find a package configuration file provided by "ament_cmake_libraries"` | the overlay's configs were reached without a sourced distro |
| `ROS 2 bindings: skipped, no ROS 2 environment sourced` | expected in a plain shell; source `setup.bash` to enable |
| `ROS 2 bindings: skipped, tesseract_ros2 overlay not found` | run `pixi run build-ros` |
| `import tesseract_robotics.tesseract_ros2_monitoring` raises `ImportError` | the install was ROS-free; re-run `pixi run install` with a distro sourced |

Tested against ROS 2 Jazzy.
