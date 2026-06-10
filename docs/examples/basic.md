# Basic Examples

Fundamental examples demonstrating kinematics, collision detection, and scene graph operations.

All examples install as console scripts. Run them with:

```bash
# Installed console script (preferred)
tesseract_kinematics_example

# Or via Python module invocation
pixi run python -m tesseract_robotics.examples.tesseract_kinematics_example
```

## Kinematics Example

Forward and inverse kinematics with the KUKA LBR IIWA 14 R820 (7-DOF). The snippet
below is the core of the example — loaded directly from the shipped source.

```python title="tesseract_kinematics_example.py"
--8<-- "src/tesseract_robotics/examples/tesseract_kinematics_example.py:fk_ik"
```

Run it:

```bash
tesseract_kinematics_example
```

!!! tip "Quaternion convention — scalar-last everywhere"
    Project canonical order is **scalar-last** `[qx, qy, qz, qw]`. Build
    with `Quaterniond.from_xyzw(qx, qy, qz, qw)`; access components via
    `q.x`, `q.y`, `q.z`, `q.w` properties. The Eigen-style
    `Quaterniond(w, x, y, z)` ctor exists for direct interop but is not the
    preferred call shape.

## Collision Detection Example

Check for collisions in the environment:

```bash
tesseract_collision_example
```

See the source at
[`tesseract_collision_example.py`](https://github.com/tesseract-robotics/tesseract_nanobind/blob/main/src/tesseract_robotics/examples/tesseract_collision_example.py).

## Scene Graph Example

Manipulate the scene graph (add/remove objects):

```bash
tesseract_scene_graph_example
```

See the source at
[`scene_graph_example.py`](https://github.com/tesseract-robotics/tesseract_nanobind/blob/main/src/tesseract_robotics/examples/scene_graph_example.py).

## Geometry Showcase

All available geometry types (Box, Sphere, Cylinder, Capsule, Cone, Mesh,
ConvexMesh, Plane, Octree):

```bash
tesseract_geometry_showcase_example
```

See the source at
[`geometry_showcase_example.py`](https://github.com/tesseract-robotics/tesseract_nanobind/blob/main/src/tesseract_robotics/examples/geometry_showcase_example.py).

??? tip "Loading Mesh Files"
    ```python
    from tesseract_robotics.tesseract_geometry import Mesh

    # From file (STL, OBJ, DAE)
    mesh = Mesh.fromFile("/path/to/model.stl")

    # With scale
    mesh = Mesh.fromFile("/path/to/model.stl", scale=[0.001, 0.001, 0.001])  # mm to m
    ```

## Point cloud → Octree Example

Loads a binary PLY point cloud, builds a `tesseract_geometry.Octree`, attaches
it as a collision obstacle in front of an ABB IRB2400, and then runs **the
same `MotionProgram` through TrajOpt twice** — once with collision disabled
(robot tints red, drags through the cloud) and once with the default
collision-aware profile (robot tints green, routes around). 

```bash
tesseract_pointcloud_octree_collision_example
# custom cloud / leaf size
tesseract_pointcloud_octree_collision_example --cloud path/to/scan.ply --resolution 0.01
```

!!! tip "Toggle in the viewer"
    The viewer starts on the colliding trajectory; press **`t` + Enter** in the terminal to swap between the colliding (red) and collision-aware (green) trajectories. Press **Enter** alone to exit.

Typical output:

```text
[start] 0 contact pair(s)
[goal]  0 contact pair(s)
Running plan_freespace (collision disabled)...
Collision-disabled trajectory: 4/11 waypoints collide with the cloud
Running plan_freespace (collision enabled)...
Collision-aware trajectory: 0/11 waypoints collide
```

Source:
[`pointcloud_octree_collision_example.py`](https://github.com/tesseract-robotics/tesseract_nanobind/blob/main/src/tesseract_robotics/examples/pointcloud_octree_collision_example.py).
For the generic "octree as a collision obstacle" recipe, see the user-guide
section [Point Clouds as Collision Obstacles](../user-guide/collision.md#point-clouds-as-collision-obstacles).

**How the comparison is built**

`run()` constructs a single two-waypoint `MotionProgram` and feeds it to
`plan_freespace(...)` twice with different `ProfileDictionary` objects —
the default `create_trajopt_default_profiles()` for the collision-aware
run, and a copy with `collision_cost_config.enabled` /
`collision_constraint_config.enabled` flipped off for the colliding run.

Same start/goal/waypoint count, same robot, same env; only the profile
differs, which is what makes the result fair to eyeball.

!!! note "Why a permissive `ContactCheckProfile`?"
    `TrajOptPipeline` runs a `DiscreteContactCheckTask` *after* the
    optimizer; that task aborts the whole pipeline if any waypoint is in
    contact. Disabling collision in the TrajOpt profile only changes what
    TrajOpt optimises for — the post-check still fires. The example
    sidesteps that by registering a `ContactCheckProfile` under namespace
    `"DiscreteContactCheckTask"` with `default_margin = -10.0`, so only
    penetrations deeper than 10 m would register and the colliding plan is
    allowed to finish and return a real `CompositeInstruction`.

## Point cloud → V-HACD Convex Hulls Example

The octree example above represents a scan as occupied voxels. This one takes
the opposite, surface-fitted route: it reconstructs a mesh from a fused
point-cloud scan (`.pcd`), decomposes it into **convex hulls** with V-HACD, and
attaches those hulls to a Tesseract `Environment` as collision (and colored
visual) geometry.

Why convex hulls? Convex-vs-convex distance queries (GJK/EPA) are exact, fast,
and produce the smooth gradients optimization-based planners want — without the
per-voxel "bulging" of a coarse octree. The expensive reconstruction +
decomposition runs once, offline; the resulting hulls then query in
microseconds.

```bash
tesseract_pcd_vhacd_collision_example
# custom cloud, hull budget, units (mm cloud -> metres is the default 0.001)
tesseract_pcd_vhacd_collision_example path/to/scan.pcd --max-hulls 64 --scale 0.001
```

Pipeline: load + downsample → ball-pivoting surface reconstruction (uses the
scan normals) → `ConvexDecompositionVHACD` → `ConvexMesh` collision shapes on a
`workpiece` link → `contactTest` verification → viewer.

Typical output:

```text
loaded 1065689 points; extent (mm) = [249.1 250.1  87.6]
reconstructed mesh: 22800 verts, 40000 tris (scaled x0.001 -> extent [0.249 0.25 0.087] m)
V-HACD: 48 hulls, 772 total verts in 1.6s
env links: ['world', 'workpiece']; workpiece collision shapes: 48
probe @ [0.775 -0.075 0.025] m: 3 contact(s) within 5mm
```

!!! warning "Units: Tesseract works in metres"
    Scans are usually in millimetres. The example scales the cloud by
    `--scale` (default `0.001`, mm→m) before building geometry; pass
    `--scale 1.0` for a cloud already in metres. Skipping this places the part
    1000× too far from the origin in the viewer.

!!! note "Requires Open3D"
    Surface reconstruction uses Open3D (`pip install open3d`); it is not a hard
    dependency of the package. The example uses ball-pivoting rather than
    Poisson, which is unstable in some Open3D builds.

Source:
[`pcd_vhacd_collision_example.py`](https://github.com/tesseract-robotics/tesseract_nanobind/blob/main/src/tesseract_robotics/examples/pcd_vhacd_collision_example.py).

## Reeds-Shepp Example

Reeds-Shepp path for a differential-drive vehicle — a 2D motion planning
demonstration using OMPL:

```bash
tesseract_reeds_shepp_example
```

See the source at
[`reeds_shepp_example.py`](https://github.com/tesseract-robotics/tesseract_nanobind/blob/main/src/tesseract_robotics/examples/reeds_shepp_example.py).

## Next Steps

- [Planning Examples](planning.md) - Motion planning
- [Online Planning](online-planning.md) - Real-time replanning
