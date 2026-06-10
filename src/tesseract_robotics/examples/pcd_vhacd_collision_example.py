"""Point cloud -> V-HACD convex hulls -> Tesseract collision Environment.

This example takes a fused point-cloud scan (``.pcd``), reconstructs a surface
mesh, decomposes it into convex hulls with V-HACD, and exports those hulls as
collision (and visual) geometry attached to a Tesseract ``Environment``.

Why convex hulls instead of an octree? Convex-vs-convex distance queries
(GJK/EPA) are exact, fast, and give the smooth gradients optimization-based
planners (TrajOpt) want -- without the per-voxel "bulging" of a coarse octree.
The expensive work (reconstruction + decomposition) happens once, offline; the
resulting hulls then query in microseconds.

Pipeline:
    1. Load + downsample the cloud (units are assumed to be millimetres).
    2. Surface reconstruction (ball-pivoting, using the scan normals).
    3. V-HACD convex decomposition  -> list[ConvexMesh].
    4. Attach the hulls to an Environment as a single "workpiece" link.
    5. Verify with a Bullet ``contactTest`` and (optionally) open the viewer.

Requires Open3D for reconstruction::

    pip install open3d   # (cp312 wheel; not a hard dependency of this package)

Run::

    python -m tesseract_robotics.examples.pcd_vhacd_collision_example \
        docs/assets/5_fused.pcd --max-hulls 64
"""

import argparse
import sys
import time
from pathlib import Path

import numpy as np

from tesseract_robotics.tesseract_collision import (
    ConvexDecompositionVHACD,
    VHACDParameters,
    VHACDFillMode,
    ContactManagersPluginFactory,
    ContactRequest,
    ContactResultMap,
    ContactResultVector,
    ContactTestType_ALL,
)
from tesseract_robotics.tesseract_common import (
    CollisionMarginData,
    GeneralResourceLocator,
    Isometry3d,
)
from tesseract_robotics.tesseract_geometry import Sphere
from tesseract_robotics.tesseract_scene_graph import (
    Collision,
    Joint,
    JointType,
    Link,
    Material,
    SceneGraph,
    Visual,
)
from tesseract_robotics.tesseract_environment import AddLinkCommand, Environment


# Default location of the sample cloud shipped under the repo's docs/.
_DEFAULT_PCD = Path(__file__).resolve().parents[3] / "docs" / "assets" / "5_fused.pcd"


def reconstruct_surface(pcd_path, scale=0.001, downsample_mm=1.0, max_tris=40000):
    """Load a .pcd and return (vertices Nx3 in metres, triangles Mx3, pts) via ball-pivoting.

    Tesseract works in metres; scans are commonly in millimetres, so vertices and
    points are multiplied by ``scale`` (default 0.001 = mm->m). Pass ``scale=1.0``
    for a cloud already in metres.
    """
    try:
        import open3d as o3d
    except ImportError:
        sys.exit("This example needs Open3D for reconstruction:  pip install open3d")

    pcd = o3d.io.read_point_cloud(str(pcd_path))
    pts = np.asarray(pcd.points)
    if len(pts) == 0:
        sys.exit(f"No points read from {pcd_path}")
    print(f"loaded {len(pts)} points; extent (mm) = {(pts.max(0) - pts.min(0)).round(1)}")

    pcd = pcd.voxel_down_sample(voxel_size=downsample_mm)
    if not pcd.has_normals():
        pcd.estimate_normals(o3d.geometry.KDTreeSearchParamKNN(knn=30))
        pcd.orient_normals_consistent_tangent_plane(30)

    t0 = time.perf_counter()
    avg = float(np.mean(pcd.compute_nearest_neighbor_distance()))
    radii = o3d.utility.DoubleVector([1.5 * avg, 3.0 * avg, 6.0 * avg])
    # Ball-pivoting reuses the scan normals and avoids Poisson's loop-closing
    # step (which is unstable in some Open3D builds).
    mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(pcd, radii)
    if len(mesh.triangles) < 1000:
        mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_alpha_shape(pcd, alpha=3.0 * avg)
    mesh.remove_degenerate_triangles()
    mesh.remove_duplicated_triangles()
    mesh.remove_unreferenced_vertices()
    if len(mesh.triangles) > max_tris:
        mesh = mesh.simplify_quadric_decimation(max_tris)
        mesh.remove_degenerate_triangles()

    V = np.asarray(mesh.vertices) * scale
    T = np.asarray(mesh.triangles)
    pts = pts * scale
    print(f"reconstructed mesh: {len(V)} verts, {len(T)} tris in {time.perf_counter() - t0:.1f}s "
          f"(scaled x{scale} -> extent {(V.max(0) - V.min(0)).round(3)} m)")
    return V, T, pts


def decompose(vertices, triangles, max_hulls=64, resolution=400000):
    """Run V-HACD; returns a list of ConvexMesh hulls."""
    # tesseract face format: each triangle is [3, i0, i1, i2]
    faces = np.empty(len(triangles) * 4, dtype=np.int32)
    faces[0::4] = 3
    faces[1::4], faces[2::4], faces[3::4] = triangles[:, 0], triangles[:, 1], triangles[:, 2]

    params = VHACDParameters()
    params.max_convex_hulls = max_hulls
    params.resolution = resolution
    params.max_num_vertices_per_ch = 64
    params.fill_mode = VHACDFillMode.FLOOD_FILL

    t0 = time.perf_counter()
    hulls = ConvexDecompositionVHACD(params).compute(vertices, faces, False)
    total_v = sum(h.getVertexCount() for h in hulls)
    print(f"V-HACD: {len(hulls)} hulls, {total_v} total verts in {time.perf_counter() - t0:.2f}s")
    return hulls


def _hull_color(i, n):
    """Distinct RGBA per hull so the decomposition is visible in the viewer."""
    import colorsys

    r, g, b = colorsys.hsv_to_rgb((i / max(n, 1)) % 1.0, 0.65, 0.95)
    return np.array([r, g, b, 1.0])


def build_environment(hulls):
    """Create an Environment with the hulls as the 'workpiece' link (visual + collision)."""
    scene = SceneGraph()
    scene.addLink(Link("world"))
    scene.setRoot("world")
    env = Environment()
    if not env.init(scene):
        raise RuntimeError("Environment.init(scene_graph) failed")

    link = Link("workpiece")
    ident = Isometry3d.Identity()
    for i, hull in enumerate(hulls):
        collision = Collision()
        collision.origin = ident
        collision.geometry = hull
        link.addCollision(collision)

        visual = Visual()
        visual.origin = ident
        visual.geometry = hull
        mat = Material(f"hull_{i}")
        mat.color = _hull_color(i, len(hulls))
        visual.material = mat
        link.addVisual(visual)

    joint = Joint("joint_workpiece")
    joint.parent_link_name = "world"
    joint.child_link_name = "workpiece"
    joint.type = JointType.FIXED
    if not env.applyCommand(AddLinkCommand(link, joint)):
        raise RuntimeError("AddLinkCommand failed")
    print(f"env links: {list(env.getLinkNames())}; "
          f"workpiece collision shapes: {len(env.getLink('workpiece').collision)}")
    return env


def verify_collision(env, probe_point_m, margin_m=0.005, probe_radius_m=0.002):
    """Create a Bullet manager from the hull geometries and probe one point (metres)."""
    # A scene-graph-initialised Environment has no contact-manager plugins, so
    # build one directly from the installed Bullet plugin config.
    cfg = """
contact_manager_plugins:
  search_libraries:
    - tesseract_collision_bullet_factories
  discrete_plugins:
    default: BulletDiscreteBVHManager
    plugins:
      BulletDiscreteBVHManager:
        class: BulletDiscreteBVHManagerFactory
"""
    factory = ContactManagersPluginFactory(cfg, GeneralResourceLocator())
    mgr = factory.createDiscreteContactManager("BulletDiscreteBVHManager")
    if mgr is None:
        print("WARNING: could not create Bullet manager; skipping collision check")
        return

    ident = Isometry3d.Identity()
    hull_geoms = [c.geometry for c in env.getLink("workpiece").collision]
    mgr.addCollisionObject("workpiece", 0, hull_geoms, [ident] * len(hull_geoms), True)
    mgr.addCollisionObject("probe", 0, [Sphere(probe_radius_m)], [ident], True)
    mgr.setActiveCollisionObjects(["probe"])
    mgr.setCollisionMarginData(CollisionMarginData(margin_m))

    T = np.eye(4)
    T[:3, 3] = probe_point_m
    mgr.setCollisionObjectsTransform("probe", Isometry3d(T))

    res = ContactResultMap()
    mgr.contactTest(res, ContactRequest(ContactTestType_ALL))
    flat = ContactResultVector()
    res.flattenMoveResults(flat)
    print(f"probe @ {np.round(probe_point_m, 3)} m: {len(flat)} contact(s) within {margin_m * 1000:.0f}mm")
    for i in range(min(3, len(flat))):
        r = flat[i]
        print(f"   {r.link_names[0]} <-> {r.link_names[1]}: distance = {r.distance * 1000:.2f} mm")


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("pcd", nargs="?", default=str(_DEFAULT_PCD), help="path to .pcd cloud")
    ap.add_argument("--max-hulls", type=int, default=64)
    ap.add_argument("--scale", type=float, default=0.001,
                    help="cloud-units -> metres (default 0.001 = mm; use 1.0 if already metres)")
    ap.add_argument("--no-view", action="store_true", help="skip launching the viewer")
    ap.add_argument("--port", type=int, default=8000)
    args = ap.parse_args()

    V, T, pts = reconstruct_surface(args.pcd, scale=args.scale)
    hulls = decompose(V, T, max_hulls=args.max_hulls)
    env = build_environment(hulls)
    verify_collision(env, pts.mean(0))

    if args.no_view:
        return
    from tesseract_robotics.viewer import TesseractViewer

    viewer = TesseractViewer(server_address=("127.0.0.1", args.port))
    viewer.update_environment(env, [0, 0, 0])
    viewer.start_serve_background()
    print(f"\nViewer serving at http://localhost:{args.port}  (Ctrl-C / Enter to exit)")
    try:
        input("Press Enter to exit...")
    except (EOFError, KeyboardInterrupt):
        pass


if __name__ == "__main__":
    main()
