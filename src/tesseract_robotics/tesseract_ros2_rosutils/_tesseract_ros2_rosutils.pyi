from collections.abc import Sequence

import tesseract_robotics.tesseract_common._tesseract_common
import tesseract_robotics.tesseract_environment._tesseract_environment
import tesseract_robotics.tesseract_state_solver._tesseract_state_solver


class Time:
    def __init__(self) -> None: ...

    @property
    def sec(self) -> int: ...

    @sec.setter
    def sec(self, arg: int, /) -> None: ...

    @property
    def nanosec(self) -> int: ...

    @nanosec.setter
    def nanosec(self, arg: int, /) -> None: ...

class Duration:
    def __init__(self) -> None: ...

    @property
    def sec(self) -> int: ...

    @sec.setter
    def sec(self, arg: int, /) -> None: ...

    @property
    def nanosec(self) -> int: ...

    @nanosec.setter
    def nanosec(self, arg: int, /) -> None: ...

class Header:
    def __init__(self) -> None: ...

    @property
    def stamp(self) -> Time: ...

    @stamp.setter
    def stamp(self, arg: Time, /) -> None: ...

    @property
    def frame_id(self) -> str: ...

    @frame_id.setter
    def frame_id(self, arg: str, /) -> None: ...

class ColorRGBA:
    def __init__(self) -> None: ...

    @property
    def r(self) -> float: ...

    @r.setter
    def r(self, arg: float, /) -> None: ...

    @property
    def g(self) -> float: ...

    @g.setter
    def g(self, arg: float, /) -> None: ...

    @property
    def b(self) -> float: ...

    @b.setter
    def b(self, arg: float, /) -> None: ...

    @property
    def a(self) -> float: ...

    @a.setter
    def a(self, arg: float, /) -> None: ...

class Vector3:
    def __init__(self) -> None: ...

    @property
    def x(self) -> float: ...

    @x.setter
    def x(self, arg: float, /) -> None: ...

    @property
    def y(self) -> float: ...

    @y.setter
    def y(self, arg: float, /) -> None: ...

    @property
    def z(self) -> float: ...

    @z.setter
    def z(self, arg: float, /) -> None: ...

class Point:
    def __init__(self) -> None: ...

    @property
    def x(self) -> float: ...

    @x.setter
    def x(self, arg: float, /) -> None: ...

    @property
    def y(self) -> float: ...

    @y.setter
    def y(self, arg: float, /) -> None: ...

    @property
    def z(self) -> float: ...

    @z.setter
    def z(self, arg: float, /) -> None: ...

class Quaternion:
    def __init__(self) -> None: ...

    @property
    def x(self) -> float: ...

    @x.setter
    def x(self, arg: float, /) -> None: ...

    @property
    def y(self) -> float: ...

    @y.setter
    def y(self, arg: float, /) -> None: ...

    @property
    def z(self) -> float: ...

    @z.setter
    def z(self, arg: float, /) -> None: ...

    @property
    def w(self) -> float: ...

    @w.setter
    def w(self, arg: float, /) -> None: ...

class Pose:
    def __init__(self) -> None: ...

    @property
    def position(self) -> Point: ...

    @position.setter
    def position(self, arg: Point, /) -> None: ...

    @property
    def orientation(self) -> Quaternion: ...

    @orientation.setter
    def orientation(self, arg: Quaternion, /) -> None: ...

class PoseArray:
    def __init__(self) -> None: ...

    @property
    def header(self) -> Header: ...

    @header.setter
    def header(self, arg: Header, /) -> None: ...

    @property
    def poses(self) -> list[Pose]: ...

    @poses.setter
    def poses(self, arg: Sequence[Pose], /) -> None: ...

class SensorJointState:
    def __init__(self) -> None: ...

    @property
    def header(self) -> Header: ...

    @header.setter
    def header(self, arg: Header, /) -> None: ...

    @property
    def name(self) -> list[str]: ...

    @name.setter
    def name(self, arg: Sequence[str], /) -> None: ...

    @property
    def position(self) -> list[float]: ...

    @position.setter
    def position(self, arg: Sequence[float], /) -> None: ...

    @property
    def velocity(self) -> list[float]: ...

    @velocity.setter
    def velocity(self, arg: Sequence[float], /) -> None: ...

    @property
    def effort(self) -> list[float]: ...

    @effort.setter
    def effort(self, arg: Sequence[float], /) -> None: ...

class JointTrajectoryPoint:
    def __init__(self) -> None: ...

    @property
    def positions(self) -> list[float]: ...

    @positions.setter
    def positions(self, arg: Sequence[float], /) -> None: ...

    @property
    def velocities(self) -> list[float]: ...

    @velocities.setter
    def velocities(self, arg: Sequence[float], /) -> None: ...

    @property
    def accelerations(self) -> list[float]: ...

    @accelerations.setter
    def accelerations(self, arg: Sequence[float], /) -> None: ...

    @property
    def effort(self) -> list[float]: ...

    @effort.setter
    def effort(self, arg: Sequence[float], /) -> None: ...

    @property
    def time_from_start(self) -> Duration: ...

    @time_from_start.setter
    def time_from_start(self, arg: Duration, /) -> None: ...

class TrajectoryMsgsJointTrajectory:
    def __init__(self) -> None: ...

    @property
    def header(self) -> Header: ...

    @header.setter
    def header(self, arg: Header, /) -> None: ...

    @property
    def joint_names(self) -> list[str]: ...

    @joint_names.setter
    def joint_names(self, arg: Sequence[str], /) -> None: ...

    @property
    def points(self) -> list[JointTrajectoryPoint]: ...

    @points.setter
    def points(self, arg: Sequence[JointTrajectoryPoint], /) -> None: ...

class JointCalibration:
    def __init__(self) -> None: ...

    @property
    def reference_position(self) -> float: ...

    @reference_position.setter
    def reference_position(self, arg: float, /) -> None: ...

    @property
    def rising(self) -> float: ...

    @rising.setter
    def rising(self, arg: float, /) -> None: ...

    @property
    def falling(self) -> float: ...

    @falling.setter
    def falling(self, arg: float, /) -> None: ...

    @property
    def empty(self) -> bool: ...

    @empty.setter
    def empty(self, arg: bool, /) -> None: ...

class JointDynamics:
    def __init__(self) -> None: ...

    @property
    def damping(self) -> float: ...

    @damping.setter
    def damping(self, arg: float, /) -> None: ...

    @property
    def friction(self) -> float: ...

    @friction.setter
    def friction(self, arg: float, /) -> None: ...

    @property
    def empty(self) -> bool: ...

    @empty.setter
    def empty(self, arg: bool, /) -> None: ...

class JointLimits:
    def __init__(self) -> None: ...

    @property
    def lower(self) -> float: ...

    @lower.setter
    def lower(self, arg: float, /) -> None: ...

    @property
    def upper(self) -> float: ...

    @upper.setter
    def upper(self, arg: float, /) -> None: ...

    @property
    def effort(self) -> float: ...

    @effort.setter
    def effort(self, arg: float, /) -> None: ...

    @property
    def velocity(self) -> float: ...

    @velocity.setter
    def velocity(self, arg: float, /) -> None: ...

    @property
    def acceleration(self) -> float: ...

    @acceleration.setter
    def acceleration(self, arg: float, /) -> None: ...

    @property
    def empty(self) -> bool: ...

    @empty.setter
    def empty(self, arg: bool, /) -> None: ...

class JointMimic:
    def __init__(self) -> None: ...

    @property
    def offset(self) -> float: ...

    @offset.setter
    def offset(self, arg: float, /) -> None: ...

    @property
    def multiplier(self) -> float: ...

    @multiplier.setter
    def multiplier(self, arg: float, /) -> None: ...

    @property
    def joint_name(self) -> str: ...

    @joint_name.setter
    def joint_name(self, arg: str, /) -> None: ...

    @property
    def empty(self) -> bool: ...

    @empty.setter
    def empty(self, arg: bool, /) -> None: ...

class JointSafety:
    def __init__(self) -> None: ...

    @property
    def soft_upper_limit(self) -> float: ...

    @soft_upper_limit.setter
    def soft_upper_limit(self, arg: float, /) -> None: ...

    @property
    def soft_lower_limit(self) -> float: ...

    @soft_lower_limit.setter
    def soft_lower_limit(self, arg: float, /) -> None: ...

    @property
    def k_position(self) -> float: ...

    @k_position.setter
    def k_position(self, arg: float, /) -> None: ...

    @property
    def k_velocity(self) -> float: ...

    @k_velocity.setter
    def k_velocity(self, arg: float, /) -> None: ...

    @property
    def empty(self) -> bool: ...

    @empty.setter
    def empty(self, arg: bool, /) -> None: ...

class Material:
    def __init__(self) -> None: ...

    @property
    def name(self) -> str: ...

    @name.setter
    def name(self, arg: str, /) -> None: ...

    @property
    def texture_filename(self) -> str: ...

    @texture_filename.setter
    def texture_filename(self, arg: str, /) -> None: ...

    @property
    def color(self) -> ColorRGBA: ...

    @color.setter
    def color(self, arg: ColorRGBA, /) -> None: ...

    @property
    def empty(self) -> bool: ...

    @empty.setter
    def empty(self, arg: bool, /) -> None: ...

class Mesh:
    def __init__(self) -> None: ...

    @property
    def vertices(self) -> list[Point]: ...

    @vertices.setter
    def vertices(self, arg: Sequence[Point], /) -> None: ...

    @property
    def faces(self) -> list[int]: ...

    @faces.setter
    def faces(self, arg: Sequence[int], /) -> None: ...

    @property
    def file_path(self) -> str: ...

    @file_path.setter
    def file_path(self, arg: str, /) -> None: ...

    @property
    def scale(self) -> list[float]: ...

    @scale.setter
    def scale(self, arg: Sequence[float], /) -> None: ...

class Geometry:
    def __init__(self) -> None: ...

    @property
    def type(self) -> int: ...

    @type.setter
    def type(self, arg: int, /) -> None: ...

    @property
    def uuid(self) -> str: ...

    @uuid.setter
    def uuid(self, arg: str, /) -> None: ...

    @property
    def sphere_radius(self) -> float: ...

    @sphere_radius.setter
    def sphere_radius(self, arg: float, /) -> None: ...

    @property
    def cylinder_dimensions(self) -> list[float]: ...

    @cylinder_dimensions.setter
    def cylinder_dimensions(self, arg: Sequence[float], /) -> None: ...

    @property
    def capsule_dimensions(self) -> list[float]: ...

    @capsule_dimensions.setter
    def capsule_dimensions(self, arg: Sequence[float], /) -> None: ...

    @property
    def cone_dimensions(self) -> list[float]: ...

    @cone_dimensions.setter
    def cone_dimensions(self, arg: Sequence[float], /) -> None: ...

    @property
    def box_dimensions(self) -> list[float]: ...

    @box_dimensions.setter
    def box_dimensions(self, arg: Sequence[float], /) -> None: ...

    @property
    def plane_coeff(self) -> list[float]: ...

    @plane_coeff.setter
    def plane_coeff(self, arg: Sequence[float], /) -> None: ...

    @property
    def mesh(self) -> Mesh: ...

    @mesh.setter
    def mesh(self, arg: Mesh, /) -> None: ...

    SPHERE: int = 1

    CYLINDER: int = 2

    CONE: int = 3

    BOX: int = 4

    PLANE: int = 5

    MESH: int = 6

    CONVEX_MESH: int = 7

    SIGNED_DISTANCE_FIELD: int = 8

    OCTREE: int = 9

    CAPSULE: int = 10

    POLYGON_MESH: int = 11

    COMPOUND_MESH: int = 12

class Inertial:
    def __init__(self) -> None: ...

    @property
    def origin(self) -> Pose: ...

    @origin.setter
    def origin(self, arg: Pose, /) -> None: ...

    @property
    def mass(self) -> float: ...

    @mass.setter
    def mass(self, arg: float, /) -> None: ...

    @property
    def ixx(self) -> float: ...

    @ixx.setter
    def ixx(self, arg: float, /) -> None: ...

    @property
    def ixy(self) -> float: ...

    @ixy.setter
    def ixy(self, arg: float, /) -> None: ...

    @property
    def ixz(self) -> float: ...

    @ixz.setter
    def ixz(self, arg: float, /) -> None: ...

    @property
    def iyy(self) -> float: ...

    @iyy.setter
    def iyy(self, arg: float, /) -> None: ...

    @property
    def iyz(self) -> float: ...

    @iyz.setter
    def iyz(self, arg: float, /) -> None: ...

    @property
    def izz(self) -> float: ...

    @izz.setter
    def izz(self, arg: float, /) -> None: ...

    @property
    def empty(self) -> bool: ...

    @empty.setter
    def empty(self, arg: bool, /) -> None: ...

class AllowedCollisionEntry:
    def __init__(self) -> None: ...

    @property
    def link_1(self) -> str: ...

    @link_1.setter
    def link_1(self, arg: str, /) -> None: ...

    @property
    def link_2(self) -> str: ...

    @link_2.setter
    def link_2(self, arg: str, /) -> None: ...

    @property
    def reason(self) -> str: ...

    @reason.setter
    def reason(self, arg: str, /) -> None: ...

class CollisionMarginData:
    def __init__(self) -> None: ...

    @property
    def default_margin(self) -> float: ...

    @default_margin.setter
    def default_margin(self, arg: float, /) -> None: ...

class StringDoublePair:
    def __init__(self) -> None: ...

    @property
    def first(self) -> str: ...

    @first.setter
    def first(self, arg: str, /) -> None: ...

    @property
    def second(self) -> float: ...

    @second.setter
    def second(self, arg: float, /) -> None: ...

class TesseractMsgsJointState:
    def __init__(self) -> None: ...

    @property
    def joint_names(self) -> list[str]: ...

    @joint_names.setter
    def joint_names(self, arg: Sequence[str], /) -> None: ...

    @property
    def position(self) -> list[float]: ...

    @position.setter
    def position(self, arg: Sequence[float], /) -> None: ...

    @property
    def velocity(self) -> list[float]: ...

    @velocity.setter
    def velocity(self, arg: Sequence[float], /) -> None: ...

    @property
    def acceleration(self) -> list[float]: ...

    @acceleration.setter
    def acceleration(self, arg: Sequence[float], /) -> None: ...

    @property
    def effort(self) -> list[float]: ...

    @effort.setter
    def effort(self, arg: Sequence[float], /) -> None: ...

    @property
    def time_from_start(self) -> Duration: ...

    @time_from_start.setter
    def time_from_start(self, arg: Duration, /) -> None: ...

class TesseractMsgsJointTrajectory:
    def __init__(self) -> None: ...

    @property
    def states(self) -> list[TesseractMsgsJointState]: ...

    @states.setter
    def states(self, arg: Sequence[TesseractMsgsJointState], /) -> None: ...

    @property
    def description(self) -> str: ...

    @description.setter
    def description(self, arg: str, /) -> None: ...

    @property
    def uuid(self) -> str: ...

    @uuid.setter
    def uuid(self, arg: str, /) -> None: ...

class VisualGeometry:
    def __init__(self) -> None: ...

    @property
    def name(self) -> str: ...

    @name.setter
    def name(self, arg: str, /) -> None: ...

    @property
    def origin(self) -> Pose: ...

    @origin.setter
    def origin(self, arg: Pose, /) -> None: ...

    @property
    def geometry(self) -> Geometry: ...

    @geometry.setter
    def geometry(self, arg: Geometry, /) -> None: ...

    @property
    def material(self) -> Material: ...

    @material.setter
    def material(self, arg: Material, /) -> None: ...

class CollisionGeometry:
    def __init__(self) -> None: ...

    @property
    def name(self) -> str: ...

    @name.setter
    def name(self, arg: str, /) -> None: ...

    @property
    def origin(self) -> Pose: ...

    @origin.setter
    def origin(self, arg: Pose, /) -> None: ...

    @property
    def geometry(self) -> Geometry: ...

    @geometry.setter
    def geometry(self, arg: Geometry, /) -> None: ...

    @property
    def material(self) -> Material: ...

    @material.setter
    def material(self, arg: Material, /) -> None: ...

class Link:
    def __init__(self) -> None: ...

    @property
    def name(self) -> str: ...

    @name.setter
    def name(self, arg: str, /) -> None: ...

    @property
    def inertial(self) -> Inertial: ...

    @inertial.setter
    def inertial(self, arg: Inertial, /) -> None: ...

    @property
    def visual(self) -> list[VisualGeometry]: ...

    @visual.setter
    def visual(self, arg: Sequence[VisualGeometry], /) -> None: ...

    @property
    def collision(self) -> list[CollisionGeometry]: ...

    @collision.setter
    def collision(self, arg: Sequence[CollisionGeometry], /) -> None: ...

class Joint:
    def __init__(self) -> None: ...

    @property
    def name(self) -> str: ...

    @name.setter
    def name(self, arg: str, /) -> None: ...

    @property
    def type(self) -> int: ...

    @type.setter
    def type(self, arg: int, /) -> None: ...

    @property
    def axis(self) -> list[float]: ...

    @axis.setter
    def axis(self, arg: Sequence[float], /) -> None: ...

    @property
    def child_link_name(self) -> str: ...

    @child_link_name.setter
    def child_link_name(self, arg: str, /) -> None: ...

    @property
    def parent_link_name(self) -> str: ...

    @parent_link_name.setter
    def parent_link_name(self, arg: str, /) -> None: ...

    @property
    def parent_to_joint_origin_transform(self) -> Pose: ...

    @parent_to_joint_origin_transform.setter
    def parent_to_joint_origin_transform(self, arg: Pose, /) -> None: ...

    @property
    def limits(self) -> JointLimits: ...

    @limits.setter
    def limits(self, arg: JointLimits, /) -> None: ...

    @property
    def dynamics(self) -> JointDynamics: ...

    @dynamics.setter
    def dynamics(self, arg: JointDynamics, /) -> None: ...

    @property
    def safety(self) -> JointSafety: ...

    @safety.setter
    def safety(self, arg: JointSafety, /) -> None: ...

    @property
    def calibration(self) -> JointCalibration: ...

    @calibration.setter
    def calibration(self, arg: JointCalibration, /) -> None: ...

    @property
    def mimic(self) -> JointMimic: ...

    @mimic.setter
    def mimic(self, arg: JointMimic, /) -> None: ...

    UNKNOWN: int = 0

    REVOLUTE: int = 1

    CONTINUOUS: int = 2

    PRISMATIC: int = 3

    FLOATING: int = 4

    PLANAR: int = 5

    FIXED: int = 6

class SceneGraph:
    def __init__(self) -> None: ...

    @property
    def id(self) -> str: ...

    @id.setter
    def id(self, arg: str, /) -> None: ...

    @property
    def root(self) -> str: ...

    @root.setter
    def root(self, arg: str, /) -> None: ...

    @property
    def links(self) -> list[Link]: ...

    @links.setter
    def links(self, arg: Sequence[Link], /) -> None: ...

    @property
    def joints(self) -> list[Joint]: ...

    @joints.setter
    def joints(self, arg: Sequence[Joint], /) -> None: ...

    @property
    def invisible_links(self) -> list[str]: ...

    @invisible_links.setter
    def invisible_links(self, arg: Sequence[str], /) -> None: ...

    @property
    def disabled_collision_links(self) -> list[str]: ...

    @disabled_collision_links.setter
    def disabled_collision_links(self, arg: Sequence[str], /) -> None: ...

    @property
    def acm(self) -> list[AllowedCollisionEntry]: ...

    @acm.setter
    def acm(self, arg: Sequence[AllowedCollisionEntry], /) -> None: ...

class EnvironmentCommand:
    def __init__(self) -> None: ...

    @property
    def command(self) -> int: ...

    @command.setter
    def command(self, arg: int, /) -> None: ...

    @property
    def add_link(self) -> Link: ...

    @add_link.setter
    def add_link(self, arg: Link, /) -> None: ...

    @property
    def add_joint(self) -> Joint: ...

    @add_joint.setter
    def add_joint(self, arg: Joint, /) -> None: ...

    @property
    def add_replace_allowed(self) -> bool: ...

    @add_replace_allowed.setter
    def add_replace_allowed(self, arg: bool, /) -> None: ...

    @property
    def move_link_joint(self) -> Joint: ...

    @move_link_joint.setter
    def move_link_joint(self, arg: Joint, /) -> None: ...

    @property
    def move_joint_name(self) -> str: ...

    @move_joint_name.setter
    def move_joint_name(self, arg: str, /) -> None: ...

    @property
    def move_joint_parent_link(self) -> str: ...

    @move_joint_parent_link.setter
    def move_joint_parent_link(self, arg: str, /) -> None: ...

    @property
    def remove_link(self) -> str: ...

    @remove_link.setter
    def remove_link(self, arg: str, /) -> None: ...

    @property
    def remove_joint(self) -> str: ...

    @remove_joint.setter
    def remove_joint(self, arg: str, /) -> None: ...

    @property
    def replace_joint(self) -> Joint: ...

    @replace_joint.setter
    def replace_joint(self, arg: Joint, /) -> None: ...

    @property
    def change_link_origin_name(self) -> str: ...

    @change_link_origin_name.setter
    def change_link_origin_name(self, arg: str, /) -> None: ...

    @property
    def change_link_origin_pose(self) -> Pose: ...

    @change_link_origin_pose.setter
    def change_link_origin_pose(self, arg: Pose, /) -> None: ...

    @property
    def change_joint_origin_name(self) -> str: ...

    @change_joint_origin_name.setter
    def change_joint_origin_name(self, arg: str, /) -> None: ...

    @property
    def change_joint_origin_pose(self) -> Pose: ...

    @change_joint_origin_pose.setter
    def change_joint_origin_pose(self, arg: Pose, /) -> None: ...

    @property
    def change_link_collision_enabled_name(self) -> str: ...

    @change_link_collision_enabled_name.setter
    def change_link_collision_enabled_name(self, arg: str, /) -> None: ...

    @property
    def change_link_collision_enabled_value(self) -> bool: ...

    @change_link_collision_enabled_value.setter
    def change_link_collision_enabled_value(self, arg: bool, /) -> None: ...

    @property
    def change_link_visibility_name(self) -> str: ...

    @change_link_visibility_name.setter
    def change_link_visibility_name(self, arg: str, /) -> None: ...

    @property
    def change_link_visibility_value(self) -> bool: ...

    @change_link_visibility_value.setter
    def change_link_visibility_value(self, arg: bool, /) -> None: ...

    @property
    def modify_allowed_collisions_type(self) -> int: ...

    @modify_allowed_collisions_type.setter
    def modify_allowed_collisions_type(self, arg: int, /) -> None: ...

    @property
    def modify_allowed_collisions(self) -> list[AllowedCollisionEntry]: ...

    @modify_allowed_collisions.setter
    def modify_allowed_collisions(self, arg: Sequence[AllowedCollisionEntry], /) -> None: ...

    @property
    def remove_allowed_collision_link(self) -> str: ...

    @remove_allowed_collision_link.setter
    def remove_allowed_collision_link(self, arg: str, /) -> None: ...

    @property
    def joint_state(self) -> SensorJointState: ...

    @joint_state.setter
    def joint_state(self, arg: SensorJointState, /) -> None: ...

    @property
    def scene_graph(self) -> SceneGraph: ...

    @scene_graph.setter
    def scene_graph(self, arg: SceneGraph, /) -> None: ...

    @property
    def scene_graph_joint(self) -> Joint: ...

    @scene_graph_joint.setter
    def scene_graph_joint(self, arg: Joint, /) -> None: ...

    @property
    def scene_graph_prefix(self) -> str: ...

    @scene_graph_prefix.setter
    def scene_graph_prefix(self, arg: str, /) -> None: ...

    @property
    def has_collision_default_margin(self) -> bool: ...

    @has_collision_default_margin.setter
    def has_collision_default_margin(self, arg: bool, /) -> None: ...

    @property
    def collision_default_margin(self) -> float: ...

    @collision_default_margin.setter
    def collision_default_margin(self, arg: float, /) -> None: ...

    @property
    def set_active_discrete_contact_manager(self) -> str: ...

    @set_active_discrete_contact_manager.setter
    def set_active_discrete_contact_manager(self, arg: str, /) -> None: ...

    @property
    def set_active_continuous_contact_manager(self) -> str: ...

    @set_active_continuous_contact_manager.setter
    def set_active_continuous_contact_manager(self, arg: str, /) -> None: ...

    @property
    def add_trajectory_link_name(self) -> str: ...

    @add_trajectory_link_name.setter
    def add_trajectory_link_name(self, arg: str, /) -> None: ...

    @property
    def add_trajectory_link_parent_name(self) -> str: ...

    @add_trajectory_link_parent_name.setter
    def add_trajectory_link_parent_name(self, arg: str, /) -> None: ...

    @property
    def add_trajectory_link_traj(self) -> TesseractMsgsJointTrajectory: ...

    @add_trajectory_link_traj.setter
    def add_trajectory_link_traj(self, arg: TesseractMsgsJointTrajectory, /) -> None: ...

    @property
    def add_trajectory_link_replace_allowed(self) -> bool: ...

    @add_trajectory_link_replace_allowed.setter
    def add_trajectory_link_replace_allowed(self, arg: bool, /) -> None: ...

    ADD_LINK: int = 0

    MOVE_LINK: int = 1

    MOVE_JOINT: int = 2

    REMOVE_LINK: int = 3

    REMOVE_JOINT: int = 4

    CHANGE_LINK_ORIGIN: int = 5

    CHANGE_JOINT_ORIGIN: int = 6

    CHANGE_LINK_COLLISION_ENABLED: int = 7

    CHANGE_LINK_VISIBILITY: int = 8

    MODIFY_ALLOWED_COLLISIONS: int = 9

    REMOVE_ALLOWED_COLLISION_LINK: int = 10

    UPDATE_JOINT_STATE: int = 11

    ADD_SCENE_GRAPH: int = 12

    CHANGE_JOINT_POSITION_LIMITS: int = 13

    CHANGE_JOINT_VELOCITY_LIMITS: int = 14

    CHANGE_JOINT_ACCELERATION_LIMITS: int = 15

    ADD_KINEMATICS_INFORMATION: int = 16

    REPLACE_JOINT: int = 17

    CHANGE_COLLISION_MARGINS: int = 18

    ADD_CONTACT_MANAGERS_PLUGIN_INFO: int = 19

    SET_ACTIVE_DISCRETE_CONTACT_MANAGER: int = 20

    SET_ACTIVE_CONTINUOUS_CONTACT_MANAGER: int = 21

    ADD_TRAJECTORY_LINK: int = 22

def iso_to_pose(iso: tesseract_robotics.tesseract_common._tesseract_common.Isometry3d) -> Pose:
    """Convert an Eigen::Isometry3d to a geometry_msgs::msg::Pose."""

def pose_to_iso(pose: Pose) -> tesseract_robotics.tesseract_common._tesseract_common.Isometry3d:
    """Convert a geometry_msgs::msg::Pose to an Eigen::Isometry3d."""

def isos_to_pose_array(transforms: Sequence[tesseract_robotics.tesseract_common._tesseract_common.Isometry3d]) -> PoseArray:
    """
    Convert a list of Eigen::Isometry3d to a geometry_msgs::msg::PoseArray.
    """

def command_to_msg(command: tesseract_robotics.tesseract_environment._tesseract_environment.Command) -> EnvironmentCommand:
    """
    Convert a tesseract_environment::Command to a tesseract_msgs::msg::EnvironmentCommand.
    """

def commands_to_msg(commands: Sequence[tesseract_robotics.tesseract_environment._tesseract_environment.Command], past_revision: int = 0) -> list[EnvironmentCommand]:
    """
    Convert a list of Command (>= past_revision) to a list of EnvironmentCommand msgs.
    """

def msg_to_command(msg: EnvironmentCommand) -> tesseract_robotics.tesseract_environment._tesseract_environment.Command:
    """
    Convert a tesseract_msgs::msg::EnvironmentCommand to a tesseract_environment::Command.
    """

def msgs_to_commands(msgs: Sequence[EnvironmentCommand]) -> list[tesseract_robotics.tesseract_environment._tesseract_environment.Command]:
    """
    Convert a list of EnvironmentCommand msgs to a list of Command objects.
    """

def trajectory_to_msg(traj: tesseract_robotics.tesseract_common._tesseract_common.JointTrajectory) -> TesseractMsgsJointTrajectory:
    """
    Convert a tesseract_common::JointTrajectory to a tesseract_msgs::msg::JointTrajectory.
    """

def msg_to_trajectory(msg: TesseractMsgsJointTrajectory) -> tesseract_robotics.tesseract_common._tesseract_common.JointTrajectory:
    """
    Convert a tesseract_msgs::msg::JointTrajectory to a tesseract_common::JointTrajectory.
    """

def trajectory_to_legacy_msg(traj: tesseract_robotics.tesseract_common._tesseract_common.JointTrajectory, initial_state: tesseract_robotics.tesseract_state_solver._tesseract_state_solver.SceneState) -> TrajectoryMsgsJointTrajectory:
    """
    Convert a tesseract_common::JointTrajectory + initial SceneState to a trajectory_msgs::msg::JointTrajectory.
    """

def legacy_msg_to_trajectory(msg: TrajectoryMsgsJointTrajectory) -> tesseract_robotics.tesseract_common._tesseract_common.JointTrajectory:
    """
    Convert a trajectory_msgs::msg::JointTrajectory to a tesseract_common::JointTrajectory.
    """
