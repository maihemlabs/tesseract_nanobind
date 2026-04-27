/**
 * @file tesseract_ros2_rosutils_bindings.cpp
 * @brief nanobind bindings for the `tesseract_rosutils` package from
 *        tesseract_ros2 (ROS 2 flavor).
 *
 * Exposes a high-leverage subset of `tesseract_rosutils::utils.h`:
 *   - `Pose <-> Eigen::Isometry3d` (quaternion math)
 *   - `EnvironmentCommand <-> tesseract_environment::Command` (polymorphic
 *      dispatch + recursion through Link/Joint/Geometry/SceneGraph/etc.)
 *   - `tesseract_msgs::JointTrajectory <-> tesseract_common::JointTrajectory`
 *   - `trajectory_msgs::JointTrajectory <-> tesseract_common::JointTrajectory`
 *
 * Deliberately excluded (trivial in pure Python):
 *   - `JointState <-> unordered_map<string, double>`
 *   - `StringDoublePair[] <-> unordered_map<string, double>`
 *
 * Convention compliance: this TU follows the repo's stated convention
 * (`src/tesseract_nb.h:45-47`) of explicit `nb::class_<T>` bindings, with no
 * type casters. Bound msg types are NOT rclpy classes; downstream code that
 * publishes to ROS must field-copy at the boundary, the same way it already
 * does for `Eigen::Isometry3d <-> numpy`.
 */

#include "tesseract_nb.h"
#include <nanobind/stl/vector.h>
#include <nanobind/stl/string.h>

// tesseract core
#include <tesseract_common/joint_state.h>
#include <tesseract_environment/command.h>
#include <tesseract_environment/commands.h>
#include <tesseract_scene_graph/scene_state.h>

// rosutils
#include <tesseract_rosutils/utils.h>

// ROS msg types
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <std_msgs/msg/header.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <builtin_interfaces/msg/time.hpp>
#include <builtin_interfaces/msg/duration.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

// tesseract_msgs
#include <tesseract_msgs/msg/joint_calibration.hpp>
#include <tesseract_msgs/msg/joint_dynamics.hpp>
#include <tesseract_msgs/msg/joint_limits.hpp>
#include <tesseract_msgs/msg/joint_mimic.hpp>
#include <tesseract_msgs/msg/joint_safety.hpp>
#include <tesseract_msgs/msg/material.hpp>
#include <tesseract_msgs/msg/mesh.hpp>
#include <tesseract_msgs/msg/geometry.hpp>
#include <tesseract_msgs/msg/inertial.hpp>
#include <tesseract_msgs/msg/allowed_collision_entry.hpp>
#include <tesseract_msgs/msg/collision_margin_data.hpp>
#include <tesseract_msgs/msg/string_double_pair.hpp>
#include <tesseract_msgs/msg/joint_state.hpp>
#include <tesseract_msgs/msg/joint_trajectory.hpp>
#include <tesseract_msgs/msg/visual_geometry.hpp>
#include <tesseract_msgs/msg/collision_geometry.hpp>
#include <tesseract_msgs/msg/link.hpp>
#include <tesseract_msgs/msg/joint.hpp>
#include <tesseract_msgs/msg/scene_graph.hpp>
#include <tesseract_msgs/msg/environment_command.hpp>

namespace te = tesseract_environment;
namespace tc = tesseract_common;
namespace tru = tesseract_rosutils;
namespace gm = geometry_msgs::msg;
namespace sm = sensor_msgs::msg;
namespace trm = trajectory_msgs::msg;
namespace bim = builtin_interfaces::msg;
namespace stm = std_msgs::msg;
namespace tm_msg = tesseract_msgs::msg;

NB_MODULE(_tesseract_ros2_rosutils, m)
{
  // Inheritance / dependency module imports — these register types we will
  // reference (Command, JointTrajectory, Isometry3d, SceneState, etc.).
  nb::module_::import_("tesseract_robotics.tesseract_common._tesseract_common");
  nb::module_::import_("tesseract_robotics.tesseract_environment._tesseract_environment");
  nb::module_::import_("tesseract_robotics.tesseract_state_solver._tesseract_state_solver");

  // ============================================================
  // Layer 2 — std_msgs / builtin_interfaces
  // (bound first because Layer 1's PoseArray and Layer 3 types
  // reference Header / Time / Duration as fields.)
  // ============================================================

  nb::class_<bim::Time>(m, "Time")
      .def(nb::init<>())
      .def_rw("sec", &bim::Time::sec)
      .def_rw("nanosec", &bim::Time::nanosec);

  nb::class_<bim::Duration>(m, "Duration")
      .def(nb::init<>())
      .def_rw("sec", &bim::Duration::sec)
      .def_rw("nanosec", &bim::Duration::nanosec);

  nb::class_<stm::Header>(m, "Header")
      .def(nb::init<>())
      .def_rw("stamp", &stm::Header::stamp)
      .def_rw("frame_id", &stm::Header::frame_id);

  nb::class_<stm::ColorRGBA>(m, "ColorRGBA")
      .def(nb::init<>())
      .def_rw("r", &stm::ColorRGBA::r)
      .def_rw("g", &stm::ColorRGBA::g)
      .def_rw("b", &stm::ColorRGBA::b)
      .def_rw("a", &stm::ColorRGBA::a);

  // ============================================================
  // Layer 1 — primitive geometry msgs
  // ============================================================

  nb::class_<gm::Vector3>(m, "Vector3")
      .def(nb::init<>())
      .def_rw("x", &gm::Vector3::x)
      .def_rw("y", &gm::Vector3::y)
      .def_rw("z", &gm::Vector3::z);

  nb::class_<gm::Point>(m, "Point")
      .def(nb::init<>())
      .def_rw("x", &gm::Point::x)
      .def_rw("y", &gm::Point::y)
      .def_rw("z", &gm::Point::z);

  nb::class_<gm::Quaternion>(m, "Quaternion")
      .def(nb::init<>())
      .def_rw("x", &gm::Quaternion::x)
      .def_rw("y", &gm::Quaternion::y)
      .def_rw("z", &gm::Quaternion::z)
      .def_rw("w", &gm::Quaternion::w);

  nb::class_<gm::Pose>(m, "Pose")
      .def(nb::init<>())
      .def_rw("position", &gm::Pose::position)
      .def_rw("orientation", &gm::Pose::orientation);

  nb::class_<gm::PoseArray>(m, "PoseArray")
      .def(nb::init<>())
      .def_rw("header", &gm::PoseArray::header)
      .def_rw("poses", &gm::PoseArray::poses);

  // ============================================================
  // Layer 3 — sensor_msgs / trajectory_msgs
  // (sensor_msgs::JointState rebound as "SensorJointState" to avoid colliding
  // with tesseract_msgs::msg::JointState bound in Layer 4. Same reason for
  // trajectory_msgs::JointTrajectory -> "TrajectoryMsgsJointTrajectory".)
  // ============================================================

  nb::class_<sm::JointState>(m, "SensorJointState")
      .def(nb::init<>())
      .def_rw("header", &sm::JointState::header)
      .def_rw("name", &sm::JointState::name)
      .def_rw("position", &sm::JointState::position)
      .def_rw("velocity", &sm::JointState::velocity)
      .def_rw("effort", &sm::JointState::effort);

  nb::class_<trm::JointTrajectoryPoint>(m, "JointTrajectoryPoint")
      .def(nb::init<>())
      .def_rw("positions", &trm::JointTrajectoryPoint::positions)
      .def_rw("velocities", &trm::JointTrajectoryPoint::velocities)
      .def_rw("accelerations", &trm::JointTrajectoryPoint::accelerations)
      .def_rw("effort", &trm::JointTrajectoryPoint::effort)
      .def_rw("time_from_start", &trm::JointTrajectoryPoint::time_from_start);

  nb::class_<trm::JointTrajectory>(m, "TrajectoryMsgsJointTrajectory")
      .def(nb::init<>())
      .def_rw("header", &trm::JointTrajectory::header)
      .def_rw("joint_names", &trm::JointTrajectory::joint_names)
      .def_rw("points", &trm::JointTrajectory::points);

  // ============================================================
  // Layer 4 — tesseract_msgs leaf types
  // ============================================================

  nb::class_<tm_msg::JointCalibration>(m, "JointCalibration")
      .def(nb::init<>())
      .def_rw("reference_position", &tm_msg::JointCalibration::reference_position)
      .def_rw("rising", &tm_msg::JointCalibration::rising)
      .def_rw("falling", &tm_msg::JointCalibration::falling)
      .def_rw("empty", &tm_msg::JointCalibration::empty);

  nb::class_<tm_msg::JointDynamics>(m, "JointDynamics")
      .def(nb::init<>())
      .def_rw("damping", &tm_msg::JointDynamics::damping)
      .def_rw("friction", &tm_msg::JointDynamics::friction)
      .def_rw("empty", &tm_msg::JointDynamics::empty);

  nb::class_<tm_msg::JointLimits>(m, "JointLimits")
      .def(nb::init<>())
      .def_rw("lower", &tm_msg::JointLimits::lower)
      .def_rw("upper", &tm_msg::JointLimits::upper)
      .def_rw("effort", &tm_msg::JointLimits::effort)
      .def_rw("velocity", &tm_msg::JointLimits::velocity)
      .def_rw("acceleration", &tm_msg::JointLimits::acceleration)
      .def_rw("empty", &tm_msg::JointLimits::empty);

  nb::class_<tm_msg::JointMimic>(m, "JointMimic")
      .def(nb::init<>())
      .def_rw("offset", &tm_msg::JointMimic::offset)
      .def_rw("multiplier", &tm_msg::JointMimic::multiplier)
      .def_rw("joint_name", &tm_msg::JointMimic::joint_name)
      .def_rw("empty", &tm_msg::JointMimic::empty);

  nb::class_<tm_msg::JointSafety>(m, "JointSafety")
      .def(nb::init<>())
      .def_rw("soft_upper_limit", &tm_msg::JointSafety::soft_upper_limit)
      .def_rw("soft_lower_limit", &tm_msg::JointSafety::soft_lower_limit)
      .def_rw("k_position", &tm_msg::JointSafety::k_position)
      .def_rw("k_velocity", &tm_msg::JointSafety::k_velocity)
      .def_rw("empty", &tm_msg::JointSafety::empty);

  nb::class_<tm_msg::Material>(m, "Material")
      .def(nb::init<>())
      .def_rw("name", &tm_msg::Material::name)
      .def_rw("texture_filename", &tm_msg::Material::texture_filename)
      .def_rw("color", &tm_msg::Material::color)
      .def_rw("empty", &tm_msg::Material::empty);

  nb::class_<tm_msg::Mesh>(m, "Mesh")
      .def(nb::init<>())
      .def_rw("vertices", &tm_msg::Mesh::vertices)
      .def_rw("faces", &tm_msg::Mesh::faces)
      .def_rw("file_path", &tm_msg::Mesh::file_path)
      .def_rw("scale", &tm_msg::Mesh::scale);

  // Geometry: discriminated union by `type`. Skip `octomap` (octomap_msgs
  // dep — not bound) and `compound_mesh`/`compound_mesh_type` (vector<Mesh>
  // edge case — keep this PR contained to the common shapes).
  auto geometry_cls = nb::class_<tm_msg::Geometry>(m, "Geometry")
      .def(nb::init<>())
      .def_rw("type", &tm_msg::Geometry::type)
      .def_rw("uuid", &tm_msg::Geometry::uuid)
      .def_rw("sphere_radius", &tm_msg::Geometry::sphere_radius)
      .def_rw("cylinder_dimensions", &tm_msg::Geometry::cylinder_dimensions)
      .def_rw("capsule_dimensions", &tm_msg::Geometry::capsule_dimensions)
      .def_rw("cone_dimensions", &tm_msg::Geometry::cone_dimensions)
      .def_rw("box_dimensions", &tm_msg::Geometry::box_dimensions)
      .def_rw("plane_coeff", &tm_msg::Geometry::plane_coeff)
      .def_rw("mesh", &tm_msg::Geometry::mesh);
  geometry_cls.attr("SPHERE") = (uint8_t)tm_msg::Geometry::SPHERE;
  geometry_cls.attr("CYLINDER") = (uint8_t)tm_msg::Geometry::CYLINDER;
  geometry_cls.attr("CONE") = (uint8_t)tm_msg::Geometry::CONE;
  geometry_cls.attr("BOX") = (uint8_t)tm_msg::Geometry::BOX;
  geometry_cls.attr("PLANE") = (uint8_t)tm_msg::Geometry::PLANE;
  geometry_cls.attr("MESH") = (uint8_t)tm_msg::Geometry::MESH;
  geometry_cls.attr("CONVEX_MESH") = (uint8_t)tm_msg::Geometry::CONVEX_MESH;
  geometry_cls.attr("SDF_MESH") = (uint8_t)tm_msg::Geometry::SDF_MESH;
  geometry_cls.attr("OCTREE") = (uint8_t)tm_msg::Geometry::OCTREE;
  geometry_cls.attr("CAPSULE") = (uint8_t)tm_msg::Geometry::CAPSULE;
  geometry_cls.attr("POLYGON_MESH") = (uint8_t)tm_msg::Geometry::POLYGON_MESH;
  geometry_cls.attr("COMPOUND_MESH") = (uint8_t)tm_msg::Geometry::COMPOUND_MESH;

  nb::class_<tm_msg::Inertial>(m, "Inertial")
      .def(nb::init<>())
      .def_rw("origin", &tm_msg::Inertial::origin)
      .def_rw("mass", &tm_msg::Inertial::mass)
      .def_rw("ixx", &tm_msg::Inertial::ixx)
      .def_rw("ixy", &tm_msg::Inertial::ixy)
      .def_rw("ixz", &tm_msg::Inertial::ixz)
      .def_rw("iyy", &tm_msg::Inertial::iyy)
      .def_rw("iyz", &tm_msg::Inertial::iyz)
      .def_rw("izz", &tm_msg::Inertial::izz)
      .def_rw("empty", &tm_msg::Inertial::empty);

  nb::class_<tm_msg::AllowedCollisionEntry>(m, "AllowedCollisionEntry")
      .def(nb::init<>())
      .def_rw("link_1", &tm_msg::AllowedCollisionEntry::link_1)
      .def_rw("link_2", &tm_msg::AllowedCollisionEntry::link_2)
      .def_rw("reason", &tm_msg::AllowedCollisionEntry::reason);

  // CollisionMarginData: only `default_margin` is bound. `margin_pairs` uses
  // tesseract_msgs::ContactMarginPair which is out of scope for this PR.
  nb::class_<tm_msg::CollisionMarginData>(m, "CollisionMarginData")
      .def(nb::init<>())
      .def_rw("default_margin", &tm_msg::CollisionMarginData::default_margin);

  nb::class_<tm_msg::StringDoublePair>(m, "StringDoublePair")
      .def(nb::init<>())
      .def_rw("first", &tm_msg::StringDoublePair::first)
      .def_rw("second", &tm_msg::StringDoublePair::second);

  // tesseract_msgs::JointState (rebound to disambiguate from the
  // tesseract_common::JointState already bound in tesseract_common).
  nb::class_<tm_msg::JointState>(m, "TesseractMsgsJointState")
      .def(nb::init<>())
      .def_rw("joint_names", &tm_msg::JointState::joint_names)
      .def_rw("position", &tm_msg::JointState::position)
      .def_rw("velocity", &tm_msg::JointState::velocity)
      .def_rw("acceleration", &tm_msg::JointState::acceleration)
      .def_rw("effort", &tm_msg::JointState::effort)
      .def_rw("time_from_start", &tm_msg::JointState::time_from_start);

  // tesseract_msgs::JointTrajectory. Note: uuid here is std::string (rosidl
  // serializes boost::uuids::uuid as a string at the wire level), distinct
  // from tesseract_common::JointTrajectory.uuid which is exposed as bytes.
  nb::class_<tm_msg::JointTrajectory>(m, "TesseractMsgsJointTrajectory")
      .def(nb::init<>())
      .def_rw("states", &tm_msg::JointTrajectory::states)
      .def_rw("description", &tm_msg::JointTrajectory::description)
      .def_rw("uuid", &tm_msg::JointTrajectory::uuid);

  // ============================================================
  // Layer 5 — composite tesseract_msgs (Link, Joint, SceneGraph)
  // ============================================================

  nb::class_<tm_msg::VisualGeometry>(m, "VisualGeometry")
      .def(nb::init<>())
      .def_rw("name", &tm_msg::VisualGeometry::name)
      .def_rw("origin", &tm_msg::VisualGeometry::origin)
      .def_rw("geometry", &tm_msg::VisualGeometry::geometry)
      .def_rw("material", &tm_msg::VisualGeometry::material);

  nb::class_<tm_msg::CollisionGeometry>(m, "CollisionGeometry")
      .def(nb::init<>())
      .def_rw("name", &tm_msg::CollisionGeometry::name)
      .def_rw("origin", &tm_msg::CollisionGeometry::origin)
      .def_rw("geometry", &tm_msg::CollisionGeometry::geometry)
      .def_rw("material", &tm_msg::CollisionGeometry::material);

  nb::class_<tm_msg::Link>(m, "Link")
      .def(nb::init<>())
      .def_rw("name", &tm_msg::Link::name)
      .def_rw("inertial", &tm_msg::Link::inertial)
      .def_rw("visual", &tm_msg::Link::visual)
      .def_rw("collision", &tm_msg::Link::collision);

  auto joint_cls = nb::class_<tm_msg::Joint>(m, "Joint")
      .def(nb::init<>())
      .def_rw("name", &tm_msg::Joint::name)
      .def_rw("type", &tm_msg::Joint::type)
      .def_rw("axis", &tm_msg::Joint::axis)
      .def_rw("child_link_name", &tm_msg::Joint::child_link_name)
      .def_rw("parent_link_name", &tm_msg::Joint::parent_link_name)
      .def_rw("parent_to_joint_origin_transform",
              &tm_msg::Joint::parent_to_joint_origin_transform)
      .def_rw("limits", &tm_msg::Joint::limits)
      .def_rw("dynamics", &tm_msg::Joint::dynamics)
      .def_rw("safety", &tm_msg::Joint::safety)
      .def_rw("calibration", &tm_msg::Joint::calibration)
      .def_rw("mimic", &tm_msg::Joint::mimic);
  joint_cls.attr("UNKNOWN") = (uint8_t)tm_msg::Joint::UNKNOWN;
  joint_cls.attr("REVOLUTE") = (uint8_t)tm_msg::Joint::REVOLUTE;
  joint_cls.attr("CONTINUOUS") = (uint8_t)tm_msg::Joint::CONTINUOUS;
  joint_cls.attr("PRISMATIC") = (uint8_t)tm_msg::Joint::PRISMATIC;
  joint_cls.attr("FLOATING") = (uint8_t)tm_msg::Joint::FLOATING;
  joint_cls.attr("PLANAR") = (uint8_t)tm_msg::Joint::PLANAR;
  joint_cls.attr("FIXED") = (uint8_t)tm_msg::Joint::FIXED;

  nb::class_<tm_msg::SceneGraph>(m, "SceneGraph")
      .def(nb::init<>())
      .def_rw("id", &tm_msg::SceneGraph::id)
      .def_rw("root", &tm_msg::SceneGraph::root)
      .def_rw("links", &tm_msg::SceneGraph::links)
      .def_rw("joints", &tm_msg::SceneGraph::joints)
      .def_rw("invisible_links", &tm_msg::SceneGraph::invisible_links)
      .def_rw("disabled_collision_links", &tm_msg::SceneGraph::disabled_collision_links)
      .def_rw("acm", &tm_msg::SceneGraph::acm);

  // ============================================================
  // Layer 6 — EnvironmentCommand
  // ============================================================
  // Discriminated union with 23 command types. We `def_rw` only the fields
  // whose types are bound in this module. The remaining fields (kinematics
  // info, contact-manager plugin info, joint-limit pair vectors, etc.)
  // remain default-initialized in C++ but are not Python-visible. The
  // associated Command subclasses (ChangeJointPositionLimitsCommand,
  // AddKinematicsInformationCommand, ChangeCollisionMarginsCommand,
  // AddContactManagersPluginInfoCommand) therefore won't roundtrip cleanly
  // through these bindings — left for a follow-up PR.
  auto envcmd_cls = nb::class_<tm_msg::EnvironmentCommand>(m, "EnvironmentCommand")
      .def(nb::init<>())
      .def_rw("command", &tm_msg::EnvironmentCommand::command)
      .def_rw("add_link", &tm_msg::EnvironmentCommand::add_link)
      .def_rw("add_joint", &tm_msg::EnvironmentCommand::add_joint)
      .def_rw("add_replace_allowed", &tm_msg::EnvironmentCommand::add_replace_allowed)
      .def_rw("move_link_joint", &tm_msg::EnvironmentCommand::move_link_joint)
      .def_rw("move_joint_name", &tm_msg::EnvironmentCommand::move_joint_name)
      .def_rw("move_joint_parent_link", &tm_msg::EnvironmentCommand::move_joint_parent_link)
      .def_rw("remove_link", &tm_msg::EnvironmentCommand::remove_link)
      .def_rw("remove_joint", &tm_msg::EnvironmentCommand::remove_joint)
      .def_rw("replace_joint", &tm_msg::EnvironmentCommand::replace_joint)
      .def_rw("change_link_origin_name", &tm_msg::EnvironmentCommand::change_link_origin_name)
      .def_rw("change_link_origin_pose", &tm_msg::EnvironmentCommand::change_link_origin_pose)
      .def_rw("change_joint_origin_name", &tm_msg::EnvironmentCommand::change_joint_origin_name)
      .def_rw("change_joint_origin_pose", &tm_msg::EnvironmentCommand::change_joint_origin_pose)
      .def_rw("change_link_collision_enabled_name",
              &tm_msg::EnvironmentCommand::change_link_collision_enabled_name)
      .def_rw("change_link_collision_enabled_value",
              &tm_msg::EnvironmentCommand::change_link_collision_enabled_value)
      .def_rw("change_link_visibility_name",
              &tm_msg::EnvironmentCommand::change_link_visibility_name)
      .def_rw("change_link_visibility_value",
              &tm_msg::EnvironmentCommand::change_link_visibility_value)
      .def_rw("modify_allowed_collisions_type",
              &tm_msg::EnvironmentCommand::modify_allowed_collisions_type)
      .def_rw("modify_allowed_collisions",
              &tm_msg::EnvironmentCommand::modify_allowed_collisions)
      .def_rw("remove_allowed_collision_link",
              &tm_msg::EnvironmentCommand::remove_allowed_collision_link)
      .def_rw("joint_state", &tm_msg::EnvironmentCommand::joint_state)
      .def_rw("scene_graph", &tm_msg::EnvironmentCommand::scene_graph)
      .def_rw("scene_graph_joint", &tm_msg::EnvironmentCommand::scene_graph_joint)
      .def_rw("scene_graph_prefix", &tm_msg::EnvironmentCommand::scene_graph_prefix)
      .def_rw("has_collision_default_margin",
              &tm_msg::EnvironmentCommand::has_collision_default_margin)
      .def_rw("collision_default_margin",
              &tm_msg::EnvironmentCommand::collision_default_margin)
      .def_rw("set_active_discrete_contact_manager",
              &tm_msg::EnvironmentCommand::set_active_discrete_contact_manager)
      .def_rw("set_active_continuous_contact_manager",
              &tm_msg::EnvironmentCommand::set_active_continuous_contact_manager)
      .def_rw("add_trajectory_link_name",
              &tm_msg::EnvironmentCommand::add_trajectory_link_name)
      .def_rw("add_trajectory_link_parent_name",
              &tm_msg::EnvironmentCommand::add_trajectory_link_parent_name)
      .def_rw("add_trajectory_link_traj",
              &tm_msg::EnvironmentCommand::add_trajectory_link_traj)
      .def_rw("add_trajectory_link_replace_allowed",
              &tm_msg::EnvironmentCommand::add_trajectory_link_replace_allowed);

  // Command-type discriminator constants. Set as Python class attributes so
  // user code can write `EnvironmentCommand.ADD_LINK` etc. Using `.attr` on
  // the bound class (rather than `def_ro_static`) sidesteps a potential ODR
  // issue with rosidl-generated `static constexpr` members on some toolchains.
  envcmd_cls.attr("ADD_LINK") = (uint8_t)tm_msg::EnvironmentCommand::ADD_LINK;
  envcmd_cls.attr("MOVE_LINK") = (uint8_t)tm_msg::EnvironmentCommand::MOVE_LINK;
  envcmd_cls.attr("MOVE_JOINT") = (uint8_t)tm_msg::EnvironmentCommand::MOVE_JOINT;
  envcmd_cls.attr("REMOVE_LINK") = (uint8_t)tm_msg::EnvironmentCommand::REMOVE_LINK;
  envcmd_cls.attr("REMOVE_JOINT") = (uint8_t)tm_msg::EnvironmentCommand::REMOVE_JOINT;
  envcmd_cls.attr("CHANGE_LINK_ORIGIN") = (uint8_t)tm_msg::EnvironmentCommand::CHANGE_LINK_ORIGIN;
  envcmd_cls.attr("CHANGE_JOINT_ORIGIN") = (uint8_t)tm_msg::EnvironmentCommand::CHANGE_JOINT_ORIGIN;
  envcmd_cls.attr("CHANGE_LINK_COLLISION_ENABLED") =
      (uint8_t)tm_msg::EnvironmentCommand::CHANGE_LINK_COLLISION_ENABLED;
  envcmd_cls.attr("CHANGE_LINK_VISIBILITY") =
      (uint8_t)tm_msg::EnvironmentCommand::CHANGE_LINK_VISIBILITY;
  envcmd_cls.attr("MODIFY_ALLOWED_COLLISIONS") =
      (uint8_t)tm_msg::EnvironmentCommand::MODIFY_ALLOWED_COLLISIONS;
  envcmd_cls.attr("REMOVE_ALLOWED_COLLISION_LINK") =
      (uint8_t)tm_msg::EnvironmentCommand::REMOVE_ALLOWED_COLLISION_LINK;
  envcmd_cls.attr("UPDATE_JOINT_STATE") = (uint8_t)tm_msg::EnvironmentCommand::UPDATE_JOINT_STATE;
  envcmd_cls.attr("ADD_SCENE_GRAPH") = (uint8_t)tm_msg::EnvironmentCommand::ADD_SCENE_GRAPH;
  envcmd_cls.attr("CHANGE_JOINT_POSITION_LIMITS") =
      (uint8_t)tm_msg::EnvironmentCommand::CHANGE_JOINT_POSITION_LIMITS;
  envcmd_cls.attr("CHANGE_JOINT_VELOCITY_LIMITS") =
      (uint8_t)tm_msg::EnvironmentCommand::CHANGE_JOINT_VELOCITY_LIMITS;
  envcmd_cls.attr("CHANGE_JOINT_ACCELERATION_LIMITS") =
      (uint8_t)tm_msg::EnvironmentCommand::CHANGE_JOINT_ACCELERATION_LIMITS;
  envcmd_cls.attr("ADD_KINEMATICS_INFORMATION") =
      (uint8_t)tm_msg::EnvironmentCommand::ADD_KINEMATICS_INFORMATION;
  envcmd_cls.attr("REPLACE_JOINT") = (uint8_t)tm_msg::EnvironmentCommand::REPLACE_JOINT;
  envcmd_cls.attr("CHANGE_COLLISION_MARGINS") =
      (uint8_t)tm_msg::EnvironmentCommand::CHANGE_COLLISION_MARGINS;
  envcmd_cls.attr("ADD_CONTACT_MANAGERS_PLUGIN_INFO") =
      (uint8_t)tm_msg::EnvironmentCommand::ADD_CONTACT_MANAGERS_PLUGIN_INFO;
  envcmd_cls.attr("SET_ACTIVE_DISCRETE_CONTACT_MANAGER") =
      (uint8_t)tm_msg::EnvironmentCommand::SET_ACTIVE_DISCRETE_CONTACT_MANAGER;
  envcmd_cls.attr("SET_ACTIVE_CONTINUOUS_CONTACT_MANAGER") =
      (uint8_t)tm_msg::EnvironmentCommand::SET_ACTIVE_CONTINUOUS_CONTACT_MANAGER;
  envcmd_cls.attr("ADD_TRAJECTORY_LINK") = (uint8_t)tm_msg::EnvironmentCommand::ADD_TRAJECTORY_LINK;

  // ============================================================
  // Layer 7 — rosutils conversion functions
  // ============================================================
  // Each lambda wraps a `tesseract_rosutils` `toMsg`/`fromMsg` call. The
  // C++ functions return `bool` for failure; we re-throw as RuntimeError
  // so Python callers don't have to inspect a return value.

  m.def("iso_to_pose",
        [](Eigen::Isometry3d const& iso) {
          gm::Pose msg;
          if (!tru::toMsg(msg, iso))
            throw std::runtime_error("toMsg(Pose, Isometry3d) failed");
          return msg;
        },
        "iso"_a,
        "Convert an Eigen::Isometry3d to a geometry_msgs::msg::Pose.");

  m.def("pose_to_iso",
        [](gm::Pose const& msg) {
          Eigen::Isometry3d iso;
          if (!tru::fromMsg(iso, msg))
            throw std::runtime_error("fromMsg(Isometry3d, Pose) failed");
          return iso;
        },
        "pose"_a,
        "Convert a geometry_msgs::msg::Pose to an Eigen::Isometry3d.");

  m.def("isos_to_pose_array",
        [](std::vector<Eigen::Isometry3d> const& transforms) {
          // Copy into a tc::VectorIsometry3d (aligned allocator) for the call.
          tc::VectorIsometry3d aligned(transforms.begin(), transforms.end());
          gm::PoseArray msg;
          if (!tru::toMsg(msg, aligned))
            throw std::runtime_error("toMsg(PoseArray, VectorIsometry3d) failed");
          return msg;
        },
        "transforms"_a,
        "Convert a list of Eigen::Isometry3d to a geometry_msgs::msg::PoseArray.");

  m.def("command_to_msg",
        [](te::Command const& cmd) {
          tm_msg::EnvironmentCommand msg;
          if (!tru::toMsg(msg, cmd))
            throw std::runtime_error("toMsg(EnvironmentCommand, Command) failed");
          return msg;
        },
        "command"_a,
        "Convert a tesseract_environment::Command to a tesseract_msgs::msg::EnvironmentCommand.");

  m.def("commands_to_msg",
        [](std::vector<std::shared_ptr<const te::Command>> const& cmds,
           unsigned long past_revision) {
          std::vector<tm_msg::EnvironmentCommand> msgs;
          if (!tru::toMsg(msgs, cmds, past_revision))
            throw std::runtime_error("toMsg(vector<EnvironmentCommand>, vector<Command>) failed");
          return msgs;
        },
        "commands"_a, "past_revision"_a = 0UL,
        "Convert a list of Command (>= past_revision) to a list of EnvironmentCommand msgs.");

  m.def("msg_to_command",
        [](tm_msg::EnvironmentCommand const& msg) {
          auto cmd = tru::fromMsg(msg);
          if (!cmd)
            throw std::runtime_error("fromMsg(Command, EnvironmentCommand) returned nullptr");
          return cmd;
        },
        "msg"_a,
        "Convert a tesseract_msgs::msg::EnvironmentCommand to a tesseract_environment::Command.");

  m.def("msgs_to_commands",
        [](std::vector<tm_msg::EnvironmentCommand> const& msgs) {
          // C++ returns vector<shared_ptr<const Command>>; expose as
          // vector<shared_ptr<Command>> on the Python side so users can pass
          // the result back into other bindings without const-cast wrappers.
          auto cmds_const = tru::fromMsg(msgs);
          std::vector<std::shared_ptr<te::Command>> cmds;
          cmds.reserve(cmds_const.size());
          for (auto const& c : cmds_const)
            cmds.push_back(std::const_pointer_cast<te::Command>(c));
          return cmds;
        },
        "msgs"_a,
        "Convert a list of EnvironmentCommand msgs to a list of Command objects.");

  m.def("trajectory_to_msg",
        [](tc::JointTrajectory const& traj) {
          tm_msg::JointTrajectory msg;
          tru::toMsg(msg, traj);
          return msg;
        },
        "traj"_a,
        "Convert a tesseract_common::JointTrajectory to a tesseract_msgs::msg::JointTrajectory.");

  m.def("msg_to_trajectory",
        [](tm_msg::JointTrajectory const& msg) {
          return tru::fromMsg(msg);
        },
        "msg"_a,
        "Convert a tesseract_msgs::msg::JointTrajectory to a tesseract_common::JointTrajectory.");

  m.def("trajectory_to_legacy_msg",
        [](tc::JointTrajectory const& traj,
           tesseract_scene_graph::SceneState const& initial_state) {
          return tru::toMsg(traj, initial_state);
        },
        "traj"_a, "initial_state"_a,
        "Convert a tesseract_common::JointTrajectory + initial SceneState to a "
        "trajectory_msgs::msg::JointTrajectory.");

  m.def("legacy_msg_to_trajectory",
        [](trm::JointTrajectory const& msg) {
          return tru::fromMsg(msg);
        },
        "msg"_a,
        "Convert a trajectory_msgs::msg::JointTrajectory to a tesseract_common::JointTrajectory.");
}
