/**
 * @file tesseract_ros2_monitoring_bindings.cpp
 * @brief nanobind bindings for the `tesseract_monitoring` package from
 *        tesseract_ros2 (ROS 2 flavor).
 *
 * Exposes:
 *   - `ROSContext`     : RAII wrapper owning rclcpp::init/shutdown and a
 *                        user-facing rclcpp::Node with a background executor.
 *   - `MonitoredEnvironmentMode` (enum, from tesseract core)
 *   - `EnvironmentMonitor`       (abstract base, from tesseract core)
 *   - `ROSEnvironmentMonitor`    (ROS 2 concrete subclass)
 *   - `CurrentStateMonitor`      (read-only view, subset of public API)
 *
 * The `ROSContext` helper is modeled on moveit_py's node-ownership lifecycle
 * (init -> node -> executor -> spin thread -> shutdown). Written from
 * scratch; no upstream code is vendored.
 */

#include "tesseract_nb.h"
#include <nanobind/stl/chrono.h>
#include <nanobind/stl/unordered_map.h>

#include <chrono>
#include <thread>

// rclcpp
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>

// tesseract core base (flat layout — matches the rest of the nanobind repo
// against tesseract 0.34.x; not the post-consolidation `tesseract/...` style).
#include <tesseract_environment/environment.h>
#include <tesseract_environment/environment_monitor.h>

// tesseract_ros2 monitoring
#include <tesseract_monitoring/environment_monitor.h>
#include <tesseract_monitoring/current_state_monitor.h>
#include <tesseract_monitoring/constants.h>

namespace te = tesseract_environment;
// Alias is `tmon`, not `tm` — `struct tm` from <ctime> (pulled in
// transitively via <chrono>) shares the same name lookup space as namespace
// aliases in C++, so `tm::Foo` mis-parses as scope-into-struct.
namespace tmon = tesseract_monitoring;

namespace
{
/**
 * @brief RAII wrapper owning `rclcpp::init`/`shutdown` plus one user-facing
 *        `rclcpp::Node` spun on a background executor thread.
 *
 * The monitor itself creates a *separate* internal node + executor for its
 * own business logic; this context only owns the node the user would have
 * hand-crafted in C++.
 */
class ROSContext
{
public:
  ROSContext(std::string node_name,
             std::vector<std::string> args,
             bool install_signal_handlers)
  {
    if (!rclcpp::ok())
    {
      std::vector<const char*> argv;
      argv.reserve(args.size());
      for (const auto& a : args)
        argv.push_back(a.c_str());

      rclcpp::InitOptions init_opts;
      rclcpp::init(static_cast<int>(argv.size()),
                   argv.empty() ? nullptr : argv.data(),
                   init_opts,
                   install_signal_handlers
                       ? rclcpp::SignalHandlerOptions::All
                       : rclcpp::SignalHandlerOptions::None);
      owns_init_ = true;
    }

    rclcpp::NodeOptions node_opts;
    node_opts.automatically_declare_parameters_from_overrides(true);
    node_ = std::make_shared<rclcpp::Node>(std::move(node_name), node_opts);

    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(node_);
    executor_thread_ = std::thread([this]() { executor_->spin(); });
  }

  ~ROSContext() { shutdown(); }

  ROSContext(const ROSContext&) = delete;
  ROSContext& operator=(const ROSContext&) = delete;
  ROSContext(ROSContext&&) = delete;
  ROSContext& operator=(ROSContext&&) = delete;

  rclcpp::Node::SharedPtr node() const { return node_; }
  std::string nodeName() const { return node_ ? std::string(node_->get_name()) : std::string{}; }

  void shutdown()
  {
    if (executor_)
    {
      executor_->cancel();
      if (executor_thread_.joinable())
        executor_thread_.join();
      if (node_)
        executor_->remove_node(node_);
      executor_.reset();
    }
    node_.reset();
    if (owns_init_ && rclcpp::ok())
      rclcpp::shutdown();
    owns_init_ = false;
  }

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_;
  std::thread executor_thread_;
  bool owns_init_{ false };
};
}  // namespace

NB_MODULE(_tesseract_ros2_monitoring, m)
{
  // Base `EnvironmentMonitor` uses `tesseract::environment::Environment` by
  // shared_ptr; load the environment module first so the holder type is
  // registered.
  nb::module_::import_("tesseract_robotics.tesseract_environment._tesseract_environment");

  m.doc() = "Python bindings for tesseract_monitoring (tesseract_ros2, ROS 2 Jazzy+)";

  // ---- ROSContext --------------------------------------------------------
  nb::class_<ROSContext>(m, "ROSContext",
      "Owns rclcpp::init/shutdown lifetime and one user-facing rclcpp::Node. "
      "A background SingleThreadedExecutor spins the node so parameter "
      "services and other rclcpp callbacks fire. Mirrors the typical "
      "environment_monitor_node.cpp construction pattern from tesseract_ros2.")
      .def(nb::init<std::string, std::vector<std::string>, bool>(),
           "node_name"_a,
           "args"_a = std::vector<std::string>{},
           "install_signal_handlers"_a = false,
           "Initialise rclcpp (if not already up), create a node named "
           "`node_name`, and start a background executor thread.")
      .def("node_name", &ROSContext::nodeName,
           "Return the fully-qualified name of the owned rclcpp::Node.")
      .def("shutdown", &ROSContext::shutdown,
           "Stop the executor, join the spin thread, drop the node, and "
           "(if this context owned rclcpp::init) call rclcpp::shutdown. "
           "Idempotent.");

  // ---- MonitoredEnvironmentMode enum ------------------------------------
  nb::enum_<te::MonitoredEnvironmentMode>(m, "MonitoredEnvironmentMode")
      .value("DEFAULT", te::MonitoredEnvironmentMode::DEFAULT)
      .value("SYNCHRONIZED", te::MonitoredEnvironmentMode::SYNCHRONIZED)
      .export_values();

  // ---- Abstract base EnvironmentMonitor ---------------------------------
  // `environment()` / `getEnvironment()` have const + non-const overloads.
  // `nb::overload_cast<>` is ambiguous for no-arg overloads; use lambdas to
  // disambiguate and expose only the shared_ptr variant to Python (simpler
  // for refcounted ownership across the binding boundary).
  nb::class_<te::EnvironmentMonitor>(m, "EnvironmentMonitor",
      "Abstract base class for tesseract environment monitors. Concrete "
      "subclasses (e.g. ROSEnvironmentMonitor) provide the transport layer.")
      .def("getNamespace",
           [](const te::EnvironmentMonitor& self) -> std::string {
             return self.getNamespace();
           },
           "Unique namespace identifying this monitor instance.")
      .def("getEnvironment",
           [](te::EnvironmentMonitor& self) -> std::shared_ptr<te::Environment> {
             return self.getEnvironment();
           },
           "Return the monitored tesseract::environment::Environment.")
      .def("waitForConnection", &te::EnvironmentMonitor::waitForConnection,
           "duration"_a = std::chrono::duration<double>(0),
           "Block until the environment is connected/initialised. "
           "duration=0 waits indefinitely.")
      .def("stopPublishingEnvironment", &te::EnvironmentMonitor::stopPublishingEnvironment)
      .def("setEnvironmentPublishingFrequency",
           &te::EnvironmentMonitor::setEnvironmentPublishingFrequency, "hz"_a)
      .def("getEnvironmentPublishingFrequency",
           &te::EnvironmentMonitor::getEnvironmentPublishingFrequency)
      .def("startStateMonitor", &te::EnvironmentMonitor::startStateMonitor,
           "joint_states_topic"_a, "publish_tf"_a = true)
      .def("stopStateMonitor", &te::EnvironmentMonitor::stopStateMonitor)
      .def("setStateUpdateFrequency",
           &te::EnvironmentMonitor::setStateUpdateFrequency, "hz"_a = 10.0)
      .def("getStateUpdateFrequency",
           &te::EnvironmentMonitor::getStateUpdateFrequency)
      .def("updateEnvironmentWithCurrentState",
           &te::EnvironmentMonitor::updateEnvironmentWithCurrentState)
      .def("startMonitoringEnvironment",
           &te::EnvironmentMonitor::startMonitoringEnvironment,
           "monitored_namespace"_a,
           "mode"_a = te::MonitoredEnvironmentMode::DEFAULT)
      .def("stopMonitoringEnvironment",
           &te::EnvironmentMonitor::stopMonitoringEnvironment)
      .def("waitForCurrentState", &te::EnvironmentMonitor::waitForCurrentState,
           "duration"_a = std::chrono::duration<double>(1.0))
      .def("shutdown", &te::EnvironmentMonitor::shutdown);

  // ---- ROSEnvironmentMonitor --------------------------------------------
  // keep_alive<nurse=1 (self), patient=2 (ctx)>: the ROSContext must outlive
  // the monitor. The monitor internally spawns a MultiThreadedExecutor on a
  // background thread; if rclcpp::shutdown fires (via ~ROSContext) while the
  // executor is still spinning, Jazzy raises RCLError or hangs.
  nb::class_<tmon::ROSEnvironmentMonitor, te::EnvironmentMonitor>(m, "ROSEnvironmentMonitor",
      "Monitors and optionally publishes a tesseract::environment::Environment "
      "over ROS 2 topics/services. The URDF-parameter constructor loads the "
      "environment from the `robot_description` / `robot_description_semantic` "
      "parameters on the context's node; the environment-passing constructor "
      "adopts a pre-built Environment.")
      .def("__init__",
           [](tmon::ROSEnvironmentMonitor* self,
              ROSContext& ctx,
              std::string robot_description,
              std::string monitor_namespace) {
             new (self) tmon::ROSEnvironmentMonitor(ctx.node(),
                                                   std::move(robot_description),
                                                   std::move(monitor_namespace));
           },
           "context"_a, "robot_description"_a, "monitor_namespace"_a,
           nb::keep_alive<1, 2>(),
           "Construct from a ROS-parameter URDF/SRDF. `robot_description` is "
           "the *parameter name* (not the URDF text); the parameter must be "
           "set on `context`'s node before this runs.")
      .def("__init__",
           [](tmon::ROSEnvironmentMonitor* self,
              ROSContext& ctx,
              std::shared_ptr<te::Environment> env,
              std::string monitor_namespace) {
             new (self) tmon::ROSEnvironmentMonitor(ctx.node(),
                                                   std::move(env),
                                                   std::move(monitor_namespace));
           },
           "context"_a, "environment"_a, "monitor_namespace"_a,
           nb::keep_alive<1, 2>(),
           "Adopt a pre-built Environment (recommended for Python scripting: "
           "build the Environment via tesseract_urdf/tesseract_srdf, then hand "
           "it to the monitor).")
      .def("getURDFDescription", &tmon::ROSEnvironmentMonitor::getURDFDescription,
           "Return the ROS parameter name that holds the URDF (empty when "
           "constructed from a pre-built Environment).")
      // Two-argument startPublishingEnvironment (publish_tf). The base class
      // has a no-arg pure-virtual overload; bind the ROS subclass's explicit
      // `(bool)` overload here.
      .def("startPublishingEnvironment",
           nb::overload_cast<bool>(&tmon::ROSEnvironmentMonitor::startPublishingEnvironment),
           "publish_tf"_a,
           "Begin publishing the environment on /<namespace>/tesseract_published_environment. "
           "When publish_tf=True, also broadcast TFs for each joint.")
      .def("getStateMonitor",
           [](tmon::ROSEnvironmentMonitor& self) -> tmon::CurrentStateMonitor& {
             return self.getStateMonitor();
           },
           nb::rv_policy::reference_internal,
           "Read-only view of the internal joint-state monitor.");

  // ---- CurrentStateMonitor (read-only subset) ---------------------------
  // Deliberately excluded in this PR:
  //   - addUpdateCallback / clearUpdateCallbacks : callback signature uses
  //     sensor_msgs::msg::JointState::ConstSharedPtr (ROS message type)
  //   - getCurrentStateTime / getMonitorStartTime / getCurrentStateAndTime :
  //     return rclcpp::Time (no wrapper yet)
  //   - haveCompleteState(age, ...)               : rclcpp::Duration overloads
  // Python can poll the state via getCurrentStateValues() / getCurrentState().
  nb::class_<tmon::CurrentStateMonitor>(m, "CurrentStateMonitor",
      "Monitors a joint_states topic and maintains the current robot state.")
      .def("isActive", &tmon::CurrentStateMonitor::isActive)
      .def("getMonitoredTopic", &tmon::CurrentStateMonitor::getMonitoredTopic)
      .def("haveCompleteState",
           nb::overload_cast<>(&tmon::CurrentStateMonitor::haveCompleteState, nb::const_),
           "True iff every DOF in the kinematic model has been observed "
           "at least once.")
      .def("getCurrentState", &tmon::CurrentStateMonitor::getCurrentState)
      .def("getCurrentStateValues", &tmon::CurrentStateMonitor::getCurrentStateValues,
           "Map of joint name -> joint position. Only observed joints "
           "appear; check haveCompleteState() first if completeness matters.")
      .def("getBoundsError", &tmon::CurrentStateMonitor::getBoundsError)
      .def("setBoundsError", &tmon::CurrentStateMonitor::setBoundsError, "error"_a)
      .def("enableCopyDynamics", &tmon::CurrentStateMonitor::enableCopyDynamics, "enabled"_a)
      .def("waitForCompleteState",
           nb::overload_cast<double>(&tmon::CurrentStateMonitor::waitForCompleteState, nb::const_),
           "wait_time"_a,
           "Block up to wait_time seconds until every DOF has been observed.");

  // ---- constants --------------------------------------------------------
  m.attr("DEFAULT_JOINT_STATES_TOPIC")                     = tmon::DEFAULT_JOINT_STATES_TOPIC;
  m.attr("DEFAULT_GET_ENVIRONMENT_CHANGES_SERVICE")        = tmon::DEFAULT_GET_ENVIRONMENT_CHANGES_SERVICE;
  m.attr("DEFAULT_GET_ENVIRONMENT_INFORMATION_SERVICE")    = tmon::DEFAULT_GET_ENVIRONMENT_INFORMATION_SERVICE;
  m.attr("DEFAULT_MODIFY_ENVIRONMENT_SERVICE")             = tmon::DEFAULT_MODIFY_ENVIRONMENT_SERVICE;
  m.attr("DEFAULT_SAVE_SCENE_GRAPH_SERVICE")               = tmon::DEFAULT_SAVE_SCENE_GRAPH_SERVICE;
  m.attr("DEFAULT_PUBLISH_ENVIRONMENT_TOPIC")              = tmon::DEFAULT_PUBLISH_ENVIRONMENT_TOPIC;
}
