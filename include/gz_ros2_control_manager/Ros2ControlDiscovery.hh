#pragma once

#include <atomic>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>

namespace gz_ros2_control_manager
{

/// One controller_manager instance discovered at runtime.
struct DiscoveredManager
{
  /// Full ROS service path of the controller_manager node, e.g.
  ///   "/rbvogui/controller_manager"
  /// or "/controller_manager" if it lives in the root namespace.
  std::string managerPath;

  /// Namespace deduced from managerPath (may be empty for root-namespace
  /// controller_manager nodes).
  std::string modelNamespace;
};

/// Owns a private rclcpp::Node and a single-threaded executor running on a
/// dedicated background thread.  Discovery itself is purely service-name
/// based (NEVER ECM, NEVER URDF/SDF, NEVER topics): every ROS service whose
/// name ends in "/controller_manager/list_controllers" is reported as a
/// candidate controller_manager.
///
/// rclcpp::init() is called lazily on the first instance and never shut down
/// from this class — the Gazebo GUI process owns the lifecycle.  The class is
/// safe to use from a background QtConcurrent::run worker.
class Ros2ControlDiscovery
{
public:
  Ros2ControlDiscovery();
  ~Ros2ControlDiscovery();

  Ros2ControlDiscovery(const Ros2ControlDiscovery &) = delete;
  Ros2ControlDiscovery &operator=(const Ros2ControlDiscovery &) = delete;

  /// Background ROS 2 node, kept spinning by an internal executor thread.
  /// Service clients created against this node will receive their callbacks.
  std::shared_ptr<rclcpp::Node> node() const { return node_; }

  /// Synchronously query the ROS 2 graph for controller_manager instances.
  /// Returns one entry per "/.../controller_manager/list_controllers" service
  /// found.  Safe to call from any thread; intended to be called from a
  /// background worker because it can briefly contend on the graph cache.
  std::vector<DiscoveredManager> discover() const;

  /// Helper exposed for unit tests: derive {managerPath, modelNamespace} from
  /// a service name ending in "/controller_manager/list_controllers".
  /// Returns empty result if the service name does not match the pattern.
  static DiscoveredManager managerFromServiceName(const std::string &serviceName);

private:
  static void ensureRclcppInitialised();

  std::shared_ptr<rclcpp::Node> node_;
  rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_;
  std::thread spinThread_;
  std::atomic<bool> stopRequested_{false};
};

}  // namespace gz_ros2_control_manager
