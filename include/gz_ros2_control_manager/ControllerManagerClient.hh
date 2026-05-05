#pragma once

#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <controller_manager_msgs/srv/configure_controller.hpp>
#include <controller_manager_msgs/srv/list_controllers.hpp>
#include <controller_manager_msgs/srv/list_hardware_components.hpp>
#include <controller_manager_msgs/srv/list_hardware_interfaces.hpp>
#include <controller_manager_msgs/srv/load_controller.hpp>
#include <controller_manager_msgs/srv/switch_controller.hpp>

namespace gz_ros2_control_manager
{

/// Minimal POD describing a controller's runtime state, lifted from
/// controller_manager_msgs/msg/ControllerState.
struct ControllerInfo
{
  std::string name;
  std::string type;
  std::string state;            ///< "active" / "inactive" / "unconfigured" / etc.
  std::vector<std::string> claimedInterfaces;
};

struct HardwareInterfaceInfo
{
  std::string name;
  bool isAvailable{false};
  bool isClaimed{false};
};

struct HardwareComponentInfo
{
  std::string name;
  std::string type;
  std::string pluginName;
  std::string state;
};

/// One entry from the controller_manager parameter table that looks like a
/// controller declaration (i.e. a string parameter named "<name>.type").
/// Discovered before the controller is loaded, so it has no runtime state.
struct ConfiguredController
{
  std::string name;
  std::string type;
};

struct ListControllersResult
{
  bool serviceAvailable{false};
  bool callSucceeded{false};
  std::string errorMessage;
  std::vector<ControllerInfo> controllers;
};

struct ListHardwareInterfacesResult
{
  bool serviceAvailable{false};
  bool callSucceeded{false};
  std::vector<HardwareInterfaceInfo> commandInterfaces;
  std::vector<HardwareInterfaceInfo> stateInterfaces;
};

struct ListHardwareComponentsResult
{
  bool serviceAvailable{false};
  bool callSucceeded{false};
  std::vector<HardwareComponentInfo> components;
};

struct ListConfiguredControllersResult
{
  bool serviceAvailable{false};
  bool callSucceeded{false};
  std::string errorMessage;
  std::vector<ConfiguredController> controllers;
};

struct SwitchControllerResult
{
  bool serviceAvailable{false};
  bool callSucceeded{false};
  bool ok{false};
  std::string errorMessage;

  /// True iff the client wait_for() returned without the future being ready.
  /// Distinct from callSucceeded=false (which also covers null/throw cases).
  bool clientTimedOut{false};
};

struct LoadControllerResult
{
  bool serviceAvailable{false};
  bool callSucceeded{false};
  bool ok{false};
  bool clientTimedOut{false};
  std::string errorMessage;
};

struct ConfigureControllerResult
{
  bool serviceAvailable{false};
  bool callSucceeded{false};
  bool ok{false};
  bool clientTimedOut{false};
  std::string errorMessage;
};

/// UI-facing decision for a controller row, computed from the state string.
/// Kept as a free helper (see actionFor) so the same mapping can be unit
/// tested without spinning up rclcpp.
struct ControllerAction
{
  bool        isActionable{false};   ///< Show an Activate/Deactivate button.
  std::string label;                 ///< "Activate" / "Deactivate" / "".
  bool        activates{false};      ///< When isActionable, true=activate.
};

/// Map a controller_manager state string to the UI action that should be
/// presented.
///   active       → Deactivate (switch_controller)
///   inactive     → Activate   (switch_controller)
///   unconfigured → Configure  (configure_controller)
/// Anything else (finalized, empty, unknown text) returns isActionable=false.
ControllerAction actionFor(const std::string &state);

/// Build a SwitchController request matching the MVP rules:
///   - exactly one of activate_controllers / deactivate_controllers is set
///   - strictness = STRICT
///   - activate_asap = false
///   - timeout = serviceSideTimeout (finite, server-side bound)
controller_manager_msgs::srv::SwitchController::Request::SharedPtr
buildSwitchRequest(const std::string &controllerName,
                   bool activate,
                   std::chrono::seconds serviceSideTimeout);

/// Build a LoadController request (controller name only).
controller_manager_msgs::srv::LoadController::Request::SharedPtr
buildLoadRequest(const std::string &controllerName);

/// Build a ConfigureController request (controller name only).
controller_manager_msgs::srv::ConfigureController::Request::SharedPtr
buildConfigureRequest(const std::string &controllerName);

/// Synchronous wrapper over the controller_manager service interface.
///
/// Every call has a bounded timeout and returns a structured result instead
/// of throwing — the GUI must remain responsive when a controller_manager
/// disappears or a service is missing.
///
/// All calls expect to be made from a worker thread; callbacks are dispatched
/// by the executor that owns the underlying node (see Ros2ControlDiscovery).
class ControllerManagerClient
{
public:
  static constexpr std::chrono::milliseconds kDefaultServiceTimeout{1500};
  static constexpr std::chrono::milliseconds kDefaultDiscoveryTimeout{500};

  /// Server-side timeout for STRICT switch_controller requests.  Bounded so
  /// the controller_manager aborts a stuck transition rather than blocking
  /// indefinitely (timeout=0 in the wire message means "infinite").
  static constexpr std::chrono::seconds      kSwitchServiceTimeout{3};

  /// Client-side wait used by Activate/Deactivate.  Strictly larger than
  /// kSwitchServiceTimeout so the server normally answers first; if it
  /// doesn't, the client gives up and the UI surfaces an explicit timeout.
  static constexpr std::chrono::milliseconds kSwitchClientTimeout{4000};

  /// Client-side wait for load_controller and configure_controller.
  /// These services have no server-side timeout parameter.
  static constexpr std::chrono::milliseconds kLoadConfigureClientTimeout{3000};

  explicit ControllerManagerClient(rclcpp::Node::SharedPtr node);

  ListControllersResult listControllers(
      const std::string &managerPath,
      std::chrono::milliseconds timeout = kDefaultServiceTimeout) const;

  ListHardwareInterfacesResult listHardwareInterfaces(
      const std::string &managerPath,
      std::chrono::milliseconds timeout = kDefaultServiceTimeout) const;

  ListHardwareComponentsResult listHardwareComponents(
      const std::string &managerPath,
      std::chrono::milliseconds timeout = kDefaultServiceTimeout) const;

  /// Read-only discovery of controllers configured on the controller_manager
  /// node via its rcl_interfaces parameter services.  A controller is
  /// recognised by the presence of a string parameter "<name>.type".
  ///
  /// Returns serviceAvailable=false (no callSucceeded set) when the parameter
  /// services are not exposed, in which case the caller should silently hide
  /// the section.
  ListConfiguredControllersResult listConfiguredControllers(
      const std::string &managerPath,
      std::chrono::milliseconds timeout = kDefaultServiceTimeout) const;

  /// Activate a single, already-loaded controller using STRICT strictness.
  /// serviceSideTimeout is written into request->timeout; clientWaitTimeout
  /// is the caller-side ceiling for wait_for(future).
  SwitchControllerResult activateController(
      const std::string &managerPath,
      const std::string &controllerName,
      std::chrono::seconds serviceSideTimeout = kSwitchServiceTimeout,
      std::chrono::milliseconds clientWaitTimeout = kSwitchClientTimeout) const;

  /// Deactivate a single, already-loaded controller using STRICT strictness.
  SwitchControllerResult deactivateController(
      const std::string &managerPath,
      const std::string &controllerName,
      std::chrono::seconds serviceSideTimeout = kSwitchServiceTimeout,
      std::chrono::milliseconds clientWaitTimeout = kSwitchClientTimeout) const;

  /// Call load_controller for a controller that is declared in parameters
  /// but not yet loaded into the controller_manager.
  LoadControllerResult loadController(
      const std::string &managerPath,
      const std::string &controllerName,
      std::chrono::milliseconds clientWaitTimeout = kLoadConfigureClientTimeout) const;

  /// Call configure_controller to move a loaded controller from unconfigured
  /// to inactive state.
  ConfigureControllerResult configureController(
      const std::string &managerPath,
      const std::string &controllerName,
      std::chrono::milliseconds clientWaitTimeout = kLoadConfigureClientTimeout) const;

private:
  rclcpp::Node::SharedPtr node_;
};

}  // namespace gz_ros2_control_manager
