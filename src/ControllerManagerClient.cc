#include "gz_ros2_control_manager/ControllerManagerClient.hh"

#include <algorithm>
#include <future>
#include <unordered_set>

#include <rcl_interfaces/msg/parameter_type.hpp>
#include <rcl_interfaces/srv/get_parameters.hpp>
#include <rcl_interfaces/srv/list_parameters.hpp>

namespace gz_ros2_control_manager
{

namespace
{

constexpr const char *kListControllersSuffix     = "/list_controllers";
constexpr const char *kListHwInterfacesSuffix    = "/list_hardware_interfaces";
constexpr const char *kListHwComponentsSuffix    = "/list_hardware_components";
constexpr const char *kSwitchControllerSuffix    = "/switch_controller";
constexpr const char *kLoadControllerSuffix      = "/load_controller";
constexpr const char *kConfigureControllerSuffix = "/configure_controller";
constexpr const char *kListParametersSuffix      = "/list_parameters";
constexpr const char *kGetParametersSuffix       = "/get_parameters";
constexpr const char *kTypeParamSuffix           = ".type";

// Parameters that look like "<name>.type" but are actually internal
// controller_manager housekeeping rather than declared controllers.
// (Kept tiny and conservative — if controller_manager grows new internal
// "*.type" parameters, this is the place to ignore them.)
const std::unordered_set<std::string> &reservedTopLevelNames()
{
  static const std::unordered_set<std::string> kSet = {
      "qos_overrides", "hardware_components_initial_state"};
  return kSet;
}

}  // namespace

// ---------------------------------------------------------------------------
// Static helpers
// ---------------------------------------------------------------------------

ControllerAction actionFor(const std::string &state)
{
  if (state == "active")       return ControllerAction{true, "Deactivate", false};
  if (state == "inactive")     return ControllerAction{true, "Activate",   true};
  if (state == "unconfigured") return ControllerAction{true, "Configure",  false};
  return ControllerAction{false, "", false};
}

controller_manager_msgs::srv::SwitchController::Request::SharedPtr
buildSwitchRequest(const std::string &controllerName,
                   bool activate,
                   std::chrono::seconds serviceSideTimeout)
{
  using ServiceT = controller_manager_msgs::srv::SwitchController;
  auto request = std::make_shared<ServiceT::Request>();
  if (activate)
    request->activate_controllers.push_back(controllerName);
  else
    request->deactivate_controllers.push_back(controllerName);
  // STRICT: a failed switch surfaces as ok=false instead of being silently
  // retried/relaxed.  This is the only mode supported by the MVP.
  request->strictness    = ServiceT::Request::STRICT;
  // Conservative: wait for hardware dependencies to be ready before flipping
  // the controllers (see SwitchController.idl docstring).
  request->activate_asap = false;
  // Finite server-side bound: if the transition can't complete in this many
  // seconds, the controller_manager aborts the switch and returns ok=false
  // rather than blocking forever (timeout=0 in the wire message).
  request->timeout       = builtin_interfaces::msg::Duration();
  request->timeout.sec   = static_cast<int32_t>(serviceSideTimeout.count());
  request->timeout.nanosec = 0u;
  return request;
}

controller_manager_msgs::srv::LoadController::Request::SharedPtr
buildLoadRequest(const std::string &controllerName)
{
  using ServiceT = controller_manager_msgs::srv::LoadController;
  auto request = std::make_shared<ServiceT::Request>();
  request->name = controllerName;
  return request;
}

controller_manager_msgs::srv::ConfigureController::Request::SharedPtr
buildConfigureRequest(const std::string &controllerName)
{
  using ServiceT = controller_manager_msgs::srv::ConfigureController;
  auto request = std::make_shared<ServiceT::Request>();
  request->name = controllerName;
  return request;
}

// ---------------------------------------------------------------------------
// Construction
// ---------------------------------------------------------------------------

ControllerManagerClient::ControllerManagerClient(rclcpp::Node::SharedPtr node)
: node_(std::move(node))
{
}

// ---------------------------------------------------------------------------
// list_controllers
// ---------------------------------------------------------------------------

ListControllersResult ControllerManagerClient::listControllers(
    const std::string &managerPath,
    std::chrono::milliseconds timeout) const
{
  using ServiceT = controller_manager_msgs::srv::ListControllers;

  ListControllersResult result;
  if (!node_)
    return result;

  const std::string serviceName = managerPath + kListControllersSuffix;
  auto client = node_->create_client<ServiceT>(serviceName);

  if (!client->wait_for_service(kDefaultDiscoveryTimeout))
    return result;
  result.serviceAvailable = true;

  auto request = std::make_shared<ServiceT::Request>();
  auto future = client->async_send_request(request);
  if (future.wait_for(timeout) != std::future_status::ready)
  {
    client->remove_pending_request(future);
    result.errorMessage = "Service call timed out: " + serviceName;
    return result;
  }

  ServiceT::Response::SharedPtr response;
  try
  {
    response = future.get();
  }
  catch (const std::exception &ex)
  {
    result.errorMessage = std::string("Service call threw: ") + ex.what();
    return result;
  }
  if (!response)
  {
    result.errorMessage = "Service returned a null response";
    return result;
  }

  result.callSucceeded = true;
  result.controllers.reserve(response->controller.size());
  for (const auto &c : response->controller)
  {
    ControllerInfo info;
    info.name  = c.name;
    info.type  = c.type;
    info.state = c.state;
    info.claimedInterfaces.assign(
        c.claimed_interfaces.begin(), c.claimed_interfaces.end());
    result.controllers.push_back(std::move(info));
  }
  return result;
}

// ---------------------------------------------------------------------------
// list_hardware_interfaces
// ---------------------------------------------------------------------------

ListHardwareInterfacesResult ControllerManagerClient::listHardwareInterfaces(
    const std::string &managerPath,
    std::chrono::milliseconds timeout) const
{
  using ServiceT = controller_manager_msgs::srv::ListHardwareInterfaces;

  ListHardwareInterfacesResult result;
  if (!node_)
    return result;

  const std::string serviceName = managerPath + kListHwInterfacesSuffix;
  auto client = node_->create_client<ServiceT>(serviceName);

  if (!client->wait_for_service(kDefaultDiscoveryTimeout))
    return result;
  result.serviceAvailable = true;

  auto request = std::make_shared<ServiceT::Request>();
  auto future = client->async_send_request(request);
  if (future.wait_for(timeout) != std::future_status::ready)
  {
    client->remove_pending_request(future);
    return result;
  }

  ServiceT::Response::SharedPtr response;
  try { response = future.get(); }
  catch (const std::exception &) { return result; }
  if (!response)
    return result;

  result.callSucceeded = true;

  auto fill = [](const auto &src, std::vector<HardwareInterfaceInfo> &dst)
  {
    dst.reserve(src.size());
    for (const auto &iface : src)
    {
      HardwareInterfaceInfo info;
      info.name        = iface.name;
      info.isAvailable = iface.is_available;
      info.isClaimed   = iface.is_claimed;
      dst.push_back(std::move(info));
    }
  };

  fill(response->command_interfaces, result.commandInterfaces);
  fill(response->state_interfaces,   result.stateInterfaces);
  return result;
}

// ---------------------------------------------------------------------------
// list_hardware_components
// ---------------------------------------------------------------------------

ListHardwareComponentsResult ControllerManagerClient::listHardwareComponents(
    const std::string &managerPath,
    std::chrono::milliseconds timeout) const
{
  using ServiceT = controller_manager_msgs::srv::ListHardwareComponents;

  ListHardwareComponentsResult result;
  if (!node_)
    return result;

  const std::string serviceName = managerPath + kListHwComponentsSuffix;
  auto client = node_->create_client<ServiceT>(serviceName);

  if (!client->wait_for_service(kDefaultDiscoveryTimeout))
    return result;
  result.serviceAvailable = true;

  auto request = std::make_shared<ServiceT::Request>();
  auto future = client->async_send_request(request);
  if (future.wait_for(timeout) != std::future_status::ready)
  {
    client->remove_pending_request(future);
    return result;
  }

  ServiceT::Response::SharedPtr response;
  try { response = future.get(); }
  catch (const std::exception &) { return result; }
  if (!response)
    return result;

  result.callSucceeded = true;
  result.components.reserve(response->component.size());

  // lifecycle_msgs/State: PRIMARY_STATE_{UNCONFIGURED=1, INACTIVE=2, ACTIVE=3,
  // FINALIZED=4}.  Service-side label is set on the message; if missing, fall
  // back to numeric primary_state.
  for (const auto &comp : response->component)
  {
    HardwareComponentInfo info;
    info.name       = comp.name;
    info.type       = comp.type;
    info.pluginName = comp.plugin_name;
    if (!comp.state.label.empty())
    {
      info.state = comp.state.label;
    }
    else
    {
      switch (comp.state.id)
      {
        case 1: info.state = "unconfigured"; break;
        case 2: info.state = "inactive";     break;
        case 3: info.state = "active";       break;
        case 4: info.state = "finalized";    break;
        default: info.state = "unknown";     break;
      }
    }
    result.components.push_back(std::move(info));
  }
  return result;
}

// ---------------------------------------------------------------------------
// list_configured_controllers (via rcl_interfaces parameter services)
// ---------------------------------------------------------------------------

ListConfiguredControllersResult ControllerManagerClient::listConfiguredControllers(
    const std::string &managerPath,
    std::chrono::milliseconds timeout) const
{
  using ListSvc = rcl_interfaces::srv::ListParameters;
  using GetSvc  = rcl_interfaces::srv::GetParameters;
  using ParameterType = rcl_interfaces::msg::ParameterType;

  ListConfiguredControllersResult result;
  if (!node_)
    return result;

  const std::string listName = managerPath + kListParametersSuffix;
  const std::string getName  = managerPath + kGetParametersSuffix;

  auto listClient = node_->create_client<ListSvc>(listName);
  auto getClient  = node_->create_client<GetSvc>(getName);

  if (!listClient->wait_for_service(kDefaultDiscoveryTimeout) ||
      !getClient ->wait_for_service(kDefaultDiscoveryTimeout))
  {
    return result;  // serviceAvailable stays false
  }
  result.serviceAvailable = true;

  // 1) ListParameters → flat list of all parameter names on the node.
  auto listRequest = std::make_shared<ListSvc::Request>();
  listRequest->prefixes = {};
  listRequest->depth = ListSvc::Request::DEPTH_RECURSIVE;
  auto listFuture = listClient->async_send_request(listRequest);
  if (listFuture.wait_for(timeout) != std::future_status::ready)
  {
    listClient->remove_pending_request(listFuture);
    result.errorMessage = "Service call timed out: " + listName;
    return result;
  }
  ListSvc::Response::SharedPtr listResponse;
  try { listResponse = listFuture.get(); }
  catch (const std::exception &ex)
  {
    result.errorMessage = std::string("list_parameters threw: ") + ex.what();
    return result;
  }
  if (!listResponse)
  {
    result.errorMessage = "list_parameters returned a null response";
    return result;
  }

  // 2) Extract candidate controller names from "<name>.type" parameters.
  //    "<name>" must not contain another '.' (i.e. only top-level dotted
  //    parameters), and must not be one of the reserved housekeeping prefixes.
  const auto isReserved = [](const std::string &n) {
    return reservedTopLevelNames().count(n) > 0;
  };
  std::vector<std::string> candidateNames;
  std::vector<std::string> candidateParamPaths;
  candidateNames.reserve(8);
  candidateParamPaths.reserve(8);

  const std::string suffix = kTypeParamSuffix;
  for (const auto &paramName : listResponse->result.names)
  {
    if (paramName.size() <= suffix.size())
      continue;
    if (!std::equal(suffix.rbegin(), suffix.rend(), paramName.rbegin()))
      continue;
    const std::string base =
        paramName.substr(0, paramName.size() - suffix.size());
    if (base.empty() || base.find('.') != std::string::npos)
      continue;
    if (isReserved(base))
      continue;
    candidateNames.push_back(base);
    candidateParamPaths.push_back(paramName);
  }

  // No declarations found — empty but successful answer.
  if (candidateNames.empty())
  {
    result.callSucceeded = true;
    return result;
  }

  // 3) GetParameters on the "<name>.type" entries to read the type string and
  //    confirm the parameter actually has a string value (vs. some unrelated
  //    "*.type" parameter that happens to share the same name).
  auto getRequest = std::make_shared<GetSvc::Request>();
  getRequest->names = candidateParamPaths;
  auto getFuture = getClient->async_send_request(getRequest);
  if (getFuture.wait_for(timeout) != std::future_status::ready)
  {
    getClient->remove_pending_request(getFuture);
    result.errorMessage = "Service call timed out: " + getName;
    return result;
  }
  GetSvc::Response::SharedPtr getResponse;
  try { getResponse = getFuture.get(); }
  catch (const std::exception &ex)
  {
    result.errorMessage = std::string("get_parameters threw: ") + ex.what();
    return result;
  }
  if (!getResponse)
  {
    result.errorMessage = "get_parameters returned a null response";
    return result;
  }

  result.callSucceeded = true;
  const std::size_t pairs = std::min(
      candidateNames.size(), getResponse->values.size());
  result.controllers.reserve(pairs);
  for (std::size_t i = 0; i < pairs; ++i)
  {
    const auto &val = getResponse->values[i];
    if (val.type != ParameterType::PARAMETER_STRING)
      continue;
    if (val.string_value.empty())
      continue;
    ConfiguredController c;
    c.name = candidateNames[i];
    c.type = val.string_value;
    result.controllers.push_back(std::move(c));
  }
  return result;
}

// ---------------------------------------------------------------------------
// switch_controller (Activate / Deactivate)
// ---------------------------------------------------------------------------

namespace
{

SwitchControllerResult switchSingle(
    rclcpp::Node::SharedPtr node,
    const std::string &managerPath,
    const std::string &controllerName,
    bool activate,
    std::chrono::seconds serviceSideTimeout,
    std::chrono::milliseconds clientWaitTimeout)
{
  using ServiceT = controller_manager_msgs::srv::SwitchController;

  SwitchControllerResult result;
  if (!node)
    return result;

  const std::string serviceName = managerPath + kSwitchControllerSuffix;
  auto client = node->create_client<ServiceT>(serviceName);

  if (!client->wait_for_service(
          ControllerManagerClient::kDefaultDiscoveryTimeout))
  {
    result.errorMessage = "Service unavailable: " + serviceName;
    return result;
  }
  result.serviceAvailable = true;

  auto request = buildSwitchRequest(
      controllerName, activate, serviceSideTimeout);

  auto future = client->async_send_request(request);
  if (future.wait_for(clientWaitTimeout) != std::future_status::ready)
  {
    client->remove_pending_request(future);
    result.clientTimedOut = true;
    result.errorMessage = "Switch call timed out client-side: " + serviceName;
    return result;
  }

  ServiceT::Response::SharedPtr response;
  try
  {
    response = future.get();
  }
  catch (const std::exception &ex)
  {
    result.errorMessage = std::string("Switch call threw: ") + ex.what();
    return result;
  }
  if (!response)
  {
    result.errorMessage = "Switch service returned a null response";
    return result;
  }

  result.callSucceeded = true;
  result.ok            = response->ok;
  result.errorMessage  = response->message;
  return result;
}

}  // namespace

// ---------------------------------------------------------------------------
// load_controller
// ---------------------------------------------------------------------------

LoadControllerResult ControllerManagerClient::loadController(
    const std::string &managerPath,
    const std::string &controllerName,
    std::chrono::milliseconds clientWaitTimeout) const
{
  using ServiceT = controller_manager_msgs::srv::LoadController;

  LoadControllerResult result;
  if (!node_)
    return result;

  const std::string serviceName = managerPath + kLoadControllerSuffix;
  auto client = node_->create_client<ServiceT>(serviceName);

  if (!client->wait_for_service(kDefaultDiscoveryTimeout))
  {
    result.errorMessage = "Service unavailable: " + serviceName;
    return result;
  }
  result.serviceAvailable = true;

  auto request = buildLoadRequest(controllerName);
  auto future  = client->async_send_request(request);
  if (future.wait_for(clientWaitTimeout) != std::future_status::ready)
  {
    client->remove_pending_request(future);
    result.clientTimedOut = true;
    result.errorMessage   = "Load call timed out: " + serviceName;
    return result;
  }

  ServiceT::Response::SharedPtr response;
  try { response = future.get(); }
  catch (const std::exception &ex)
  {
    result.errorMessage = std::string("Load call threw: ") + ex.what();
    return result;
  }
  if (!response)
  {
    result.errorMessage = "Load service returned a null response";
    return result;
  }

  result.callSucceeded = true;
  result.ok            = response->ok;
  return result;
}

// ---------------------------------------------------------------------------
// configure_controller
// ---------------------------------------------------------------------------

ConfigureControllerResult ControllerManagerClient::configureController(
    const std::string &managerPath,
    const std::string &controllerName,
    std::chrono::milliseconds clientWaitTimeout) const
{
  using ServiceT = controller_manager_msgs::srv::ConfigureController;

  ConfigureControllerResult result;
  if (!node_)
    return result;

  const std::string serviceName = managerPath + kConfigureControllerSuffix;
  auto client = node_->create_client<ServiceT>(serviceName);

  if (!client->wait_for_service(kDefaultDiscoveryTimeout))
  {
    result.errorMessage = "Service unavailable: " + serviceName;
    return result;
  }
  result.serviceAvailable = true;

  auto request = buildConfigureRequest(controllerName);
  auto future  = client->async_send_request(request);
  if (future.wait_for(clientWaitTimeout) != std::future_status::ready)
  {
    client->remove_pending_request(future);
    result.clientTimedOut = true;
    result.errorMessage   = "Configure call timed out: " + serviceName;
    return result;
  }

  ServiceT::Response::SharedPtr response;
  try { response = future.get(); }
  catch (const std::exception &ex)
  {
    result.errorMessage = std::string("Configure call threw: ") + ex.what();
    return result;
  }
  if (!response)
  {
    result.errorMessage = "Configure service returned a null response";
    return result;
  }

  result.callSucceeded = true;
  result.ok            = response->ok;
  return result;
}

// ---------------------------------------------------------------------------
// switch_controller (Activate / Deactivate)
// ---------------------------------------------------------------------------

SwitchControllerResult ControllerManagerClient::activateController(
    const std::string &managerPath,
    const std::string &controllerName,
    std::chrono::seconds serviceSideTimeout,
    std::chrono::milliseconds clientWaitTimeout) const
{
  return switchSingle(node_, managerPath, controllerName, /*activate=*/true,
                      serviceSideTimeout, clientWaitTimeout);
}

SwitchControllerResult ControllerManagerClient::deactivateController(
    const std::string &managerPath,
    const std::string &controllerName,
    std::chrono::seconds serviceSideTimeout,
    std::chrono::milliseconds clientWaitTimeout) const
{
  return switchSingle(node_, managerPath, controllerName, /*activate=*/false,
                      serviceSideTimeout, clientWaitTimeout);
}

}  // namespace gz_ros2_control_manager
