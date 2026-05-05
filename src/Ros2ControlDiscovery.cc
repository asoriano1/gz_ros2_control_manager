#include "gz_ros2_control_manager/Ros2ControlDiscovery.hh"

#include <chrono>
#include <mutex>
#include <utility>

#include <rclcpp/logging.hpp>

namespace gz_ros2_control_manager
{

namespace
{

constexpr const char *kListControllersSuffix = "/controller_manager/list_controllers";
constexpr const char *kControllerManagerSuffix = "/controller_manager";

bool endsWith(const std::string &value, const std::string &suffix)
{
  if (suffix.size() > value.size())
    return false;
  return std::equal(suffix.rbegin(), suffix.rend(), value.rbegin());
}

}  // namespace

void Ros2ControlDiscovery::ensureRclcppInitialised()
{
  // rclcpp::init is only safe to call once globally.  Multiple plugins (or
  // repeated plugin loads) could race here, hence a process-local mutex.
  static std::mutex initMutex;
  std::lock_guard<std::mutex> lock(initMutex);
  if (!rclcpp::ok())
  {
    int argc = 0;
    char **argv = nullptr;
    rclcpp::init(argc, argv);
  }
}

Ros2ControlDiscovery::Ros2ControlDiscovery()
{
  ensureRclcppInitialised();

  rclcpp::NodeOptions options;
  options.start_parameter_event_publisher(false);
  options.start_parameter_services(false);

  node_ = std::make_shared<rclcpp::Node>(
      "gz_ros2_control_manager_gui_node", options);

  executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  executor_->add_node(node_);

  spinThread_ = std::thread([this]()
  {
    // spin() returns when the executor is cancelled.
    executor_->spin();
  });
}

Ros2ControlDiscovery::~Ros2ControlDiscovery()
{
  auto logger = node_ ? node_->get_logger()
                      : rclcpp::get_logger("gz_ros2_control_manager");
  RCLCPP_DEBUG(logger, "Ros2ControlDiscovery: stopping executor");
  stopRequested_ = true;
  if (executor_)
    executor_->cancel();
  if (spinThread_.joinable())
    spinThread_.join();
  RCLCPP_DEBUG(logger, "Ros2ControlDiscovery: spin thread joined");

  // Intentionally do NOT call rclcpp::shutdown(): other plugins or the host
  // application may still be using rclcpp, and the Gazebo GUI process owns
  // the global rclcpp lifecycle.
}

DiscoveredManager Ros2ControlDiscovery::managerFromServiceName(
    const std::string &serviceName)
{
  DiscoveredManager out;
  if (!endsWith(serviceName, kListControllersSuffix))
    return out;

  const std::size_t suffixLen = std::string(kListControllersSuffix).size();
  out.managerPath = serviceName.substr(0, serviceName.size() - suffixLen)
                  + std::string(kControllerManagerSuffix);

  // Strip the trailing "/controller_manager" to derive the model namespace.
  // For a root-namespace controller_manager (i.e. "/controller_manager"),
  // the resulting namespace is empty, which is a valid case.
  const std::size_t cmLen = std::string(kControllerManagerSuffix).size();
  if (out.managerPath.size() >= cmLen)
    out.modelNamespace = out.managerPath.substr(0, out.managerPath.size() - cmLen);

  return out;
}

std::vector<DiscoveredManager> Ros2ControlDiscovery::discover() const
{
  std::vector<DiscoveredManager> result;
  if (!node_)
    return result;

  const auto servicesAndTypes = node_->get_service_names_and_types();
  result.reserve(servicesAndTypes.size() / 8 + 1);

  for (const auto &entry : servicesAndTypes)
  {
    const std::string &serviceName = entry.first;
    if (!endsWith(serviceName, kListControllersSuffix))
      continue;

    auto manager = managerFromServiceName(serviceName);
    if (!manager.managerPath.empty())
      result.push_back(std::move(manager));
  }
  return result;
}

}  // namespace gz_ros2_control_manager
