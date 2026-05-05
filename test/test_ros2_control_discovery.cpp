#include <gtest/gtest.h>

#include "gz_ros2_control_manager/Ros2ControlDiscovery.hh"

using gz_ros2_control_manager::Ros2ControlDiscovery;

TEST(ManagerFromServiceName, NamespacedManagerRbvogui)
{
  const auto m = Ros2ControlDiscovery::managerFromServiceName(
      "/rbvogui/controller_manager/list_controllers");
  EXPECT_EQ(m.managerPath, "/rbvogui/controller_manager");
  EXPECT_EQ(m.modelNamespace, "/rbvogui");
}

TEST(ManagerFromServiceName, NamespacedManagerRobot1)
{
  const auto m = Ros2ControlDiscovery::managerFromServiceName(
      "/robot1/controller_manager/list_controllers");
  EXPECT_EQ(m.managerPath, "/robot1/controller_manager");
  EXPECT_EQ(m.modelNamespace, "/robot1");
}

TEST(ManagerFromServiceName, NestedNamespaceManager)
{
  const auto m = Ros2ControlDiscovery::managerFromServiceName(
      "/robots/foo/controller_manager/list_controllers");
  EXPECT_EQ(m.managerPath, "/robots/foo/controller_manager");
  EXPECT_EQ(m.modelNamespace, "/robots/foo");
}

TEST(ManagerFromServiceName, RootNamespaceManager)
{
  const auto m = Ros2ControlDiscovery::managerFromServiceName(
      "/controller_manager/list_controllers");
  EXPECT_EQ(m.managerPath, "/controller_manager");
  EXPECT_EQ(m.modelNamespace, "");
}

TEST(ManagerFromServiceName, IgnoresUnrelatedServiceOnSameManager)
{
  // switch_controller is a controller_manager service, but not the discovery
  // anchor — should be ignored.
  const auto m = Ros2ControlDiscovery::managerFromServiceName(
      "/rbvogui/controller_manager/switch_controller");
  EXPECT_EQ(m.managerPath, "");
  EXPECT_EQ(m.modelNamespace, "");
}

TEST(ManagerFromServiceName, IgnoresArbitraryListControllers)
{
  // A service named */something_else/list_controllers must not match — only
  // names with the literal "/controller_manager/list_controllers" tail count.
  const auto m = Ros2ControlDiscovery::managerFromServiceName(
      "/foo/bar/list_controllers");
  EXPECT_EQ(m.managerPath, "");
  EXPECT_EQ(m.modelNamespace, "");
}

TEST(ManagerFromServiceName, IgnoresBareSuffix)
{
  const auto m = Ros2ControlDiscovery::managerFromServiceName(
      "list_controllers");
  EXPECT_EQ(m.managerPath, "");
  EXPECT_EQ(m.modelNamespace, "");
}

TEST(ManagerFromServiceName, IgnoresEmptyString)
{
  const auto m = Ros2ControlDiscovery::managerFromServiceName("");
  EXPECT_EQ(m.managerPath, "");
  EXPECT_EQ(m.modelNamespace, "");
}

TEST(ManagerFromServiceName, IgnoresPartialMatch)
{
  // Sanity: the suffix appearing as a substring (not at the end) must be
  // ignored.
  const auto m = Ros2ControlDiscovery::managerFromServiceName(
      "/robot/controller_manager/list_controllers/extra");
  EXPECT_EQ(m.managerPath, "");
  EXPECT_EQ(m.modelNamespace, "");
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
