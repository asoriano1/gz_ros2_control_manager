#include <gtest/gtest.h>

#include <chrono>

#include "gz_ros2_control_manager/ControllerManagerClient.hh"

using gz_ros2_control_manager::actionFor;
using gz_ros2_control_manager::buildSwitchRequest;
using gz_ros2_control_manager::buildLoadRequest;
using gz_ros2_control_manager::buildConfigureRequest;
using gz_ros2_control_manager::ControllerManagerClient;
using SwitchRequest     = controller_manager_msgs::srv::SwitchController::Request;
using LoadRequest       = controller_manager_msgs::srv::LoadController::Request;
using ConfigureRequest  = controller_manager_msgs::srv::ConfigureController::Request;

// ---------------------------------------------------------------------------
// actionFor
// ---------------------------------------------------------------------------

TEST(ActionFor, ActiveOffersDeactivate)
{
  const auto a = actionFor("active");
  EXPECT_TRUE(a.isActionable);
  EXPECT_EQ(a.label, "Deactivate");
  EXPECT_FALSE(a.activates);
}

TEST(ActionFor, InactiveOffersActivate)
{
  const auto a = actionFor("inactive");
  EXPECT_TRUE(a.isActionable);
  EXPECT_EQ(a.label, "Activate");
  EXPECT_TRUE(a.activates);
}

TEST(ActionFor, UnconfiguredOffersConfigureAction)
{
  const auto a = actionFor("unconfigured");
  EXPECT_TRUE(a.isActionable);
  EXPECT_EQ(a.label, "Configure");
  // Configure does not activate the controller — it only moves it to inactive.
  EXPECT_FALSE(a.activates);
}

TEST(ActionFor, FinalizedHasNoAction)
{
  const auto a = actionFor("finalized");
  EXPECT_FALSE(a.isActionable);
  EXPECT_EQ(a.label, "");
  EXPECT_FALSE(a.activates);
}

TEST(ActionFor, EmptyHasNoAction)
{
  const auto a = actionFor("");
  EXPECT_FALSE(a.isActionable);
  EXPECT_EQ(a.label, "");
  EXPECT_FALSE(a.activates);
}

TEST(ActionFor, UnknownTextHasNoAction)
{
  // Anything that isn't literal "active" / "inactive" must not surface a
  // button — this protects future-us from accidentally exposing actions for
  // states the MVP does not handle (e.g., a typo'd "ACTIVE").
  const auto a = actionFor("ACTIVE");
  EXPECT_FALSE(a.isActionable);
  EXPECT_EQ(a.label, "");
  EXPECT_FALSE(a.activates);
}

// ---------------------------------------------------------------------------
// buildSwitchRequest
// ---------------------------------------------------------------------------

TEST(BuildSwitchRequest, ActivateSetsOnlyActivateField)
{
  using std::chrono::seconds;
  auto req = buildSwitchRequest("foo", true, seconds{3});
  ASSERT_TRUE(static_cast<bool>(req));
  EXPECT_EQ(req->activate_controllers, std::vector<std::string>{"foo"});
  EXPECT_TRUE(req->deactivate_controllers.empty());
}

TEST(BuildSwitchRequest, DeactivateSetsOnlyDeactivateField)
{
  using std::chrono::seconds;
  auto req = buildSwitchRequest("foo", false, seconds{3});
  ASSERT_TRUE(static_cast<bool>(req));
  EXPECT_EQ(req->deactivate_controllers, std::vector<std::string>{"foo"});
  EXPECT_TRUE(req->activate_controllers.empty());
}

TEST(BuildSwitchRequest, IsAlwaysStrict)
{
  using std::chrono::seconds;
  auto reqA = buildSwitchRequest("foo", true,  seconds{3});
  auto reqD = buildSwitchRequest("foo", false, seconds{3});
  EXPECT_EQ(reqA->strictness, SwitchRequest::STRICT);
  EXPECT_EQ(reqD->strictness, SwitchRequest::STRICT);
}

TEST(BuildSwitchRequest, ActivateAsapStaysFalse)
{
  using std::chrono::seconds;
  auto req = buildSwitchRequest("foo", true, seconds{3});
  EXPECT_FALSE(req->activate_asap);
}

TEST(BuildSwitchRequest, ServiceTimeoutIsFiniteAndApplied)
{
  using std::chrono::seconds;
  auto req = buildSwitchRequest("foo", true, seconds{3});
  EXPECT_EQ(req->timeout.sec, 3);
  EXPECT_EQ(req->timeout.nanosec, 0u);
}

TEST(BuildSwitchRequest, ServiceTimeoutPropagatesNonDefault)
{
  using std::chrono::seconds;
  auto req = buildSwitchRequest("foo", true, seconds{7});
  EXPECT_EQ(req->timeout.sec, 7);
}

TEST(BuildSwitchRequest, NeverProducesInfiniteTimeout)
{
  // Defensive: the MVP must never send timeout=0 (= infinite) on the wire.
  using std::chrono::seconds;
  auto req = buildSwitchRequest("foo", true,
                                ControllerManagerClient::kSwitchServiceTimeout);
  EXPECT_GT(req->timeout.sec, 0);
}

TEST(SwitchTimeouts, ClientWaitExceedsServerTimeout)
{
  // Sanity check: the client wait_for window must be strictly larger than
  // the server-side timeout so the client doesn't give up before the server
  // would have aborted its own switch.
  const auto srv = ControllerManagerClient::kSwitchServiceTimeout;
  const auto cli = ControllerManagerClient::kSwitchClientTimeout;
  using std::chrono::milliseconds;
  using std::chrono::duration_cast;
  EXPECT_GT(cli, duration_cast<milliseconds>(srv));
}

// ---------------------------------------------------------------------------
// buildLoadRequest
// ---------------------------------------------------------------------------

TEST(BuildLoadRequest, PropagatesControllerName)
{
  auto req = buildLoadRequest("my_controller");
  ASSERT_TRUE(static_cast<bool>(req));
  EXPECT_EQ(req->name, "my_controller");
}

TEST(BuildLoadRequest, EmptyNameProducesEmptyRequest)
{
  // The plugin rejects empty names before calling the service.  Still, the
  // request builder must not crash on empty input.
  auto req = buildLoadRequest("");
  ASSERT_TRUE(static_cast<bool>(req));
  EXPECT_EQ(req->name, "");
}

// ---------------------------------------------------------------------------
// buildConfigureRequest
// ---------------------------------------------------------------------------

TEST(BuildConfigureRequest, PropagatesControllerName)
{
  auto req = buildConfigureRequest("joint_state_broadcaster");
  ASSERT_TRUE(static_cast<bool>(req));
  EXPECT_EQ(req->name, "joint_state_broadcaster");
}

TEST(BuildConfigureRequest, EmptyNameProducesEmptyRequest)
{
  auto req = buildConfigureRequest("");
  ASSERT_TRUE(static_cast<bool>(req));
  EXPECT_EQ(req->name, "");
}

// ---------------------------------------------------------------------------
// Load/Configure timeout constants — sanity guards
// ---------------------------------------------------------------------------

TEST(LoadConfigureTimeouts, ClientTimeoutIsPositive)
{
  EXPECT_GT(ControllerManagerClient::kLoadConfigureClientTimeout.count(), 0);
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
