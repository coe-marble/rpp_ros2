#include <chrono>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

#include <rpp_cpp/component_status.hpp>
#include <rpp_cpp/handover_state.hpp>
#include <rpp_ros2/slot_manager_ros.hpp>

namespace
{
using namespace std::chrono_literals;

class TestContext
{
public:
  std::vector<std::string> list_subcomponents() const
  {
    return {"controller", "localization"};
  }
};

class TestRuntime
{
public:
  struct Entry
  {
    rpp::LifecycleState state{rpp::LifecycleState::UNCONFIGURED};
    std::string component;
    std::string instance_id;
  };

  TestRuntime()
  {
    entries_.emplace("controller/primary", Entry{
      rpp::LifecycleState::UNCONFIGURED, "rpp_control::Controller", "controller-id"});
    entries_.emplace("localization/primary", Entry{
      rpp::LifecycleState::UNCONFIGURED, "rpp_localization::Filter", "localization-id"});
  }

  const TestContext & get_context() const {return context_;}
  bool allows_dynamic_components(const std::string &) const {return true;}

  std::vector<rpp::ComponentStatus> get_subcomponent_statuses(const std::string & slot) const
  {
    std::vector<rpp::ComponentStatus> result;
    for (const auto & item : entries_) {
      if (item.first.rfind(slot + "/", 0) != 0) {continue;}
      const auto name = item.first.substr(slot.size() + 1);
      result.push_back({name, item.second.component, item.second.instance_id, item.second.state,
          std::nullopt, {}, item.second.state == rpp::LifecycleState::ACTIVE, 0});
    }
    return result;
  }

  rpp::CallbackResult configure_component(const std::string & slot, const std::string & name)
  {
    return transition(slot, name, rpp::LifecycleState::INACTIVE, "configure");
  }

  rpp::CallbackResult activate_component(
    const std::string & slot, const std::string & name, rpp::HandoverState = {})
  {
    return transition(slot, name, rpp::LifecycleState::ACTIVE, "activate");
  }

  rpp::DeactivationResult<rpp::HandoverState> deactivate_component(
    const std::string & slot, const std::string & name, std::chrono::milliseconds)
  {
    return {transition(slot, name, rpp::LifecycleState::INACTIVE, "deactivate"), std::nullopt};
  }

  rpp::CallbackResult cleanup_component(const std::string & slot, const std::string & name)
  {
    return transition(slot, name, rpp::LifecycleState::UNCONFIGURED, "cleanup");
  }

  rpp::CallbackResult load_component(
    const std::string & slot, const std::string & name, const std::string & component)
  {
    entries_[key(slot, name)] = {rpp::LifecycleState::UNCONFIGURED, component, name + "-id"};
    events.push_back("load:" + slot + "/" + name);
    return {};
  }

  rpp::CallbackResult unload_component(
    const std::string & slot, const std::string & name, std::chrono::milliseconds)
  {
    entries_.erase(key(slot, name));
    events.push_back("unload:" + slot + "/" + name);
    return {};
  }

  rpp::CallbackResult reload_component(
    const std::string & slot, const std::string & name, bool, std::chrono::milliseconds)
  {
    events.push_back("reload:" + slot + "/" + name);
    return {};
  }

  rpp::CallbackResult switch_component(
    const std::string & slot, const std::string & source, const std::string & target,
    bool, std::chrono::milliseconds)
  {
    auto & from = entries_.at(key(slot, source));
    auto & to = entries_.at(key(slot, target));
    from.state = rpp::LifecycleState::INACTIVE;
    to.state = rpp::LifecycleState::ACTIVE;
    events.push_back("switch:" + slot + "/" + source + "/" + target);
    return {};
  }

  std::vector<std::string> events;

private:
  static std::string key(const std::string & slot, const std::string & name)
  {
    return slot + "/" + name;
  }

  rpp::CallbackResult transition(
    const std::string & slot, const std::string & name, rpp::LifecycleState state,
    const std::string & action)
  {
    const auto found = entries_.find(key(slot, name));
    if (found == entries_.end()) {return rpp::CallbackResult::failure("Unknown component");}
    found->second.state = state;
    events.push_back(action + ":" + slot + "/" + name);
    return {};
  }

  TestContext context_;
  std::map<std::string, Entry> entries_;
};

class SlotManagerRosTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    if (!rclcpp::ok()) {rclcpp::init(0, nullptr);}
    node_ = std::make_shared<rclcpp::Node>("slot_manager_ros_test");
    executor_.add_node(node_);
  }

  void TearDown() override
  {
    helper_.reset();
    executor_.remove_node(node_);
    node_.reset();
    if (rclcpp::ok()) {rclcpp::shutdown();}
  }

  void create_helper(rpp_ros2::SlotManagerRosCallbacks callbacks = {})
  {
    rpp_ros2::SlotManagerRosOptions options;
    options.publish_period = 0ms;
    helper_ = std::make_unique<rpp_ros2::SlotManagerRos<TestRuntime>>(
      *node_, runtime_, std::move(options), std::move(callbacks));
  }

  template<class Service>
  std::shared_ptr<typename Service::Response> call(typename Service::Request request)
  {
    auto client = node_->create_client<Service>(
      "/slot_manager_ros_test/controller/" + service_name<Service>());
    if (!client->wait_for_service(1s)) {throw std::runtime_error("Service did not become ready");}
    auto future = client->async_send_request(std::make_shared<typename Service::Request>(
        std::move(request)));
    if (executor_.spin_until_future_complete(future, 1s) != rclcpp::FutureReturnCode::SUCCESS) {
      throw std::runtime_error("Service call did not complete");
    }
    return future.get();
  }

  template<class Service>
  static std::string service_name();

  TestRuntime runtime_;
  std::shared_ptr<rclcpp::Node> node_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  std::unique_ptr<rpp_ros2::SlotManagerRos<TestRuntime>> helper_;
};

template<>
std::string SlotManagerRosTest::service_name<rpp_ros2::srv::ConfigureComponent>()
{
  return "configure";
}

template<>
std::string SlotManagerRosTest::service_name<rpp_ros2::srv::ActivateComponent>()
{
  return "activate";
}

template<>
std::string SlotManagerRosTest::service_name<rpp_ros2::srv::GetSlotStatus>()
{
  return "get_status";
}

template<>
std::string SlotManagerRosTest::service_name<rpp_ros2::srv::ListComponents>()
{
  return "list_components";
}

TEST_F(SlotManagerRosTest, CreatesIndependentServiceSetsForEachRuntimeSlot)
{
  create_helper();

  const auto services = node_->get_service_names_and_types();
  for (const std::string & slot : {"controller", "localization"}) {
    for (const std::string & action : {"configure", "activate", "deactivate", "cleanup", "load",
         "unload", "reload", "switch", "get_status", "list_components"})
    {
      EXPECT_NE(services.find("/slot_manager_ros_test/" + slot + "/" + action), services.end());
    }
  }

  auto status = call<rpp_ros2::srv::GetSlotStatus>({});
  ASSERT_TRUE(status->success) << status->message;
  ASSERT_EQ(status->status.name, "controller");
  ASSERT_EQ(status->status.components.size(), 1U);
  EXPECT_EQ(status->status.components[0].name, "primary");
  EXPECT_EQ(status->status.components[0].state, rpp_ros2::msg::ComponentStatus::UNCONFIGURED);
}

TEST_F(SlotManagerRosTest, CommandsUseTheirServiceSlotAndPublishTheResultingState)
{
  create_helper();

  rpp_ros2::srv::ConfigureComponent::Request configure;
  configure.name = "primary";
  auto configured = call<rpp_ros2::srv::ConfigureComponent>(std::move(configure));
  ASSERT_TRUE(configured->success) << configured->message;

  rpp_ros2::srv::ActivateComponent::Request activate;
  activate.name = "primary";
  auto activated = call<rpp_ros2::srv::ActivateComponent>(std::move(activate));
  ASSERT_TRUE(activated->success) << activated->message;

  rpp_ros2::srv::ListComponents::Request list;
  list.loaded_only = true;
  auto components = call<rpp_ros2::srv::ListComponents>(std::move(list));
  ASSERT_TRUE(components->success) << components->message;
  ASSERT_EQ(components->components.size(), 1U);
  EXPECT_EQ(components->components[0].state, rpp_ros2::msg::ComponentStatus::ACTIVE);
  EXPECT_EQ(runtime_.events, (std::vector<std::string>{
      "configure:controller/primary", "activate:controller/primary"}));
}

TEST_F(SlotManagerRosTest, InvokesTheOptionalCallbackAfterSuccessfulOperations)
{
  std::vector<rpp_ros2::SlotOperation> observed;
  rpp_ros2::SlotManagerRosCallbacks callbacks;
  callbacks.after_success = [&](const rpp_ros2::SlotOperation & operation) {
      observed.push_back(operation);
      return rpp::CallbackResult{};
    };
  create_helper(std::move(callbacks));

  rpp_ros2::srv::ConfigureComponent::Request request;
  request.name = "primary";
  auto response = call<rpp_ros2::srv::ConfigureComponent>(std::move(request));
  ASSERT_TRUE(response->success) << response->message;
  ASSERT_EQ(observed.size(), 1U);
  EXPECT_EQ(observed[0].kind, rpp_ros2::SlotOperation::Kind::Configure);
  EXPECT_EQ(observed[0].slot, "controller");
  EXPECT_EQ(observed[0].target, "primary");
}

}  // namespace
