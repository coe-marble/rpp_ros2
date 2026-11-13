#pragma once

#include <chrono>
#include <cstdint>
#include <exception>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rpp_cpp/runtime.hpp>
#include <rpp_ros2/component_lookup.hpp>
#include <rpp_ros2/msg/component_status.hpp>
#include <rpp_ros2/msg/slot_status.hpp>
#include <rpp_ros2/srv/activate_component.hpp>
#include <rpp_ros2/srv/cleanup_component.hpp>
#include <rpp_ros2/srv/configure_component.hpp>
#include <rpp_ros2/srv/deactivate_component.hpp>
#include <rpp_ros2/srv/get_slot_status.hpp>
#include <rpp_ros2/srv/list_components.hpp>
#include <rpp_ros2/srv/load_component.hpp>
#include <rpp_ros2/srv/reload_component.hpp>
#include <rpp_ros2/srv/switch_components.hpp>
#include <rpp_ros2/srv/unload_component.hpp>

namespace rpp_ros2
{

struct SlotManagerRosOptions
{
  /// Private node namespace: ~/slot/action expands to /namespace/node/slot/action.
  std::string namespace_prefix = "~";
  /// Zero disables periodic publication; commands still publish resulting status.
  std::chrono::milliseconds publish_period{1000};
  rclcpp::CallbackGroup::SharedPtr callback_group;
};

/**
 * One ROS service set and status topic per slot of a built RPP runtime.
 * The node and runtime must outlive this helper; do not move the runtime while
 * this helper refers to it. Spin a SingleThreadedExecutor on the construction
 * thread and use that thread for all other lifecycle/membership operations,
 * including with concurrent RPP runtimes. No executor or worker is created.
 * Destroy/close after callbacks stop. Root-runtime ownership remains with the app.
 */
template<class Runtime>
class SlotManagerRos
{
  using StatusService = rpp_ros2::srv::GetSlotStatus;
  struct SlotEntities
  {
    rclcpp::Publisher<rpp_ros2::msg::SlotStatus>::SharedPtr publisher;
    std::vector<rclcpp::ServiceBase::SharedPtr> services;
  };

public:
  SlotManagerRos(
    rclcpp::Node & node, Runtime & runtime, SlotManagerRosOptions options = {})
  : node_(node), runtime_(runtime), options_(std::move(options)),
    owner_(std::this_thread::get_id())
  {
    if (options_.namespace_prefix.empty()) {
      throw std::invalid_argument("A nonempty ROS namespace is required");
    }
    if (options_.publish_period.count() < 0) {
      throw std::invalid_argument("Status period must be non-negative");
    }
    while (!options_.namespace_prefix.empty() && options_.namespace_prefix.back() == '/') {
      options_.namespace_prefix.pop_back();
    }
    group_ = options_.callback_group ? options_.callback_group :
      node_.get_node_base_interface()->get_default_callback_group();
    if (group_->type() != rclcpp::CallbackGroupType::MutuallyExclusive) {
      throw std::invalid_argument("Slot operations require a mutually exclusive callback group");
    }
    for (const auto & slot : runtime_.get_context().list_subcomponents()) {add_slot(slot);}
    publish_status();
    if (options_.publish_period.count() > 0) {
      timer_ = node_.create_wall_timer(options_.publish_period, [this] {tick();}, group_);
    }
  }

  ~SlotManagerRos() = default;
  SlotManagerRos(const SlotManagerRos &) = delete;
  SlotManagerRos & operator=(const SlotManagerRos &) = delete;
  SlotManagerRos(SlotManagerRos &&) = delete;
  SlotManagerRos & operator=(SlotManagerRos &&) = delete;

  /// Publish all declared slots, including empty slots and unloaded/error entries.
  void publish_status()
  {
    check_owner();
    for (const auto & item : slots_) {publish_status(item.first);}
  }

  void publish_status(const std::string & slot)
  {
    auto message = snapshot(slot);
    slots_.at(slot).publisher->publish(message);
  }

  /// Release ROS entities without configuring, activating or closing the runtime.
  void close()
  {
    if (closed_) {return;}
    check_owner();
    closed_ = true;
    timer_.reset();
    slots_.clear();
  }

private:
  std::string path(const std::string & slot, const std::string & action) const
  {
    return options_.namespace_prefix + "/" + slot + "/" + action;
  }

  void check_owner() const
  {
    if (std::this_thread::get_id() != owner_) {
      throw std::logic_error(
              "Run slot operations on the construction thread with a SingleThreadedExecutor");
    }
    if (closed_) {throw std::logic_error("The ROS slot-manager helper is closed");}
  }

  static uint8_t state_id(rpp::LifecycleState state)
  {
    using Status = rpp_ros2::msg::ComponentStatus;
    switch (state) {
      case rpp::LifecycleState::UNLOADED: return Status::UNLOADED;
      case rpp::LifecycleState::UNCONFIGURED: return Status::UNCONFIGURED;
      case rpp::LifecycleState::INACTIVE: return Status::INACTIVE;
      case rpp::LifecycleState::ACTIVE: return Status::ACTIVE;
      case rpp::LifecycleState::ERROR: return Status::ERROR;
    }
    throw std::logic_error("Unknown RPP lifecycle state");
  }

  rpp_ros2::msg::SlotStatus snapshot(const std::string & name) const
  {
    check_owner();
    rpp_ros2::msg::SlotStatus slot;
    slot.header.stamp = node_.now();
    slot.name = name;
    slot.allows_dynamic_components = runtime_.allows_dynamic_components(name);
    for (const auto & info : runtime_.get_subcomponent_statuses(name)) {
      rpp_ros2::msg::ComponentStatus status;
      status.name = info.name;
      status.component = info.component;
      status.instance_id = info.instance_id;
      status.state = state_id(info.state);
      status.transition = info.transition.value_or("");
      status.callable = info.callable;
      status.in_flight = info.in_flight;
      status.last_success = info.last_result.success;
      status.last_message = info.last_result.message;
      slot.components.push_back(std::move(status));
    }
    return slot;
  }

  void add_slot(const std::string & slot)
  {
    auto & entities = slots_[slot];
    entities.publisher = node_.create_publisher<rpp_ros2::msg::SlotStatus>(
      path(slot, "status"), rclcpp::QoS(1).reliable().transient_local());
    add_command<srv::ConfigureComponent>(slot, "configure", [this, slot](const auto & req) {
        return runtime_.configure_component(slot, req.name);
      });
    add_command<srv::ActivateComponent>(slot, "activate", [this, slot](const auto & req) {
        return runtime_.activate_component(slot, req.name);
      });
    add_command<srv::DeactivateComponent>(slot, "deactivate", [this, slot](const auto & req) {
        return runtime_.deactivate_component(
          slot, req.name, std::chrono::milliseconds(req.timeout_ms)).result;
      });
    add_command<srv::CleanupComponent>(slot, "cleanup", [this, slot](const auto & req) {
        return runtime_.cleanup_component(slot, req.name);
      });
    add_command<srv::LoadComponent>(slot, "load", [this, slot](const auto & req) {
        const auto component = req.library_name.empty() ? req.component :
          resolve_library_component(req.library_name, req.component);
        return runtime_.load_component(slot, req.name, component);
      });
    add_command<srv::UnloadComponent>(slot, "unload", [this, slot](const auto & req) {
        return runtime_.unload_component(slot, req.name, std::chrono::milliseconds(req.timeout_ms));
      });
    add_command<srv::ReloadComponent>(slot, "reload", [this, slot](const auto & req) {
        return runtime_.reload_component(
          slot, req.name, req.transfer_state, std::chrono::milliseconds(req.timeout_ms));
      });
    add_command<srv::SwitchComponents>(slot, "switch", [this, slot](const auto & req) {
        return runtime_.switch_component(
          slot, req.source, req.target, req.transfer_state, std::chrono::milliseconds(req.timeout_ms));
      });
    entities.services.push_back(node_.create_service<StatusService>(
      path(slot, "get_status"),
      [this, slot](const std::shared_ptr<StatusService::Request> request,
      std::shared_ptr<StatusService::Response> response) {
        try {
          check_owner();
          response->status = snapshot(slot);
          response->success = true;
        } catch (const std::exception & error) {
          response->success = false;
          response->message = error.what();
        }
      }, rclcpp::ServicesQoS(), group_));
    using ListService = srv::ListComponents;
    entities.services.push_back(node_.create_service<ListService>(
      path(slot, "list_components"),
      [this, slot](const std::shared_ptr<ListService::Request> request,
      std::shared_ptr<ListService::Response> response) {
        try {
          check_owner();
          auto status = snapshot(slot);
          for (auto & component : status.components) {
            if (!request->loaded_only || !component.instance_id.empty()) {
              response->components.push_back(std::move(component));
            }
          }
          response->success = true;
        } catch (const std::exception & error) {
          response->success = false;
          response->message = error.what();
        }
      }, rclcpp::ServicesQoS(), group_));
  }

  template<class Service, class Invoke>
  void add_command(const std::string & slot, const std::string & action, Invoke invoke)
  {
    // This common response handling does not change the distinct ROS service types.
    using Request = typename Service::Request;
    using Response = typename Service::Response;
    slots_.at(slot).services.push_back(node_.create_service<Service>(
      path(slot, action),
      [this, slot, action, invoke = std::move(invoke)](
        const std::shared_ptr<Request> request, std::shared_ptr<Response> response) {
        rpp::CallbackResult result;
        try {
          check_owner();
          result = invoke(*request);
        } catch (const std::exception & error) {
          result = rpp::CallbackResult::failure(error.what());
        } catch (...) {
          result = rpp::CallbackResult::failure("Unknown slot-operation exception");
        }
        response->success = result.success;
        response->message = result.message;
        if (!result.success) {
          RCLCPP_WARN(node_.get_logger(), "%s/%s: %s", slot.c_str(),
            action.c_str(), result.message.c_str());
        }
        // Publication failure must not rewrite a completed operation's result.
        if (std::this_thread::get_id() == owner_ && !closed_) {publish_safely(slot);}
      }, rclcpp::ServicesQoS(), group_));
  }

  void publish_safely(const std::string & slot)
  {
    try {
      publish_status(slot);
    } catch (const std::exception & error) {
      RCLCPP_ERROR(node_.get_logger(), "Cannot publish status for %s: %s",
        slot.c_str(), error.what());
    }
  }

  void tick()
  {
    for (const auto & item : slots_) {publish_safely(item.first);}
  }

  rclcpp::Node & node_;
  Runtime & runtime_;
  SlotManagerRosOptions options_;
  std::thread::id owner_;
  bool closed_ = false;
  rclcpp::CallbackGroup::SharedPtr group_;
  std::map<std::string, SlotEntities> slots_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace rpp_ros2
