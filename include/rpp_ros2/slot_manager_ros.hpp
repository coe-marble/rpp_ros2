#pragma once

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <exception>
#include <functional>
#include <map>
#include <memory>
#include <set>
#include <stdexcept>
#include <string>
#include <thread>
#include <typeinfo>
#include <utility>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rpp_cpp/runtime.hpp>
#include <rpp_cpp/slot_manager.hpp>
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
  /// Defaults to the node's group. Run callbacks on the helper's construction thread.
  rclcpp::CallbackGroup::SharedPtr callback_group;
};

/// A lifecycle operation exposed through SlotManagerRos callbacks.
struct SlotOperation
{
  enum class Kind {Configure, Activate, Deactivate, Cleanup, Load, Unload, Reload, Switch};

  Kind kind;
  std::string slot;
  std::string source;
  std::string target;
  rpp::SwitchRequest switch_request{};
};

/// Optional observer for successful ROS lifecycle operations.
///
/// The observer runs synchronously after the runtime operation and before its
/// service response. It may update application state, but must not perform a
/// further lifecycle operation on the same runtime.
struct SlotManagerRosCallbacks
{
  std::function<rpp::CallbackResult(const SlotOperation &)> after_success;
  // Optional application quiescence boundary. Hooks must support nested access;
  // before_access may wait on management, after_access must not throw.
  std::function<void()> before_access;
  std::function<void()> after_access;
  std::function<void(const SlotOperation &)> before_operation;
  // Serve the last status while the runtime is busy.
  std::function<bool()> use_cached_status;
  // The application supplies activation state in-process. ROS carries only instance IDs.
  std::function<rpp::HandoverState(
    const std::type_info &, const std::string & slot, const std::string & target)> make_handover_state;
};

/**
 * One ROS service set and status topic per slot of a built RPP runtime.
 * The node and runtime must outlive this helper; do not move the runtime while
 * this helper refers to it. Run ROS callbacks and lifecycle operations on the
 * construction thread.
 * Destroy/close after callbacks stop. Root-runtime ownership remains with the app.
 */
template<class Runtime>
class SlotManagerRos
{
  class RuntimeAccess
  {
    const SlotManagerRosCallbacks & callbacks_;
  public:
    explicit RuntimeAccess(const SlotManagerRosCallbacks & callbacks) : callbacks_(callbacks)
    {if (callbacks_.before_access) {callbacks_.before_access();}}
    ~RuntimeAccess() {if (callbacks_.after_access) {callbacks_.after_access();}}
  };
  using StatusService = rpp_ros2::srv::GetSlotStatus;
  struct SlotEntities
  {
    rclcpp::Publisher<rpp_ros2::msg::SlotStatus>::SharedPtr publisher;
    std::vector<rclcpp::ServiceBase::SharedPtr> services;
  };

public:
  SlotManagerRos(
    rclcpp::Node & node, Runtime & runtime, SlotManagerRosOptions options = {},
    SlotManagerRosCallbacks callbacks = {})
  : node_(node), runtime_(runtime), options_(std::move(options)),
    callbacks_(std::move(callbacks)),
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
    if (callbacks_.use_cached_status && callbacks_.use_cached_status()) {
      return cached_status_.at(name);
    }
    RuntimeAccess access(callbacks_);
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
    if (callbacks_.use_cached_status) {cached_status_.insert_or_assign(name, slot);}
    return slot;
  }

  void add_slot(const std::string & slot)
  {
    auto & entities = slots_[slot];
    entities.publisher = node_.create_publisher<rpp_ros2::msg::SlotStatus>(
      path(slot, "status"), rclcpp::QoS(1).reliable().transient_local());
    add_command<srv::ConfigureComponent>(slot, "configure", [this, slot](const auto & req, const auto &) {
        return runtime_.configure_component(slot, req.name);
      }, [slot](const auto & req) {return SlotOperation{
          SlotOperation::Kind::Configure, slot, {}, req.name};});
    add_command<srv::ActivateComponent>(slot, "activate", [this, slot](const auto & req, const auto &) {
        return runtime_.activate_component(slot, req.name, prepare_activation_state(slot, req.name));
      }, [slot](const auto & req) {return SlotOperation{
          SlotOperation::Kind::Activate, slot, {}, req.name};});
    add_command<srv::DeactivateComponent>(slot, "deactivate", [this, slot](const auto & req, const auto &) {
        return runtime_.deactivate_component(
          slot, req.name, std::chrono::milliseconds(req.timeout_ms));
      }, [slot](const auto & req) {return SlotOperation{
          SlotOperation::Kind::Deactivate, slot, {}, req.name};});
    add_command<srv::CleanupComponent>(slot, "cleanup", [this, slot](const auto & req, const auto &) {
        return runtime_.cleanup_component(slot, req.name);
      }, [slot](const auto & req) {return SlotOperation{
          SlotOperation::Kind::Cleanup, slot, {}, req.name};});
    add_command<srv::LoadComponent>(slot, "load", [this, slot](const auto & req, const auto &) {
        const auto component = req.library_name.empty() ? req.component :
          resolve_library_component(req.library_name, req.component);
        return runtime_.load_component(slot, req.name, component);
      }, [slot](const auto & req) {return SlotOperation{
          SlotOperation::Kind::Load, slot, {}, req.name};});
    add_command<srv::UnloadComponent>(slot, "unload", [this, slot](const auto & req, const auto &) {
        return runtime_.unload_component(slot, req.name, std::chrono::milliseconds(req.timeout_ms));
      }, [slot](const auto & req) {return SlotOperation{
          SlotOperation::Kind::Unload, slot, {}, req.name};});
    add_command<srv::ReloadComponent>(slot, "reload", [this, slot](const auto & req, const auto &) {
        return reload_slot(slot, req.name, std::chrono::milliseconds(req.timeout_ms));
      }, [slot](const auto & req) {return SlotOperation{
          SlotOperation::Kind::Reload, slot, {}, req.name};});
    add_command<srv::SwitchComponents>(slot, "switch", [this, slot](const auto & req, const auto & operation) {
        return switch_slot(operation, std::chrono::milliseconds(req.timeout_ms));
      }, [this, slot](const auto & req) {
        return describe_switch(slot, req);
      });
    entities.services.push_back(node_.create_service<StatusService>(
      path(slot, "get_status"),
      [this, slot](const std::shared_ptr<StatusService::Request> request,
      std::shared_ptr<StatusService::Response> response) {
        try {
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

  rpp::CallbackResult reload_slot(
    const std::string & slot, const std::string & name, std::chrono::milliseconds timeout)
  {
    if (!runtime_.allows_dynamic_components(slot)) {
      return runtime_.reload_component(slot, name, {}, timeout);
    }
    const auto statuses = runtime_.get_subcomponent_statuses(slot);
    const auto current = std::find_if(statuses.begin(), statuses.end(),
        [&](const auto & status) {return status.name == name;});
    const bool was_active = current != statuses.end() && current->state == rpp::LifecycleState::ACTIVE;
    if (!was_active) {return runtime_.reload_component(slot, name, {}, timeout);}

    auto state = prepare_activation_state(slot, name);
    auto stopped = runtime_.deactivate_component(slot, name, timeout);
    if (!stopped.success) {return stopped;}
    auto reloaded = runtime_.reload_component(slot, name, {}, timeout);
    if (!reloaded.success) {return reloaded;}
    // Reuse application activation preparation for the newly configured instance.
    // Reference generators, for example, must bind their horizon before activation.
    if (callbacks_.before_operation) {
      callbacks_.before_operation({SlotOperation::Kind::Activate, slot, {}, name});
    }
    return runtime_.activate_component(slot, name, std::move(state));
  }

  SlotOperation describe_switch(
    const std::string & slot, const srv::SwitchComponents::Request & request)
  {
    SlotOperation operation{SlotOperation::Kind::Switch, slot, {}, {}};
    const auto statuses = runtime_.get_subcomponent_statuses(slot);
    std::set<std::string> requested_ids;
    const auto resolve = [&](const auto & ids, auto & names,
        rpp::LifecycleState expected, const char * state) {
        for (const auto & id : ids) {
          if (id.empty() || !requested_ids.insert(id).second) {
            throw std::invalid_argument("Empty or repeated switch instance ID '" + id + "'");
          }
          const auto status = std::find_if(statuses.begin(), statuses.end(),
              [&](const auto & item) {return item.instance_id == id;});
          if (status == statuses.end()) {
            throw std::invalid_argument("Unknown component instance '" + id + "' in slot '" + slot + "'");
          }
          if (status->transition || status->state != expected) {
            throw std::invalid_argument("Instance '" + id + "' must be " + state);
          }
          names.push_back(status->name);
        }
      };
    // Application callbacks use registration names after the wire IDs are resolved.
    resolve(request.deactivate, operation.switch_request.deactivate, rpp::LifecycleState::ACTIVE, "active");
    resolve(request.activate, operation.switch_request.activate, rpp::LifecycleState::INACTIVE, "configured and inactive");
    return operation;
  }

  rpp::HandoverState prepare_activation_state(const std::string & slot, const std::string & name)
  {
    if (!callbacks_.make_handover_state) {return {};}
    const auto plugin = runtime_.get_context().get_subcomponent_context(slot, name)
      .template get_instance<rpp::Plugin>();
    auto state = callbacks_.make_handover_state(plugin->handover_state_type(), slot, name);
    plugin->validate_handover_state(state).require_success("Prepare activation state for '" + name + "'");
    return state;
  }

  rpp::CallbackResult switch_slot(const SlotOperation & operation, std::chrono::milliseconds timeout)
  {
    const auto & request = operation.switch_request;
    // Prepare every payload before deactivating any outgoing component.
    std::vector<rpp::HandoverState> activation_states;
    activation_states.reserve(request.activate.size());
    for (const auto & name : request.activate) {
      activation_states.push_back(prepare_activation_state(operation.slot, name));
    }

    std::size_t deactivated = 0;
    std::size_t activated = 0;
    const auto failed = [&](const std::string & action, const std::string & name,
        const rpp::CallbackResult & result) {
        auto message = "Cannot " + action + " '" + name + "': " + result.message;
        const auto append = [&](const char * label, const auto & names, std::size_t count) {
            if (count == 0) {return;}
            message += label;
            for (std::size_t i = 0; i < count; ++i) {
              if (i != 0) {message += ", ";}
              message += names[i];
            }
            message += "]";
          };
        append(". Deactivated [", request.deactivate, deactivated);
        append(". Activated [", request.activate, activated);
        return rpp::CallbackResult::failure(std::move(message));
      };

    for (const auto & name : request.deactivate) {
      auto result = runtime_.deactivate_component(operation.slot, name, timeout);
      if (!result.success) {
        return failed("deactivate", name, result);
      }
      ++deactivated;
    }
    for (std::size_t i = 0; i < request.activate.size(); ++i) {
      const auto & name = request.activate[i];
      auto result = runtime_.activate_component(operation.slot, name, std::move(activation_states[i]));
      if (!result.success) {
        return failed("activate", name, result);
      }
      ++activated;
    }
    return {};
  }

  template<class Service, class Invoke, class Describe>
  void add_command(
    const std::string & slot, const std::string & action, Invoke invoke, Describe describe)
  {
    // This common response handling does not change the distinct ROS service types.
    using Request = typename Service::Request;
    using Response = typename Service::Response;
    slots_.at(slot).services.push_back(node_.create_service<Service>(
      path(slot, action),
      [this, slot, invoke = std::move(invoke), describe = std::move(describe)](
        const std::shared_ptr<Request> request, std::shared_ptr<Response> response) {
        rpp::CallbackResult result;
        try {
          check_owner();
          RuntimeAccess access(callbacks_);
          const auto operation = describe(*request);
          if (callbacks_.before_operation) {callbacks_.before_operation(operation);}
          result = invoke(*request, operation);
          if (result.success && callbacks_.after_success) {
            const auto synchronized = callbacks_.after_success(operation);
            if (!synchronized.success) {
              result = rpp::CallbackResult::failure(
                "Slot operation succeeded but its callback failed: " + synchronized.message);
            }
          }
        } catch (const std::exception & error) {
          result = rpp::CallbackResult::failure(error.what());
        } catch (...) {
          result = rpp::CallbackResult::failure("Unknown slot-operation exception");
        }
        response->success = result.success;
        response->message = result.message;
        // Publication failure must not rewrite a completed operation's result.
        if (std::this_thread::get_id() == owner_ && !closed_) {
          publish_safely(slot);
        }
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
  SlotManagerRosCallbacks callbacks_;
  mutable std::map<std::string, rpp_ros2::msg::SlotStatus> cached_status_;
  std::thread::id owner_;
  bool closed_ = false;
  rclcpp::CallbackGroup::SharedPtr group_;
  std::map<std::string, SlotEntities> slots_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace rpp_ros2
