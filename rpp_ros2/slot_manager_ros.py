"""ROS services for the public slot API of an existing RPP runtime."""

from __future__ import annotations

import math
from threading import get_ident

from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rpp_py.lifecycle import CallbackResult, LifecycleState
from rpp_py.runtime import ComponentRuntime, ScriptRuntime

from rpp_ros2.msg import ComponentStatus, SlotStatus
from rpp_ros2.component_lookup import resolve_library_component
from rpp_ros2.srv import (
    ActivateComponent, CleanupComponent, ConfigureComponent, DeactivateComponent,
    GetSlotStatus, ListComponents, LoadComponent, ReloadComponent, SwitchComponents, UnloadComponent,
)


_STATES = {
    LifecycleState.UNLOADED: ComponentStatus.UNLOADED,
    LifecycleState.UNCONFIGURED: ComponentStatus.UNCONFIGURED,
    LifecycleState.INACTIVE: ComponentStatus.INACTIVE,
    LifecycleState.ACTIVE: ComponentStatus.ACTIVE,
    LifecycleState.ERROR: ComponentStatus.ERROR,
}


class SlotManagerRos:
    """Expose one service set and status topic per slot of a built runtime.

    Keep the node and runtime alive, and spin a SingleThreadedExecutor on the
    construction thread. Other lifecycle/membership operations must use that
    thread too. close() releases ROS entities without changing component states.
    """

    def __init__(self, node: Node, runtime: ComponentRuntime | ScriptRuntime, *,
                 namespace="~", publish_period=1.0, callback_group=None):
        if not namespace:
            raise ValueError("A nonempty ROS namespace is required")
        if not math.isfinite(publish_period) or publish_period < 0:
            raise ValueError("publish_period must be finite and non-negative")
        group = callback_group if callback_group is not None else node.default_callback_group
        if not isinstance(group, MutuallyExclusiveCallbackGroup):
            raise ValueError("Slot lifecycle callbacks require a mutually exclusive callback group")
        self._node = node
        self._runtime = runtime
        self._namespace = namespace.rstrip("/")
        self._group = group
        self._owner = get_ident()
        self._closed = False
        self._slots = {}
        self._timer = None
        try:
            for name in runtime.get_context().list_subcomponents():
                self._add_slot(name)
            self.publish_status()
            if publish_period:
                # Publish even while simulation time is paused; stamps still use ROS time.
                self._wall_clock = Clock(clock_type=ClockType.STEADY_TIME)
                self._timer = node.create_timer(publish_period, self._tick,
                                               callback_group=group, clock=self._wall_clock)
        except Exception:
            self.close()
            raise

    def _path(self, slot, action):
        return f"{self._namespace}/{slot}/{action}"

    def _check_owner(self):
        if get_ident() != self._owner:
            raise RuntimeError("Run slot operations on the construction thread "
                               "with a SingleThreadedExecutor")
        if self._closed:
            raise RuntimeError("The ROS slot-manager helper is closed")

    def _add_slot(self, name):
        entities = {"services": [], "publisher": None}
        self._slots[name] = entities
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        entities["publisher"] = self._node.create_publisher(
            SlotStatus, self._path(name, "status"), qos)
        runtime = self._runtime
        commands = (
            ("configure", ConfigureComponent,
             lambda req: runtime.configure_component(name, req.name)),
            ("activate", ActivateComponent,
             lambda req: runtime.activate_component(name, req.name)),
            ("deactivate", DeactivateComponent,
             lambda req: runtime.deactivate_component(
                 name, req.name, timeout=req.timeout_ms / 1000.0).result),
            ("cleanup", CleanupComponent,
             lambda req: runtime.cleanup_component(name, req.name)),
            ("load", LoadComponent,
             lambda req: self._load(name, req)),
            ("unload", UnloadComponent,
             lambda req: runtime.unload_component(
                 name, req.name, timeout=req.timeout_ms / 1000.0)),
            ("reload", ReloadComponent,
             lambda req: runtime.reload_component(
                 name, req.name, transfer_state=req.transfer_state,
                 timeout=req.timeout_ms / 1000.0)),
            ("switch", SwitchComponents,
             lambda req: runtime.switch_component(
                 name, req.source, req.target, transfer_state=req.transfer_state,
                 timeout=req.timeout_ms / 1000.0)),
        )
        for action, service_type, invoke in commands:
            entities["services"].append(self._node.create_service(
                service_type, self._path(name, action), self._command(name, action, invoke),
                callback_group=self._group))
        entities["services"].append(self._node.create_service(
            GetSlotStatus, self._path(name, "get_status"), self._get_status(name),
            callback_group=self._group))
        entities["services"].append(self._node.create_service(
            ListComponents, self._path(name, "list_components"), self._list_components(name),
            callback_group=self._group))

    def _load(self, slot, request):
        component = (resolve_library_component(request.library_name, request.component)
                     if request.library_name else request.component or None)
        return self._runtime.load_component(slot, request.name, component)

    def _command(self, slot, action, invoke):
        def callback(request, response):
            try:
                self._check_owner()
                result = invoke(request)
            except Exception as error:
                result = CallbackResult.failure(str(error))
            response.success = result.success
            response.message = result.message
            if not result.success:
                self._node.get_logger().warning(f"{slot}/{action}: {result.message}")
            # A publication failure cannot rewrite a completed operation's result.
            if get_ident() == self._owner and not self._closed:
                self._publish_safely(slot)
            return response
        return callback

    def _snapshot(self, name):
        self._check_owner()
        slot = SlotStatus()
        slot.header.stamp = self._node.get_clock().now().to_msg()
        slot.name = name
        slot.allows_dynamic_components = self._runtime.allows_dynamic_components(name)
        for info in self._runtime.get_subcomponent_statuses(name):
            status = ComponentStatus()
            status.name = info.name
            status.component = info.component
            status.instance_id = info.instance_id
            status.state = _STATES[info.state]
            status.transition = info.transition or ""
            status.callable = info.callable
            status.in_flight = info.in_flight
            status.last_success = info.last_result.success
            status.last_message = info.last_result.message
            slot.components.append(status)
        return slot

    def _get_status(self, slot):
        def callback(request, response):
            try:
                self._check_owner()
                response.status = self._snapshot(slot)
                response.success = True
            except Exception as error:
                response.success = False
                response.message = str(error)
            return response
        return callback

    def publish_status(self, slot=None):
        """Publish one slot or all declared slots, including empty slots."""
        self._check_owner()
        names = self._slots if slot is None else (slot,)
        for name in names:
            self._slots[name]["publisher"].publish(self._snapshot(name))

    def _list_components(self, slot):
        def callback(request, response):
            try:
                self._check_owner()
                snapshot = self._snapshot(slot)
                response.components = [info for info in snapshot.components
                                       if not request.loaded_only or info.instance_id]
                response.success = True
            except Exception as error:
                response.success = False
                response.message = str(error)
            return response
        return callback

    def _publish_safely(self, slot):
        try:
            self.publish_status(slot)
        except Exception as error:
            self._node.get_logger().error(f"Cannot publish status for {slot}: {error}")

    def _tick(self):
        for slot in self._slots:
            self._publish_safely(slot)

    def close(self):
        """Release ROS entities without configuring, activating or closing the runtime."""
        if self._closed:
            return
        self._check_owner()
        self._closed = True
        if self._timer is not None:
            self._node.destroy_timer(self._timer)
        for entities in self._slots.values():
            for service in entities["services"]:
                self._node.destroy_service(service)
            if entities["publisher"] is not None:
                self._node.destroy_publisher(entities["publisher"])
        self._slots.clear()
