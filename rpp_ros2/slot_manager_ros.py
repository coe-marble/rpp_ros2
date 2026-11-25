"""ROS services for the public slot API of an existing RPP runtime."""

from __future__ import annotations

import math
from copy import deepcopy
from dataclasses import dataclass
from threading import get_ident
from typing import Callable

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


@dataclass(frozen=True)
class SlotOperation:
    """A successful lifecycle operation completed by :class:`SlotManagerRos`."""

    kind: str
    slot: str
    source: str = ""
    target: str = ""
    deactivate: tuple[str, ...] = ()
    activate: tuple[str, ...] = ()


class SlotManagerRos:
    """Expose one service set and status topic per slot of a built runtime.

    Keep the node and runtime alive, and spin a SingleThreadedExecutor on the
    construction thread. Other lifecycle/membership operations must use that
    thread too. close() releases ROS entities without changing component states.
    The callback group defaults to the node's group.
    """

    def __init__(self, node: Node, runtime: ComponentRuntime | ScriptRuntime, *,
                 namespace="~", publish_period=1.0, callback_group=None,
                 after_success: Callable[[SlotOperation], CallbackResult] | None = None,
                 make_handover_state: Callable[[type | None], object] | None = None):
        if not namespace:
            raise ValueError("A nonempty ROS namespace is required")
        if not math.isfinite(publish_period) or publish_period < 0:
            raise ValueError("publish_period must be finite and non-negative")
        group = callback_group if callback_group is not None else node.default_callback_group
        self._node = node
        self._runtime = runtime
        self._namespace = namespace.rstrip("/")
        self._group = group
        self._owner = get_ident()
        self._after_success = after_success
        self._make_handover_state = make_handover_state
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
             lambda req, operation: runtime.configure_component(name, req.name),
             lambda req: SlotOperation("configure", name, target=req.name)),
            ("activate", ActivateComponent,
             lambda req, operation: runtime.activate_component(
                 name, req.name, self._prepare_activation_state(name, req.name)),
             lambda req: SlotOperation("activate", name, target=req.name)),
            ("deactivate", DeactivateComponent,
             lambda req, operation: runtime.deactivate_component(
                 name, req.name, timeout=req.timeout_ms / 1000.0),
             lambda req: SlotOperation("deactivate", name, target=req.name)),
            ("cleanup", CleanupComponent,
             lambda req, operation: runtime.cleanup_component(name, req.name),
             lambda req: SlotOperation("cleanup", name, target=req.name)),
            ("load", LoadComponent,
             lambda req, operation: self._load(name, req),
             lambda req: SlotOperation("load", name, target=req.name)),
            ("unload", UnloadComponent,
             lambda req, operation: runtime.unload_component(
                 name, req.name, timeout=req.timeout_ms / 1000.0),
             lambda req: SlotOperation("unload", name, target=req.name)),
            ("reload", ReloadComponent,
             lambda req, operation: self._reload(name, req),
             lambda req: SlotOperation("reload", name, target=req.name)),
            ("switch", SwitchComponents,
             lambda req, operation: self._switch(operation, req.timeout_ms / 1000.0),
             lambda req: self._describe_switch(name, req)),
        )
        for action, service_type, invoke, describe in commands:
            entities["services"].append(self._node.create_service(
                service_type, self._path(name, action), self._command(name, action, invoke, describe),
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

    def _reload(self, slot, request):
        current = next((status for status in self._runtime.get_subcomponent_statuses(slot)
                        if status.name == request.name), None)
        state = (self._prepare_activation_state(slot, request.name)
                 if current is not None and current.state is LifecycleState.ACTIVE else None)
        return self._runtime.reload_component(
            slot, request.name, state, timeout=request.timeout_ms / 1000.0)

    def _describe_switch(self, slot, request):
        statuses = {item.instance_id: item for item in self._runtime.get_subcomponent_statuses(slot)
                    if item.instance_id}
        requested = set()
        resolved = []
        for instance_ids, expected, description in (
            (request.deactivate, LifecycleState.ACTIVE, "active"),
            (request.activate, LifecycleState.INACTIVE, "configured and inactive"),
        ):
            names = []
            for instance_id in instance_ids:
                if not instance_id or instance_id in requested:
                    raise ValueError(f"Empty or repeated switch instance ID {instance_id!r}")
                requested.add(instance_id)
                status = statuses.get(instance_id)
                if status is None:
                    raise ValueError(f"Unknown component instance {instance_id!r} in slot {slot!r}")
                if status.transition or status.state is not expected:
                    raise ValueError(f"Instance {instance_id!r} must be {description}")
                names.append(status.name)
            resolved.append(tuple(names))
        return SlotOperation("switch", slot, deactivate=resolved[0], activate=resolved[1])

    def _prepare_activation_state(self, slot, name):
        if self._make_handover_state is None:
            return None
        plugin = self._runtime.get_context().get_subcomponent_context(slot, name).get_instance()
        state = self._make_handover_state(plugin.HANDOVER_STATE_TYPE)
        plugin._validate_state(state)
        return deepcopy(state)

    def _switch(self, operation, timeout):
        # Prepare all payloads before stopping any outgoing component.
        runtime = self._runtime
        slot = operation.slot
        states = [self._prepare_activation_state(slot, name) for name in operation.activate]
        deactivated = []
        activated = []

        def failed(action, name, result):
            message = f"Cannot {action} {name!r}: {result.message}"
            if deactivated:
                message += f". Deactivated [{', '.join(deactivated)}]"
            if activated:
                message += f". Activated [{', '.join(activated)}]"
            return CallbackResult.failure(message)

        for name in operation.deactivate:
            result = runtime.deactivate_component(slot, name, timeout=timeout)
            if not result.success:
                return failed("deactivate", name, result)
            deactivated.append(name)
        for name, state in zip(operation.activate, states):
            result = runtime.activate_component(slot, name, state)
            if not result.success:
                return failed("activate", name, result)
            activated.append(name)
        return CallbackResult()

    def _command(self, slot, action, invoke, describe):
        def callback(request, response):
            try:
                self._check_owner()
                operation = describe(request)
                result = invoke(request, operation)
                if result.success and self._after_success is not None:
                    synchronized = self._after_success(operation)
                    if not synchronized.success:
                        result = CallbackResult.failure(
                            "Slot operation succeeded but its callback failed: " +
                            synchronized.message)
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
