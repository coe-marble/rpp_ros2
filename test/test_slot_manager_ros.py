"""Exercise slot-specific ROS callbacks through the real Python runtime facade."""

from itertools import count
from threading import Thread
from types import SimpleNamespace

from builtin_interfaces.msg import Time
import pytest
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rpp_py.component_manager import ComponentManager
from rpp_py.lifecycle import CallbackResult, DeactivationResult, LifecycleError, LifecycleState
from rpp_py.runtime import ScriptRuntime

from rpp_ros2 import SlotManagerRos
import rpp_ros2.slot_manager_ros as ros_module
from rpp_ros2.msg import ComponentStatus
from rpp_ros2.srv import (
    ActivateComponent, CleanupComponent, ConfigureComponent, DeactivateComponent,
    GetSlotStatus, ListComponents, LoadComponent, ReloadComponent, SwitchComponents, UnloadComponent,
)


COMMANDS = {
    "configure": ConfigureComponent, "activate": ActivateComponent,
    "deactivate": DeactivateComponent, "cleanup": CleanupComponent,
    "load": LoadComponent, "unload": UnloadComponent,
    "reload": ReloadComponent, "switch": SwitchComponents,
}


class PluginContext:
    _plugin_type = "tests::Plugin"

    def __init__(self, name):
        self._instance_id = name
        self._instance = SimpleNamespace(HANDOVER_STATE_TYPE=type(None))
        self.events = []
        self.active = False
        self.configure_result = CallbackResult()

    def _bind(self, _manager, _name):
        pass

    def _configure(self):
        self.events.append("configure")
        return self.configure_result

    def _activate(self, state):
        self.events.append(("activate", state))
        self.active = True
        return CallbackResult()

    def _deactivate(self):
        self.events.append("deactivate")
        self.active = False
        return DeactivationResult()

    def _cleanup(self):
        self.events.append("cleanup")
        return CallbackResult()

    def _has_active_work(self):
        return self.active

    def _deactivate_descendants_for_shutdown(self, *, timeout=None):
        return CallbackResult()

    def _close(self):
        self.events.append("close")


class ScriptContext:
    def __init__(self, slots):
        self.slots = slots

    def list_subcomponents(self):
        return list(self.slots)

    def _slot(self, name):
        if name not in self.slots:
            raise LifecycleError(f"Unknown slot {name!r}")
        return self.slots[name]


class Node:
    """Replace ROS entity creation; callbacks still use generated ROS messages."""

    def __init__(self):
        self.default_callback_group = MutuallyExclusiveCallbackGroup()
        self.services = {}
        self.publishers = {}
        self.errors = []
        self.warnings = []

    def create_publisher(self, _message_type, name, qos):
        publisher = SimpleNamespace(name=name, qos=qos, messages=[])
        publisher.publish = publisher.messages.append
        self.publishers[name] = publisher
        return publisher

    def create_service(self, service_type, name, callback, *, callback_group):
        service = SimpleNamespace(name=name, callback=callback, service_type=service_type,
                                  group=callback_group)
        self.services[name] = service
        return service

    def destroy_service(self, service):
        del self.services[service.name]

    def destroy_publisher(self, publisher):
        del self.publishers[publisher.name]

    def get_clock(self):
        return SimpleNamespace(now=lambda: SimpleNamespace(to_msg=lambda: Time(sec=42)))

    def get_logger(self):
        return SimpleNamespace(error=self.errors.append, warning=self.warnings.append)


@pytest.fixture
def tree():
    contexts = [PluginContext("first-id"), PluginContext("second-id")]
    ids = count()
    slots = {"first": ComponentManager(), "second": ComponentManager(),
             "models": ComponentManager(allow_dynamic_components=True,
                                        factory=lambda _: PluginContext(f"dynamic-{next(ids)}"))}
    for name, context in zip(("first", "second"), contexts):
        slots[name].adopt("primary", context, f"/{name}/component", initial=True).require_success()
    runtime = ScriptRuntime(ScriptContext(slots))
    node = Node()
    helper = SlotManagerRos(node, runtime, publish_period=0)
    yield node, helper, runtime, slots, contexts
    helper.close()


def command(node, action, slot="first", **fields):
    service_type = COMMANDS[action]
    if action != "switch":
        fields.setdefault("name", "primary")
    request = service_type.Request(**fields)
    return node.services[f"~/{slot}/{action}"].callback(request, service_type.Response())


def listed(node, slot="first", *, loaded_only=True):
    return node.services[f"~/{slot}/list_components"].callback(
        ListComponents.Request(loaded_only=loaded_only),
        ListComponents.Response())


def latest(node, slot="first"):
    return node.publishers[f"~/{slot}/status"].messages[-1]


def test_each_slot_has_distinct_services_without_changing_lifecycle(tree):
    node, _, _, slots, contexts = tree
    services = {**COMMANDS, "get_status": GetSlotStatus, "list_components": ListComponents}
    assert set(node.services) == {f"~/{slot}/{action}" for slot in slots for action in services}
    for slot in slots:
        for action, service_type in services.items():
            assert node.services[f"~/{slot}/{action}"].service_type is service_type
            assert not hasattr(service_type.Request(), "slot")
    assert all(not context.events for context in contexts)
    assert latest(node).components[0].state == ComponentStatus.UNCONFIGURED
    assert latest(node).header.stamp.sec == 42
    assert latest(node, "models").allows_dynamic_components
    assert latest(node, "models").components == []


def test_lifecycle_commands_target_the_service_slot_and_publish_resulting_states(tree):
    node, _, _, slots, contexts = tree
    for action, expected in (("configure", ComponentStatus.INACTIVE),
                             ("activate", ComponentStatus.ACTIVE),
                             ("deactivate", ComponentStatus.INACTIVE),
                             ("cleanup", ComponentStatus.UNCONFIGURED)):
        response = command(node, action)
        assert response.success, response.message
        assert latest(node).components[0].state == expected
    assert contexts[0].events == ["configure", ("activate", None), "deactivate", "cleanup"]
    assert contexts[1].events == []
    assert slots["second"].get_state("primary") is LifecycleState.UNCONFIGURED


def test_manager_failure_and_unknown_component_are_reported(tree):
    node, _, _, _, contexts = tree
    contexts[0].configure_result = CallbackResult.failure("Configuration rejected")
    response = command(node, "configure")
    assert not response.success
    assert response.message == "Configuration rejected"
    status = latest(node).components[0]
    assert status.state == ComponentStatus.ERROR
    assert not status.last_success
    assert status.last_message == response.message
    assert not command(node, "configure", name="missing").success


def test_status_observes_changes_made_through_the_runtime(tree):
    node, _, runtime, _, _ = tree
    runtime.configure_component("second", "primary").require_success()
    response = node.services["~/second/get_status"].callback(
        GetSlotStatus.Request(), GetSlotStatus.Response())
    assert response.success
    assert response.status.name == "second"
    status = response.status.components[0]
    assert status.state == ComponentStatus.INACTIVE
    assert status.instance_id == "second-id"
    assert status.component == "/second/component"


def test_list_components_identifies_inactive_switch_targets(tree):
    node, _, _, _, _ = tree
    for name in ("alpha", "beta"):
        assert command(node, "load", "models", name=name, component=f"/parts/{name}").success
        assert command(node, "configure", "models", name=name).success
    assert command(node, "activate", "models", name="alpha").success
    response = listed(node, "models")
    assert response.success
    assert {info.name: info.state for info in response.components} == {
        "alpha": ComponentStatus.ACTIVE, "beta": ComponentStatus.INACTIVE}
    assert command(node, "switch", "models", source="alpha", target="beta").success
    assert {info.name: info.state for info in listed(node, "models").components} == {
        "alpha": ComponentStatus.INACTIVE, "beta": ComponentStatus.ACTIVE}


def test_reload_and_unload_preserve_native_semantics_and_list_filtering(tree):
    node, _, _, _, _ = tree
    assert command(node, "load", "models", name="alpha", component="/parts/alpha").success
    assert command(node, "configure", "models", name="alpha").success
    assert command(node, "activate", "models", name="alpha").success
    previous_id = listed(node, "models").components[0].instance_id
    assert command(node, "reload", "models", name="alpha").success
    current = listed(node, "models").components[0]
    assert current.state == ComponentStatus.ACTIVE
    assert current.instance_id != previous_id
    assert command(node, "unload", "models", name="alpha").success
    assert listed(node, "models").components == []
    unloaded = listed(node, "models", loaded_only=False).components[0]
    assert unloaded.name == "alpha" and unloaded.state == ComponentStatus.UNLOADED
    # An empty folder reuses the previously loaded component folder.
    assert command(node, "load", "models", name="alpha").success


def test_load_uses_library_lookup_when_library_name_is_set(tree, monkeypatch):
    from unittest.mock import Mock

    node, _, runtime, _, _ = tree
    lookup = Mock(return_value="/saved/library/component")
    monkeypatch.setattr(ros_module, "resolve_library_component", lookup)
    response = command(node, "load", "models", name="chosen", library_name="Library",
                       component="SavedComponent")
    assert response.success, response.message
    lookup.assert_called_once_with("Library", "SavedComponent")
    assert runtime.get_subcomponent_statuses("models")[0].component == "/saved/library/component"


def test_load_preserves_folder_mode_without_library_lookup(tree, monkeypatch):
    from unittest.mock import Mock

    lookup = Mock(side_effect=AssertionError("Folder mode must not resolve a library"))
    monkeypatch.setattr(ros_module, "resolve_library_component", lookup)
    response = command(tree[0], "load", "models", name="chosen", component="/parts/chosen")
    assert response.success, response.message
    lookup.assert_not_called()


def test_failed_library_lookup_does_not_register_a_component(tree, monkeypatch):
    def missing(*_args):
        raise ValueError("Component not found")

    monkeypatch.setattr(ros_module, "resolve_library_component", missing)
    response = command(tree[0], "load", "models", name="chosen", library_name="Library",
                       component="missing")
    assert not response.success and response.message == "Component not found"
    assert listed(tree[0], "models", loaded_only=False).components == []


@pytest.mark.parametrize("action,fields", [
    ("load", {"component": "/parts/new"}), ("unload", {}), ("reload", {}),
    ("switch", {"source": "primary", "target": "other"}),
])
def test_dynamic_operations_do_not_bypass_static_slot_permissions(tree, action, fields):
    response = command(tree[0], action, **fields)
    assert not response.success
    assert "Dynamic component" in response.message


def test_wrong_thread_cannot_change_lifecycle(tree):
    node, _, _, _, contexts = tree
    responses = []
    worker = Thread(target=lambda: responses.append(command(node, "configure")))
    worker.start()
    worker.join(timeout=2)
    assert not worker.is_alive()
    assert not responses[0].success
    assert "construction thread" in responses[0].message
    assert contexts[0].events == []


def test_publication_failure_does_not_rewrite_operation_success(tree):
    node, helper, _, slots, _ = tree

    def fail_publish(_message):
        raise RuntimeError("Publisher unavailable")

    node.publishers["~/first/status"].publish = fail_publish
    assert command(node, "configure").success
    assert slots["first"].get_state("primary") is LifecycleState.INACTIVE
    assert "Publisher unavailable" in node.errors[-1]
    before = len(node.publishers["~/second/status"].messages)
    helper._tick()
    assert len(node.publishers["~/second/status"].messages) == before + 1


def test_close_only_releases_ros_entities(tree):
    node, helper, _, slots, _ = tree
    assert command(node, "configure").success
    assert command(node, "activate").success
    helper.close()
    helper.close()
    assert not node.services and not node.publishers
    assert slots["first"].get_state("primary") is LifecycleState.ACTIVE
    with pytest.raises(RuntimeError, match="closed"):
        helper.publish_status()


@pytest.mark.parametrize("period", [-1, float("nan"), float("inf")])
def test_invalid_timing_is_rejected_before_creating_entities(period):
    node = Node()
    with pytest.raises(ValueError):
        SlotManagerRos(node, ScriptRuntime(ScriptContext({})), publish_period=period)
    assert not node.services and not node.publishers


def test_reentrant_group_is_rejected():
    with pytest.raises(ValueError, match="mutually exclusive"):
        SlotManagerRos(Node(), ScriptRuntime(ScriptContext({})),
                       callback_group=ReentrantCallbackGroup())


def test_failed_construction_releases_partially_created_entities(monkeypatch):
    node = Node()
    create_service = node.create_service

    def fail_after_first(*args, **kwargs):
        if node.services:
            raise RuntimeError("Cannot create service")
        return create_service(*args, **kwargs)

    monkeypatch.setattr(node, "create_service", fail_after_first)
    runtime = ScriptRuntime(ScriptContext({"empty": ComponentManager()}))
    with pytest.raises(RuntimeError, match="Cannot create service"):
        SlotManagerRos(node, runtime, publish_period=0)
    assert not node.services and not node.publishers
