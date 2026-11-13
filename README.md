# rpp_ros2

ROS 2 Lyrical helpers for the public slot API of `rpp_cpp` and `rpp_py` runtimes.
The C++ helper is header-only; the Python helper is an installed module. All
message and service definitions are in this package. ROS generates and compiles
the interface support code; there is no compiled helper library.

Create `SlotManagerRos(node, runtime)` after the RPP builder returns your runtime.
The helper uses public runtime operations and retains their lifecycle validation,
activation policy, and dynamic-modification permissions. The application owns
root-runtime configuration, activation, and shutdown.

## Service layout

Every declared slot, including an empty slot, gets its own services and status
topic. For node `controller`, namespace `/robot`, and slot `filter`, the default
layout is:

```text
/robot/controller/filter/configure
/robot/controller/filter/activate
/robot/controller/filter/deactivate
/robot/controller/filter/cleanup
/robot/controller/filter/load
/robot/controller/filter/unload
/robot/controller/filter/reload
/robot/controller/filter/switch
/robot/controller/filter/list_components
/robot/controller/filter/get_status
/robot/controller/filter/status              # topic
```

The default prefix is the node's private namespace, `~`. ROS adds the node name
and its namespace. Slot names must form valid ROS namespaces. A custom prefix
can separate helpers attached to different runtimes on the same node.

Each command is a separate service with its own type. There is no slot selector
in requests: the service path identifies the slot. `name` is the registration
name inside that slot, not the plugin class name.

This helper does not authenticate service callers. Use DDS/SROS2 access-control
policy to decide which participants may call lifecycle services.

All service types below belong to `rpp_ros2/srv`:

| Service | Type | Request fields |
| --- | --- | --- |
| `configure` | `ConfigureComponent` | `name` |
| `activate` | `ActivateComponent` | `name` |
| `deactivate` | `DeactivateComponent` | `name`, `timeout_ms` |
| `cleanup` | `CleanupComponent` | `name` |
| `load` | `LoadComponent` | `name`, `library_name`, `component` |
| `unload` | `UnloadComponent` | `name`, `timeout_ms` |
| `reload` | `ReloadComponent` | `name`, `transfer_state`, `timeout_ms` |
| `switch` | `SwitchComponents` | `source`, `target`, `transfer_state`, `timeout_ms` |
| `list_components` | `ListComponents` | `loaded_only` |
| `get_status` | `GetSlotStatus` | empty |

Responses contain `success` and `message`, preserving the runtime's result.
Exceptions become failed responses. `list_components` also returns component
snapshots; `get_status` also returns a complete `SlotStatus` snapshot.

`timeout_ms` defaults to 1000; zero requests a nonblocking guarded-call drain.
This bounds draining active calls, not the execution of plugin lifecycle
callbacks. `switch` defaults to `transfer_state: true`; `reload` defaults to
`false`. The runtime checks compatibility before transferring typed handover
state. Separate deactivate/activate requests do not transport handover state.

Dynamic load, unload, reload, and switching require the builder to enable dynamic
modification for that slot. A request does not enable it. Reload reconstructs
the runtime from registered artifacts; it does not refresh the plugin registry
or compile plugin sources.

## Load by library or folder

When `library_name` is nonempty, `component` is a saved component's name or ID
inside that registered library's `.rppws/parts` workspace. The saved component
includes its parameters and subcomponents. Lookup prefers an exact ID and
otherwise requires a unique name; missing or ambiguous matches fail. One-level
linked saved components resolve to their concrete instance.

When `library_name` is empty, `component` is a workspace component-folder path,
as accepted by `runtime.load_component()`. An empty folder reuses the named
registration's previous folder, if one exists.

For example, register a saved UKF instance under the local name `ukf`:

```bash
ros2 service call /robot/controller/filter/load rpp_ros2/srv/LoadComponent \
  "{name: ukf, library_name: rpp_localization, component: DefaultUkf15}"
```

The equivalent folder-based request is:

```bash
ros2 service call /robot/controller/filter/load rpp_ros2/srv/LoadComponent \
  "{name: ukf, component: /workspace/.rppws/parts/plugin/component-id}"
```

Loading registers an unconfigured instance; it does not configure or activate it.
The C++ lookup reads the registered copied/linked library under `RPP_HOME`; the
Python lookup uses the library manager configured by the RPP builder. Neither
lookup initializes, refreshes, or rewrites the registry.

## List candidates and switch

Prepare a loaded target, then inspect the components in this slot:

```bash
ros2 service call /robot/controller/filter/configure rpp_ros2/srv/ConfigureComponent \
  "{name: ukf}"
ros2 service call /robot/controller/filter/list_components rpp_ros2/srv/ListComponents \
  "{}"
```

Listing defaults to `loaded_only: true`. Each entry includes its name, instance
ID, lifecycle state, transition, guarded-call count, and last operation's result.
A loaded target with `state: 2` (`ComponentStatus.INACTIVE`) is an inactive switch
candidate. An active source has `state: 3` (`ComponentStatus.ACTIVE`). The runtime
still decides whether the requested transition and handover are valid.

Switch from the active `ekf` instance to the inactive `ukf` instance:

```bash
ros2 service call /robot/controller/filter/switch rpp_ros2/srv/SwitchComponents \
  "{source: ekf, target: ukf, transfer_state: true, timeout_ms: 1000}"
```

To include unloaded registrations in a listing:

```bash
ros2 service call /robot/controller/filter/list_components rpp_ros2/srv/ListComponents \
  "{loaded_only: false}"
```

Other operations use their corresponding service types:

```bash
ros2 service call /robot/controller/filter/activate rpp_ros2/srv/ActivateComponent \
  "{name: ukf}"
ros2 service call /robot/controller/filter/reload rpp_ros2/srv/ReloadComponent \
  "{name: ukf, transfer_state: false}"
ros2 service call /robot/controller/filter/deactivate rpp_ros2/srv/DeactivateComponent \
  "{name: ukf, timeout_ms: 1000}"
ros2 service call /robot/controller/filter/cleanup rpp_ros2/srv/CleanupComponent \
  "{name: ukf}"
ros2 service call /robot/controller/filter/unload rpp_ros2/srv/UnloadComponent \
  "{name: ukf}"
```

## Observe slot status

```bash
ros2 service call /robot/controller/filter/get_status rpp_ros2/srv/GetSlotStatus \
  "{}"
ros2 topic echo /robot/controller/filter/status rpp_ros2/msg/SlotStatus \
  --qos-durability transient_local
```

Each slot topic publishes immediately, after a command on that slot, and every
second by default. It is reliable and transient-local with depth one. Snapshots
include empty slots and unloaded/error registrations, plus the slot's dynamic
permission. Component states are `UNLOADED=0`, `UNCONFIGURED=1`, `INACTIVE=2`,
`ACTIVE=3`, and `ERROR=4`. Loaded instances have a nonempty instance ID.

Periodic publication uses steady/wall time, even if simulation time is paused;
message timestamps use the node's ROS clock.

## C++

Given your ROS node, RPP builder, and build request:

```cpp
#include <rpp_ros2/slot_manager_ros.hpp>

auto runtime = builder.build_script(request);
rpp_ros2::SlotManagerRos ros_slots(*node, runtime);
rclcpp::spin(node);
ros_slots.close();
runtime.close().require_success();
```

Link the consuming target to the exported header-only target:

```cmake
find_package(rpp_ros2 REQUIRED)
target_link_libraries(my_node PRIVATE rpp_ros2::rpp_ros2)
```

`rpp_ros2::SlotManagerRosOptions` selects `namespace_prefix` (default `~`),
`publish_period` (milliseconds), and `callback_group`. Both runtime kinds and
both RPP threading policies are accepted. To permit dynamic operations, construct
your builder with its `allow_dynamic_components` option or per-slot policy.

## Python

```python
import rclpy
from rpp_ros2 import SlotManagerRos

runtime = builder.build_script(request)
ros_slots = SlotManagerRos(node, runtime)
try:
    rclpy.spin(node)
finally:
    ros_slots.close()
    runtime.close().require_success()
```

`namespace` defaults to `~`; `publish_period` is seconds and defaults to `1.0`.
For dynamic operations, construct the RPP builder with
`allow_dynamic_components=True` or a per-slot policy.

In both languages a zero publication period disables the timer; commands still
publish. `publish_status()` publishes all slots, or accepts a slot name to
publish only that slot.

## Execution and installation

Construct and spin on the runtime's lifecycle owner thread using a single-threaded
executor. Services and timers use a mutually exclusive callback group, defaulting
to the node's group. A mutually exclusive group alone does not guarantee thread
affinity in a multi-threaded executor: callbacks on another thread are rejected
before accessing the runtime. Concurrent RPP permits guarded worker calls while
lifecycle/membership operations still belong to one owner.

Keep the node and runtime alive. Do not move a C++ runtime while its helper
exists. Close/destroy the helper after its callbacks stop, before destroying the
node or runtime. Closing the helper releases ROS entities only. Root-runtime
lifecycle, deferred self-deactivation processing, and shutdown remain under
application control. Endpoints cover the slots directly under the supplied
runtime; recreate the helper if that runtime is replaced.

Install this package through ament/colcon with its ROS dependencies available.
Install the Python RPP dependencies into the Python environment used by ROS.
Lyrical's `ament_cmake_python` 2.8.8 or newer merges the handwritten module with
the generated `rpp_ros2.msg` and `rpp_ros2.srv` modules. No separate interface
package is required.
