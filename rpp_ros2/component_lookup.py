"""Read saved components from registered libraries without modifying the registry."""

import json
from pathlib import Path

from rpp_plugin_registrator.registry_config import get_library_manager


def resolve_library_component(library_name: str, component: str) -> str:
    """Resolve a saved component's ID or unique name, preserving its configuration."""
    if not component:
        raise ValueError("A component name or ID is required with library_name")
    if not library_name or library_name in {".", ".."} or Path(library_name).name != library_name:
        raise ValueError("library_name must be a registered library name")
    # The builder has already configured the library manager. Do not initialize
    # or refresh libraries from inside a ROS service callback.
    library_path = get_library_manager().get_library_path(library_name)
    if library_path is None:
        raise ValueError(f"Library {library_name!r} is not registered")
    parts = Path(library_path) / ".rppws" / "parts"
    if not parts.is_dir():
        raise ValueError(f"Library {library_name!r} has no saved component workspace")
    records = []
    for description in sorted(parts.rglob("description.json")):
        if description.is_file():
            with description.open(encoding="utf-8") as stream:
                records.append((description.parent, json.load(stream)))

    def unique(matches, selector):
        if len(matches) != 1:
            raise ValueError(f"Component {selector!r} in library {library_name!r} "
                             f"matched {len(matches)} saved components; use a unique name or ID")
        return matches[0]

    by_id = [entry for entry in records if entry[1].get("Id") == component]
    folder, record = unique(by_id or [entry for entry in records
                                     if entry[1].get("Name") == component], component)
    if "LinkedComponentId" in record:
        target = record["LinkedComponentId"]
        folder, record = unique([entry for entry in records if entry[1].get("Id") == target], target)
        if "LinkedComponentId" in record:
            raise ValueError("Doubly linked components are not supported")
    return str(folder.resolve())
