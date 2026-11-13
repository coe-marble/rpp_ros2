"""Saved-component discovery is read-only and rejects ambiguous names."""

import json
from types import SimpleNamespace

import pytest

from rpp_ros2 import component_lookup


@pytest.fixture
def library(tmp_path, monkeypatch):
    root = tmp_path / "Library"
    (root / ".rppws/parts").mkdir(parents=True)
    manager = SimpleNamespace(get_library_path=lambda name: str(root) if name == "Library" else None)
    monkeypatch.setattr(component_lookup, "get_library_manager", lambda: manager)
    return root


def saved(root, identifier, name, *, linked_to=None):
    folder = root / ".rppws/parts/plugin" / identifier
    folder.mkdir(parents=True)
    record = {"Id": identifier, "Name": name}
    if linked_to is not None:
        record["LinkedComponentId"] = linked_to
    (folder / "description.json").write_text(json.dumps(record), encoding="utf-8")
    return str(folder.resolve())


def test_unique_name_and_id_find_the_same_saved_instance(library):
    expected = saved(library, "first-id", "SavedComponent")
    assert component_lookup.resolve_library_component("Library", "SavedComponent") == expected
    assert component_lookup.resolve_library_component("Library", "first-id") == expected


def test_ambiguous_names_require_an_id(library):
    saved(library, "first-id", "SameName")
    second = saved(library, "second-id", "SameName")
    with pytest.raises(ValueError, match="matched 2"):
        component_lookup.resolve_library_component("Library", "SameName")
    assert component_lookup.resolve_library_component("Library", "second-id") == second


def test_unknown_library_and_component_are_reported_without_writes(library):
    saved(library, "first-id", "SavedComponent")
    before = {path: path.read_bytes() for path in library.rglob("*") if path.is_file()}
    with pytest.raises(ValueError, match="not registered"):
        component_lookup.resolve_library_component("missing", "SavedComponent")
    with pytest.raises(ValueError, match="matched 0"):
        component_lookup.resolve_library_component("Library", "missing")
    assert before == {path: path.read_bytes() for path in library.rglob("*") if path.is_file()}


def test_linked_component_resolves_to_the_concrete_instance(library):
    expected = saved(library, "first-id", "SavedComponent")
    saved(library, "alias-id", "Alias", linked_to="first-id")
    assert component_lookup.resolve_library_component("Library", "Alias") == expected


def test_doubly_linked_components_are_rejected(library):
    saved(library, "first-id", "SavedComponent")
    saved(library, "alias-id", "Alias", linked_to="first-id")
    saved(library, "other-id", "Other", linked_to="alias-id")
    with pytest.raises(ValueError, match="Doubly linked"):
        component_lookup.resolve_library_component("Library", "Other")
