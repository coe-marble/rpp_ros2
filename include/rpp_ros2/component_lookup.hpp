#pragma once

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <nlohmann/json.hpp>
#include <rpp_cpp/rpp_paths.hpp>

namespace rpp_ros2
{

/// Resolve a saved component by ID or unique name; no registry/workspace files are changed.
inline std::string resolve_library_component(
  const std::string & library_name, const std::string & component)
{
  namespace fs = std::filesystem;
  if (component.empty()) {
    throw std::invalid_argument("A component name or ID is required with library_name");
  }
  if (library_name.empty() || library_name == "." || library_name == ".." ||
    fs::path(library_name).filename().string() != library_name)
  {
    throw std::invalid_argument("library_name must be a registered library name");
  }
  const auto read = [](const fs::path & path) {
      std::ifstream stream(path);
      if (!stream) {throw std::runtime_error("Cannot read component metadata: " + path.string());}
      return nlohmann::json::parse(stream);
    };
  // Registry libraries can be copied directories or linked-source JSON entries.
  // Use the RPP path API so RPP_HOME follows the application's environment.
  const fs::path libraries(rpp::get_app_libraries_dir());
  auto library = libraries / library_name;
  if (!fs::is_directory(library)) {
    const auto link = libraries / (library_name + ".json");
    if (!fs::is_regular_file(link)) {
      throw std::runtime_error("Library '" + library_name + "' is not registered");
    }
    library = read(link).at("Path").get<std::string>();
  }
  const auto parts = library / ".rppws" / "parts";
  if (!fs::is_directory(parts)) {
    throw std::runtime_error("Library '" + library_name + "' has no saved component workspace");
  }
  std::vector<fs::path> descriptions;
  for (const auto & entry : fs::recursive_directory_iterator(parts)) {
    if (entry.is_regular_file() && entry.path().filename() == "description.json") {
      descriptions.push_back(entry.path());
    }
  }
  std::sort(descriptions.begin(), descriptions.end());
  using Record = std::pair<fs::path, nlohmann::json>;
  std::vector<Record> records;
  for (const auto & path : descriptions) {records.emplace_back(path.parent_path(), read(path));}
  const auto matches = [&](const std::string & field, const std::string & value) {
      std::vector<const Record *> found;
      for (const auto & record : records) {
        if (record.second.value(field, std::string{}) == value) {found.push_back(&record);}
      }
      return found;
    };
  const auto unique = [&](const std::vector<const Record *> & found, const std::string & selector) {
      if (found.size() != 1) {
        throw std::runtime_error("Component '" + selector + "' in library '" + library_name +
                "' matched " + std::to_string(found.size()) +
                " saved components; use a unique name or ID");
      }
      return found.front();
    };
  auto found = matches("Id", component);
  const auto * selected = unique(found.empty() ? matches("Name", component) : found, component);
  if (selected->second.contains("LinkedComponentId")) {
    const auto target = selected->second.at("LinkedComponentId").get<std::string>();
    selected = unique(matches("Id", target), target);
    if (selected->second.contains("LinkedComponentId")) {
      throw std::runtime_error("Doubly linked components are not supported");
    }
  }
  return fs::canonical(selected->first).string();
}

}  // namespace rpp_ros2
