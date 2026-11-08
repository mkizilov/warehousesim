#include "warehousesim/editor/editor_files.hpp"

#include <utility>

#include "warehousesim/mapio/map_file.hpp"
#include "warehousesim/world/layouts.hpp"

namespace warehousesim::editor {

namespace {

FileOutcome failure(std::string message) { return {false, std::move(message)}; }

std::string size(const world::Warehouse& layout) {
  return std::to_string(layout.width()) + " x " + std::to_string(layout.height());
}

}  // namespace

std::string readiness(const std::vector<world::LayoutIssue>& issues) {
  if (issues.empty()) {
    return "ready for a simulation";
  }
  for (const world::LayoutIssue& issue : issues) {
    if (issue.blocksSimulation) {
      return "not ready: " + issue.message;
    }
  }
  return "ready, but " + issues.front().message;
}

std::string readiness(const MapEditor& editor) { return readiness(editor.issues()); }

FileOutcome saveMap(MapEditor& editor, const std::filesystem::path& path) {
  if (const auto problem = mapio::saveMapFile(path, {editor.name(), editor.layout()})) {
    return failure("not saved: " + problem->text());
  }
  editor.markSaved();
  return {true,
          "saved " + path.string() + " (" + size(editor.layout()) + "); " + readiness(editor)};
}

FileOutcome loadMap(MapEditor& editor, const std::filesystem::path& path) {
  mapio::MapLoad load = mapio::loadMapFile(path);
  if (!load.ok()) {
    std::string message = "not loaded: " + load.problems.front().text();
    if (load.problems.size() > 1) {
      message += " (and " + std::to_string(load.problems.size() - 1) + " more)";
    }
    return failure(std::move(message));
  }
  mapio::MapDocument& document = *load.document;
  if (document.name.empty()) {
    document.name = path.stem().string().substr(0, mapio::kMaxNameBytes);
  }
  editor.replace(std::move(document.layout), std::move(document.name));
  return {true, "loaded " + path.string() + " (" + size(editor.layout()) + ")"};
}

FileOutcome loadPredefined(MapEditor& editor, std::string_view name) {
  auto layout = world::makePredefinedWarehouse(name);
  if (!layout) {
    std::string known;
    for (const world::LayoutInfo& info : world::predefinedLayouts()) {
      known += known.empty() ? "" : ", ";
      known += info.name;
    }
    return failure("no predefined layout called \"" + std::string(name) +
                   "\"; there are: " + known);
  }
  editor.replace(*std::move(layout), std::string(name));
  return {true,
          "loaded the predefined layout " + std::string(name) + " (" + size(editor.layout()) + ")"};
}

}  // namespace warehousesim::editor
