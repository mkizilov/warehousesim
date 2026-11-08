#pragma once

#include <filesystem>
#include <string>
#include <string_view>
#include <vector>

#include "warehousesim/editor/map_editor.hpp"
#include "warehousesim/world/layout_check.hpp"

// Loading and saving for the editor: the connection between MapEditor and the map file format
// (mapio) and the predefined layouts. Each function says in `message` what happened, in words a
// status line can show.
namespace warehousesim::editor {

struct FileOutcome {
  bool ok = false;
  std::string message;
};

// Writes the map to `path` (see mapio::saveMapFile: all or nothing). On success the editor is
// marked saved. The map is saved whatever its issues are, so work in progress can be kept; the
// message says if it is not ready for a simulation.
[[nodiscard]] FileOutcome saveMap(MapEditor& editor, const std::filesystem::path& path);

// Replaces the editor's document with the map in `path`. If the file is missing, unreadable or not
// a valid map the editor is left exactly as it was and the message names the first problem. A map
// without a name in it is named after the file.
[[nodiscard]] FileOutcome loadMap(MapEditor& editor, const std::filesystem::path& path);

// Replaces the editor's document with a predefined layout (see world::predefinedLayouts), named
// after it. An unknown name leaves the editor as it was and lists the known ones.
[[nodiscard]] FileOutcome loadPredefined(MapEditor& editor, std::string_view name);

// A short summary of the issues of a map (see world::checkLayout): "ready for a simulation", "not
// ready: <first blocking issue>" or "ready, but <first warning>".
[[nodiscard]] std::string readiness(const std::vector<world::LayoutIssue>& issues);
[[nodiscard]] std::string readiness(const MapEditor& editor);

}  // namespace warehousesim::editor
