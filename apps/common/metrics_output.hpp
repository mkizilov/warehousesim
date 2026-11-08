#pragma once

#include <filesystem>
#include <iostream>
#include <string>

#include "simulation_source.hpp"

namespace warehousesim::apps {

// What to call the warehouse of a run in the metrics files: the layout's name, or the name of the
// map file (without its folder).
[[nodiscard]] inline std::string warehouseLabel(const WarehouseSource& source) {
  if (!source.mapFile.empty()) {
    return std::filesystem::path(source.mapFile).filename().string();
  }
  return source.layout.empty() ? std::string("demo") : source.layout;
}

// `--metrics-csv BASE` names three files, BASE.runs.csv, BASE.robots.csv and BASE.series.csv. Their
// folder has to exist, and BASE must not be a folder or end in a separator: found out when the
// program starts, not when the first file is written at the end of a long run.
[[nodiscard]] inline bool metricsBaseUsable(const std::string& base, std::ostream& err) {
  namespace fs = std::filesystem;
  const fs::path path(base);
  std::error_code ignored;
  if (path.filename().empty() || fs::is_directory(path, ignored)) {
    err << "error: --metrics-csv needs a file name to start the three files' names with, not '"
        << base << "'\n";
    return false;
  }
  const fs::path folder = path.parent_path();
  if (!folder.empty() && !fs::is_directory(folder, ignored)) {
    err << "error: the folder of --metrics-csv " << base << " does not exist\n";
    return false;
  }
  return true;
}

}  // namespace warehousesim::apps
