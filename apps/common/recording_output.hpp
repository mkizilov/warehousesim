#pragma once

#include <filesystem>
#include <iostream>
#include <string>
#include <system_error>

namespace warehousesim::apps {

// `--record FILE` writes its file when the run is over. Whether that can work is found out when the
// program starts, not at the end of a long run: FILE must not be a folder, and its folder must
// exist.
[[nodiscard]] inline bool recordingFileUsable(const std::string& file, std::ostream& err) {
  namespace fs = std::filesystem;
  const fs::path path(file);
  std::error_code ignored;
  if (path.filename().empty() || fs::is_directory(path, ignored)) {
    err << "error: --record needs a file name, not '" << file << "'\n";
    return false;
  }
  const fs::path folder = path.parent_path();
  if (!folder.empty() && !fs::is_directory(folder, ignored)) {
    err << "error: the folder of --record " << file << " does not exist\n";
    return false;
  }
  return true;
}

}  // namespace warehousesim::apps
