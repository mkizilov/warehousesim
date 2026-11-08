// A directory for tests that need real files. It lives under the current working directory (the
// build tree when run by ctest), never outside it, is emptied when it is made and removed when the
// test is done, so tests leave nothing behind and do not see each other's files.
#pragma once

#include <filesystem>
#include <fstream>
#include <iterator>
#include <string>
#include <system_error>

namespace warehousesim_test {

class ScratchDir {
 public:
  // `name` must be unique among the tests that can run at the same time (use the test's name).
  explicit ScratchDir(const std::string& name)
      : path_(std::filesystem::current_path() / ("scratch_" + name)) {
    std::error_code ignored;
    std::filesystem::remove_all(path_, ignored);
    std::filesystem::create_directories(path_);
  }
  ~ScratchDir() {
    std::error_code ignored;
    std::filesystem::remove_all(path_, ignored);
  }
  ScratchDir(const ScratchDir&) = delete;
  ScratchDir& operator=(const ScratchDir&) = delete;

  [[nodiscard]] const std::filesystem::path& path() const noexcept { return path_; }
  [[nodiscard]] std::filesystem::path file(const std::string& name) const { return path_ / name; }

 private:
  std::filesystem::path path_;
};

inline void writeFile(const std::filesystem::path& path, const std::string& content) {
  std::ofstream out(path, std::ios::binary | std::ios::trunc);
  out << content;
}

inline std::string readFile(const std::filesystem::path& path) {
  std::ifstream in(path, std::ios::binary);
  return std::string(std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>());
}

}  // namespace warehousesim_test
