#pragma once

#include <charconv>
#include <cstdint>
#include <optional>
#include <string>
#include <string_view>
#include <system_error>

#include "warehousesim/world/layouts.hpp"

namespace warehousesim::apps {

[[nodiscard]] inline std::optional<std::uint64_t> parseUnsigned(std::string_view text) {
  std::uint64_t value = 0;
  const char* const end = text.data() + text.size();
  const auto [ptr, ec] = std::from_chars(text.data(), end, value);
  if (ec != std::errc{} || ptr != end || text.empty()) {
    return std::nullopt;
  }
  return value;
}

// If argv[index] is `name`, parses the following argument into `out`, advances `index`
// past it and returns true. Returns false when argv[index] is not `name` or the value is
// missing/invalid, in which case `failed` is set only for the latter.
inline bool readUnsignedOption(int argc, char** argv, int& index, std::string_view name,
                               std::uint64_t& out, bool& failed) {
  if (std::string_view(argv[index]) != name) {
    return false;
  }
  if (index + 1 >= argc) {
    failed = true;
    return false;
  }
  const auto parsed = parseUnsigned(argv[index + 1]);
  if (!parsed) {
    failed = true;
    return false;
  }
  out = *parsed;
  ++index;
  return true;
}

// Same contract as readUnsignedOption, for options taking a string value.
inline bool readStringOption(int argc, char** argv, int& index, std::string_view name,
                             std::string& out, bool& failed) {
  if (std::string_view(argv[index]) != name) {
    return false;
  }
  if (index + 1 >= argc) {
    failed = true;
    return false;
  }
  out = argv[index + 1];
  ++index;
  return true;
}

// "demo, compact, fulfilment" - for usage and error messages.
[[nodiscard]] inline std::string layoutNames() {
  std::string names;
  for (const world::LayoutInfo& layout : world::predefinedLayouts()) {
    names += names.empty() ? "" : ", ";
    names += layout.name;
  }
  return names;
}

}  // namespace warehousesim::apps
