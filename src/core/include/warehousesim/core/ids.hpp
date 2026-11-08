#pragma once

#include <compare>
#include <cstdint>

namespace warehousesim::core {

// Zero is reserved as "no id" so default-constructed ids are never valid.
template <typename Tag>
struct StrongId {
  std::uint32_t value = 0;

  [[nodiscard]] constexpr bool valid() const noexcept { return value != 0; }

  friend constexpr auto operator<=>(const StrongId&, const StrongId&) = default;
};

struct RobotTag;
struct TaskTag;

using RobotId = StrongId<RobotTag>;
using TaskId = StrongId<TaskTag>;

}  // namespace warehousesim::core
