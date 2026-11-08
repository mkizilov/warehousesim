#include "warehousesim/robots/robot_state.hpp"

#include <cstddef>

namespace warehousesim::robots {

namespace {

constexpr std::size_t kStateCount = kAllRobotStates.size();

// Row = from, column = to, in RobotState enumerator order.
constexpr bool kTransitions[kStateCount][kStateCount] = {
    //                  Idle   Navig. Load.  Unload. Charg.  Unavail.
    /* Idle        */ {false, true, false, false, true, true},
    /* Navigating  */ {true, false, true, true, true, true},
    /* Loading     */ {true, true, false, false, false, true},
    /* Unloading   */ {true, false, false, false, false, true},
    /* Charging    */ {true, true, false, false, false, true},
    /* Unavailable */ {true, false, false, false, false, false},
};

}  // namespace

const char* toString(RobotState state) noexcept {
  switch (state) {
    case RobotState::Idle:
      return "idle";
    case RobotState::Navigating:
      return "navigating";
    case RobotState::Loading:
      return "loading";
    case RobotState::Unloading:
      return "unloading";
    case RobotState::Charging:
      return "charging";
    case RobotState::Unavailable:
      return "unavailable";
  }
  return "unknown";
}

const char* toString(NavigationGoal goal) noexcept {
  switch (goal) {
    case NavigationGoal::None:
      return "none";
    case NavigationGoal::Pickup:
      return "pickup";
    case NavigationGoal::Dropoff:
      return "dropoff";
    case NavigationGoal::Charger:
      return "charger";
  }
  return "unknown";
}

bool isValidTransition(RobotState from, RobotState to) noexcept {
  return kTransitions[static_cast<std::size_t>(from)][static_cast<std::size_t>(to)];
}

}  // namespace warehousesim::robots
