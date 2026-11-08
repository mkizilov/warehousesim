#pragma once

#include <array>
#include <cstdint>
#include <stdexcept>

namespace warehousesim::robots {

// The operational state of a robot. The allowed transitions are fixed by isValidTransition():
//
//   Idle        -> Navigating | Charging | Unavailable
//   Navigating  -> Loading | Unloading | Charging | Idle | Unavailable
//   Loading     -> Navigating | Idle | Unavailable
//   Unloading   -> Idle | Unavailable
//   Charging    -> Idle | Navigating | Unavailable
//   Unavailable -> Idle
//
// A task cycle is Idle -> Navigating (to pickup) -> Loading -> Navigating (to dropoff) ->
// Unloading -> Idle; a charging trip is Idle -> Navigating (to charger) -> Charging -> Idle.
enum class RobotState : std::uint8_t {
  Idle,         // waiting for work, standing still
  Navigating,   // following a path towards a destination (see NavigationGoal)
  Loading,      // stationary, taking cargo on at a pickup
  Unloading,    // stationary, handing cargo over at a dropoff
  Charging,     // stationary on a charging station, recharging
  Unavailable,  // out of service (fault or flat battery); does nothing until restored
};

inline constexpr std::array<RobotState, 6> kAllRobotStates{
    RobotState::Idle,      RobotState::Navigating, RobotState::Loading,
    RobotState::Unloading, RobotState::Charging,   RobotState::Unavailable};

// Why a Navigating robot is travelling; None in every other state.
enum class NavigationGoal : std::uint8_t {
  None,
  Pickup,
  Dropoff,
  Charger,
};

[[nodiscard]] const char* toString(RobotState state) noexcept;
[[nodiscard]] const char* toString(NavigationGoal goal) noexcept;

// Whether a robot may go directly from `from` to `to`. A state never transitions to itself.
[[nodiscard]] bool isValidTransition(RobotState from, RobotState to) noexcept;

// Stationary states are ones in which the robot is busy in place (or out of service) and must
// not be asked to move out of the way.
[[nodiscard]] constexpr bool isStationary(RobotState state) noexcept {
  return state == RobotState::Loading || state == RobotState::Unloading ||
         state == RobotState::Charging || state == RobotState::Unavailable;
}

// Thrown when a robot operation is not allowed from the robot's current state or situation.
// The robot is left unchanged.
class InvalidTransition : public std::logic_error {
 public:
  using std::logic_error::logic_error;
};

}  // namespace warehousesim::robots
