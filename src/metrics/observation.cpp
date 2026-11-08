#include "warehousesim/metrics/observation.hpp"

namespace warehousesim::metrics {

const char* toString(Activity activity) noexcept {
  switch (activity) {
    case Activity::Moving:
      return "moving";
    case Activity::Waiting:
      return "waiting";
    case Activity::Handling:
      return "handling";
    case Activity::Charging:
      return "charging";
    case Activity::Idle:
      return "idle";
    case Activity::Unavailable:
      return "out_of_service";
  }
  return "unknown";
}

}  // namespace warehousesim::metrics
