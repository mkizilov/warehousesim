#pragma once

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <optional>

#include "warehousesim/metrics/export.hpp"
#include "warehousesim/metrics/fleet_metrics.hpp"
#include "warehousesim/simulation/runner.hpp"
#include "warehousesim/simulation/simulation.hpp"
#include "warehousesim/view/fleet_controller.hpp"
#include "warehousesim/view/view_controller.hpp"

namespace warehousesim::view {

// The glue between the fleet interface and the simulation, one frame at a time: it reads what is to
// be drawn from the runner, tells the controller what became of the commands it posted, runs the
// controller on this frame's input and hands what it asked for to the runner. It is the only place
// where the two meet, and it does so only through the runner's public interface: commands go in
// through post(), playback state through setPaused() and setSpeed(). Both are borrowed and must
// outlive the session.
class FleetSession {
 public:
  // Where the `E` key writes the performance figures: `<base>.runs.csv`, `<base>.robots.csv` and
  // `<base>.series.csv` (see metrics::writeRunFiles). The label names the run; its seed is the
  // runner's current one.
  struct MetricsExport {
    std::filesystem::path base;
    metrics::RunLabel label;
  };

  FleetSession(simulation::SimulationRunner& runner, FleetController& controller,
               std::optional<MetricsExport> exportTo = std::nullopt);

  // Runs one frame and returns the snapshot to draw (the runner's last two states blended by how
  // far real time has got between them). Commands that the runner's queue has no room for are
  // reported on the controller's message line, never dropped silently.
  simulation::Snapshot step(const InputEvents& input, double frameSeconds);

  // The run's performance figures as of this frame, while the dashboard is on; none otherwise (they
  // are only fetched when they are going to be drawn).
  [[nodiscard]] const std::optional<metrics::FleetMetrics>& metrics() const noexcept {
    return metrics_;
  }

 private:
  void exportMetrics();

  simulation::SimulationRunner& runner_;
  FleetController& controller_;
  std::optional<MetricsExport> exportTo_;
  std::optional<metrics::FleetMetrics> metrics_;
  std::uint64_t epoch_ = 0;
  std::size_t reported_ = 0;  // how much of the runner's command log the controller has seen
};

}  // namespace warehousesim::view
