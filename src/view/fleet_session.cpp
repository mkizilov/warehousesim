#include "warehousesim/view/fleet_session.hpp"

#include <string>
#include <utility>

#include "warehousesim/view/interpolation.hpp"

namespace warehousesim::view {

FleetSession::FleetSession(simulation::SimulationRunner& runner, FleetController& controller,
                           std::optional<MetricsExport> exportTo)
    : runner_(runner), controller_(controller), exportTo_(std::move(exportTo)) {}

void FleetSession::exportMetrics() {
  if (!exportTo_) {
    controller_.report("nowhere to write the metrics: start with --metrics-csv BASE", true);
    return;
  }
  metrics::RunLabel label = exportTo_->label;
  label.seed = runner_.seed();
  const std::string base = exportTo_->base.string();
  if (const auto problem = metrics::writeRunFiles(exportTo_->base, label, runner_.metrics())) {
    controller_.report(*problem, true);
    return;
  }
  controller_.report("wrote " + base + ".runs.csv, .robots.csv and .series.csv", false);
}

simulation::Snapshot FleetSession::step(const InputEvents& input, double frameSeconds) {
  const simulation::FrameState frame = runner_.frame();
  simulation::Snapshot drawn = blend(frame.previous, frame.latest, frame.alpha);

  // The log starts over when the scenario is restarted; what the controller has seen of the old one
  // is of no use for the new one.
  if (frame.epoch != epoch_) {
    epoch_ = frame.epoch;
    reported_ = 0;
  }
  for (const simulation::CommandRecord& record : runner_.commandLog(reported_)) {
    controller_.note(record);
    ++reported_;
  }

  const FleetOutput output = controller_.apply(input, frameSeconds, drawn, frame.epoch);
  if (output.paused) {
    runner_.setPaused(*output.paused);
  }
  if (output.speed) {
    runner_.setSpeed(*output.speed);
  }
  for (const simulation::Command& command : output.commands) {
    if (!runner_.post(command)) {
      controller_.report("too many commands waiting; try again", true);
    }
  }
  if (output.exportMetrics) {
    exportMetrics();
  }
  metrics_.reset();
  if (controller_.showMetrics()) {
    metrics_ = runner_.metrics();
  }
  return drawn;
}

}  // namespace warehousesim::view
