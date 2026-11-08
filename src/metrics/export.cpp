#include "warehousesim/metrics/export.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <locale>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <system_error>

namespace warehousesim::metrics {

namespace {

// Fixed decimals in the C locale, whatever the program's locale is.
std::string fixed(double value, int decimals) {
  std::ostringstream out;
  out.imbue(std::locale::classic());
  out << std::fixed << std::setprecision(decimals) << value;
  return out.str();
}

std::string maybe(const std::optional<double>& value, int decimals) {
  return value ? fixed(*value, decimals) : std::string();
}

std::string maybePercent(const std::optional<double>& fraction) {
  return fraction ? fixed(*fraction * 100.0, 1) + "%" : std::string("n/a");
}

std::string maybeNumber(const std::optional<double>& value, int decimals) {
  return value ? fixed(*value, decimals) : std::string("n/a");
}

std::string join(const std::vector<std::string>& fields) {
  std::string line;
  for (std::size_t i = 0; i < fields.size(); ++i) {
    line += i == 0 ? "" : ",";
    line += fields[i];
  }
  return line;
}

// 1, 0.5, 2.25, 0.000001: up to nine decimals, without trailing zeros.
std::string trimmed(double value) {
  std::string text = fixed(value, 9);
  while (text.back() == '0') {
    text.pop_back();
  }
  if (text.back() == '.') {
    text.pop_back();
  }
  return text;
}

std::string num(std::uint64_t value) { return std::to_string(value); }

// A line of the dashboard: a label in a column of at least 12 characters and always followed by a
// space, then the text, cut to the dashboard's width.
std::string row(const std::string& label, const std::string& text) {
  std::string line = label;
  line.resize(label.empty() ? 12 : std::max<std::size_t>(label.size() + 1, 12), ' ');
  line += text;
  if (line.size() > kDashboardColumns) {
    line.resize(kDashboardColumns);
  }
  while (!line.empty() && line.back() == ' ') {
    line.pop_back();
  }
  return line;
}

}  // namespace

std::string csvField(std::string_view value) {
  if (value.find_first_of(",\"\r\n") == std::string_view::npos) {
    return std::string(value);
  }
  std::string quoted = "\"";
  for (const char c : value) {
    quoted += c;
    if (c == '"') {
      quoted += '"';
    }
  }
  quoted += '"';
  return quoted;
}

std::vector<std::string> splitCsvLine(std::string_view line) {
  std::vector<std::string> fields(1);
  bool quoted = false;
  for (std::size_t i = 0; i < line.size(); ++i) {
    const char c = line[i];
    if (quoted) {
      if (c == '"' && i + 1 < line.size() && line[i + 1] == '"') {
        fields.back() += '"';
        ++i;
      } else if (c == '"') {
        quoted = false;
      } else {
        fields.back() += c;
      }
    } else if (c == '"' && fields.back().empty()) {
      quoted = true;
    } else if (c == ',') {
      fields.emplace_back();
    } else {
      fields.back() += c;
    }
  }
  if (quoted) {
    throw std::invalid_argument("a quote is left open in a CSV line");
  }
  return fields;
}

std::vector<std::string> runColumns() {
  return {"layout",
          "robots",
          "seed",
          "spawn_interval",
          "ticks",
          "basis",
          "sim_seconds_per_tick",
          "tasks_submitted",
          "tasks_completed",
          "tasks_failed",
          "tasks_cancelled",
          "tasks_open_at_end",
          "tasks_per_sim_hour",
          "tasks_per_robot_hour",
          "avg_completion_ticks",
          "avg_completion_sim_s",
          "max_completion_ticks",
          "utilization",
          "moving_fraction",
          "waiting_fraction",
          "handling_fraction",
          "charging_fraction",
          "idle_fraction",
          "out_of_service_fraction",
          "idle_robot_ticks",
          "waiting_robot_ticks",
          "cells_per_robot",
          "cells_per_completed_task",
          "conflicts_prevented",
          "conflict_detections",
          "collisions_occurred",
          "replans",
          "replans_per_sim_hour",
          "reroutes",
          "repaths",
          "obstacle_repairs",
          "searches",
          "energy_units_est",
          "energy_units_per_task_est",
          "energy_idle_est",
          "energy_moving_est",
          "energy_handling_est"};
}

std::string runsCsvHeader() { return join(runColumns()); }

std::string runsCsvRow(const RunLabel& label, const FleetMetrics& m) {
  const EnergyEstimate energy = m.energyEstimate();
  const std::vector<std::string> fields{csvField(label.layout),
                                        num(m.robots.size()),
                                        num(label.seed),
                                        num(label.taskSpawnInterval),
                                        num(m.ticks),
                                        std::string(kBasis),
                                        fixed(m.options.secondsPerTick, 4),
                                        num(m.tasksSubmitted),
                                        num(m.tasksCompleted),
                                        num(m.tasksFailed),
                                        num(m.tasksCancelled),
                                        num(m.tasksOpen),
                                        maybe(m.tasksPerHour(), 4),
                                        maybe(m.tasksPerRobotHour(), 4),
                                        maybe(m.meanCompletionTicks(), 4),
                                        maybe(m.meanCompletionSeconds(), 4),
                                        num(m.completionTicksMax),
                                        maybe(m.utilization(), 4),
                                        maybe(m.fraction(Activity::Moving), 4),
                                        maybe(m.fraction(Activity::Waiting), 4),
                                        maybe(m.fraction(Activity::Handling), 4),
                                        maybe(m.fraction(Activity::Charging), 4),
                                        maybe(m.fraction(Activity::Idle), 4),
                                        maybe(m.fraction(Activity::Unavailable), 4),
                                        num(m.ticksIn(Activity::Idle)),
                                        num(m.ticksIn(Activity::Waiting)),
                                        maybe(m.cellsPerRobot(), 4),
                                        maybe(m.cellsPerCompletedTask(), 4),
                                        num(m.conflictsPrevented),
                                        num(m.conflictDetections),
                                        num(m.collisionsOccurred),
                                        num(m.replanEvents()),
                                        maybe(m.replansPerHour(), 4),
                                        num(m.reroutes),
                                        num(m.repaths),
                                        num(m.obstacleRepairs),
                                        num(m.searches),
                                        num(energy.total()),
                                        maybe(m.energyPerCompletedTask(), 4),
                                        num(energy.idle),
                                        num(energy.moving),
                                        num(energy.handling)};
  return join(fields);
}

std::string runsCsv(const std::vector<std::pair<RunLabel, FleetMetrics>>& runs) {
  std::string text = runsCsvHeader() + "\n";
  for (const auto& [label, metrics] : runs) {
    text += runsCsvRow(label, metrics) + "\n";
  }
  return text;
}

std::string robotsCsv(const FleetMetrics& m) {
  std::string text =
      "robot,basis,moving_ticks,waiting_ticks,handling_ticks,charging_ticks,idle_ticks,"
      "out_of_service_ticks,cells,deliveries,conflicts_prevented,utilization\n";
  for (const RobotMetrics& robot : m.robots) {
    const std::uint64_t unavailable = robot.budget.in(Activity::Unavailable);
    const std::uint64_t service = robot.budget.total() - unavailable;
    const std::uint64_t busy = robot.budget.in(Activity::Moving) +
                               robot.budget.in(Activity::Waiting) +
                               robot.budget.in(Activity::Handling);
    const std::optional<double> utilization =
        service == 0
            ? std::nullopt
            : std::optional<double>(static_cast<double>(busy) / static_cast<double>(service));
    text += join({num(robot.id.value), std::string(kBasis), num(robot.budget.in(Activity::Moving)),
                  num(robot.budget.in(Activity::Waiting)), num(robot.budget.in(Activity::Handling)),
                  num(robot.budget.in(Activity::Charging)), num(robot.budget.in(Activity::Idle)),
                  num(unavailable), num(robot.cellsTravelled), num(robot.deliveries),
                  num(robot.conflictsPrevented), maybe(utilization, 4)}) +
            "\n";
  }
  return text;
}

std::string seriesCsv(const FleetMetrics& m) {
  std::string text =
      "tick,basis,window_ticks,tasks_completed,tasks_per_sim_hour,utilization,waiting_fraction,"
      "tasks_open\n";
  for (const MetricsSample& s : m.samples) {
    const double hours = static_cast<double>(s.windowTicks) * m.options.secondsPerTick / 3600.0;
    const std::optional<double> perHour =
        s.windowTicks == 0 ? std::nullopt
                           : std::optional<double>(static_cast<double>(s.tasksCompleted) / hours);
    const std::optional<double> utilization =
        s.serviceRobotTicks == 0 ? std::nullopt
                                 : std::optional<double>(static_cast<double>(s.busyRobotTicks) /
                                                         static_cast<double>(s.serviceRobotTicks));
    // As in the runs file, a fraction of all robot-ticks, out of service ones included.
    const std::uint64_t robotTicks = s.windowTicks * m.robots.size();
    const std::optional<double> waiting =
        robotTicks == 0 ? std::nullopt
                        : std::optional<double>(static_cast<double>(s.waitingRobotTicks) /
                                                static_cast<double>(robotTicks));
    text += join({num(s.tick), std::string(kBasis), num(s.windowTicks), num(s.tasksCompleted),
                  maybe(perHour, 4), maybe(utilization, 4), maybe(waiting, 4), num(s.tasksOpen)}) +
            "\n";
  }
  return text;
}

namespace {

namespace fs = std::filesystem;

std::optional<std::string> writeWhole(const fs::path& path, const std::string& text) {
  fs::path temporary = path;
  temporary += ".tmp";
  {
    std::ofstream out(temporary, std::ios::binary | std::ios::trunc);
    out << text;
    out.flush();
    if (!out) {
      std::error_code ignored;
      fs::remove(temporary, ignored);
      return "could not write " + path.string();
    }
  }
  std::error_code error;
  fs::rename(temporary, path, error);
  if (error) {
    const std::string reason = error.message();
    fs::remove(temporary, error);
    return "could not write " + path.string() + ": " + reason;
  }
  return std::nullopt;
}

}  // namespace

std::optional<std::string> writeRunFiles(const fs::path& base, const RunLabel& label,
                                         const FleetMetrics& metrics) {
  const auto named = [&base](const char* suffix) {
    fs::path path = base;
    path += suffix;
    return path;
  };
  const std::string runs = runsCsvHeader() + "\n" + runsCsvRow(label, metrics) + "\n";
  if (auto problem = writeWhole(named(".runs.csv"), runs)) {
    return problem;
  }
  if (auto problem = writeWhole(named(".robots.csv"), robotsCsv(metrics))) {
    return problem;
  }
  return writeWhole(named(".series.csv"), seriesCsv(metrics));
}

std::string formatDuration(double seconds) {
  // Beyond 1e15 s (some 30 million years) a whole number of seconds is not meaningful, and rounding
  // it to an integer would overflow.
  if (!std::isfinite(seconds) || seconds < 0.0 || seconds > 1e15) {
    return "n/a";
  }
  const auto total = static_cast<std::uint64_t>(std::llround(seconds));
  const std::uint64_t hours = total / 3600;
  const std::uint64_t minutes = (total % 3600) / 60;
  const std::uint64_t secs = total % 60;
  std::string text;
  if (hours > 0) {
    text += std::to_string(hours) + "h ";
  }
  if (hours > 0 || minutes > 0) {
    text += std::to_string(minutes) + "m ";
  }
  return text + std::to_string(secs) + "s";
}

std::vector<std::string> dashboardLines(const FleetMetrics& m) {
  const EnergyEstimate energy = m.energyEstimate();
  std::vector<std::string> lines;
  lines.push_back(row("FLEET METRICS", "(simulated)"));
  lines.push_back(row("sim time", formatDuration(m.simulatedSeconds()) + " = " + num(m.ticks) +
                                      " ticks at " + trimmed(m.options.secondsPerTick) +
                                      " s/tick"));
  lines.push_back(row("robots", num(m.robots.size())));
  lines.push_back(row("tasks", "done " + num(m.tasksCompleted) + "  failed " + num(m.tasksFailed) +
                                   "  cancelled " + num(m.tasksCancelled) + "  open " +
                                   num(m.tasksOpen)));
  lines.push_back(row("throughput", maybeNumber(m.tasksPerHour(), 1) + " tasks/sim-hour  (" +
                                        maybeNumber(m.tasksPerRobotHour(), 1) +
                                        " per robot-hour)"));
  lines.push_back(row("completion", "mean " + maybeNumber(m.meanCompletionTicks(), 1) +
                                        " ticks = " + maybeNumber(m.meanCompletionSeconds(), 1) +
                                        " s  max " + num(m.completionTicksMax) + " ticks"));
  lines.push_back(row("utilization", maybePercent(m.utilization()) + " of in-service robot time"));
  lines.push_back(row("robot time", "moving " + maybePercent(m.fraction(Activity::Moving)) +
                                        "  waiting " + maybePercent(m.fraction(Activity::Waiting)) +
                                        "  handling " +
                                        maybePercent(m.fraction(Activity::Handling))));
  lines.push_back(row("", "charging " + maybePercent(m.fraction(Activity::Charging)) + "  idle " +
                              maybePercent(m.fraction(Activity::Idle)) + "  out of service " +
                              maybePercent(m.fraction(Activity::Unavailable))));
  lines.push_back(row("travel", maybeNumber(m.cellsPerRobot(), 1) + " cells/robot  " +
                                    maybeNumber(m.cellsPerCompletedTask(), 1) + " cells/task"));
  lines.push_back(row("conflicts", num(m.conflictsPrevented) + " prevented  " +
                                       num(m.collisionsOccurred) + " collisions occurred"));
  lines.push_back(row("replans", num(m.replanEvents()) + " (" + maybeNumber(m.replansPerHour(), 1) +
                                     "/sim-hour)  " + num(m.reroutes) + " reroutes " +
                                     num(m.repaths) + " repaths"));
  lines.push_back(row("energy est.", num(energy.total()) + " units (" +
                                         maybeNumber(m.energyPerCompletedTask(), 1) + "/task)"));
  lines.push_back(row("", "model: idle " + num(m.options.energy.idlePerTick) + "/tick, cell " +
                              num(m.options.energy.perCell) + ", handling " +
                              num(m.options.energy.handlingPerTick) + "/tick"));
  lines.push_back(row(std::string(kDisclaimer), ""));
  return lines;
}

}  // namespace warehousesim::metrics
