#pragma once

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <optional>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "warehousesim/metrics/fleet_metrics.hpp"

// The ways simulated metrics leave the program: CSV for spreadsheets and scripts, and the lines of
// text of the dashboard. Every output says what it is: the CSV files carry a `basis` column that is
// always "simulated", and the dashboard ends with the disclaimer below.
namespace warehousesim::metrics {

inline constexpr std::string_view kBasis = "simulated";
inline constexpr std::string_view kDisclaimer = "Simulated results. Not a claim about real robots.";

// What identifies a run in the CSV, besides the metrics themselves. Together with the fleet size
// and the number of ticks it is everything needed to run the scenario again.
struct RunLabel {
  std::string layout;
  std::uint64_t seed = 0;
  std::uint64_t taskSpawnInterval = 0;
};

// RFC 4180: a field with a comma, quote or line break is wrapped in quotes with quotes doubled.
[[nodiscard]] std::string csvField(std::string_view value);
// The fields of one line (no line break). Throws std::invalid_argument for a quote left open.
[[nodiscard]] std::vector<std::string> splitCsvLine(std::string_view line);

// One row per run. Figures are written with a fixed number of decimals, so the same run always
// gives the same text on every platform; a figure that is undefined (no time observed, no task
// finished) is an empty field, not a 0.
[[nodiscard]] std::vector<std::string> runColumns();
[[nodiscard]] std::string runsCsvHeader();  // no line break
[[nodiscard]] std::string runsCsvRow(const RunLabel& label, const FleetMetrics& metrics);  // ditto
// A whole file: the header and one line (each ending in a line break) per run.
[[nodiscard]] std::string runsCsv(const std::vector<std::pair<RunLabel, FleetMetrics>>& runs);

// One row per robot of a single run, with the time budget in ticks.
[[nodiscard]] std::string robotsCsv(const FleetMetrics& metrics);
// One row per sample (window) of a single run.
[[nodiscard]] std::string seriesCsv(const FleetMetrics& metrics);

// "1h 23m 20s", "5m 0s", "42s"; "n/a" for a time that is negative or not finite.
[[nodiscard]] std::string formatDuration(double seconds);

// Writes the three files of one run next to each other: `<base>.runs.csv` (one row),
// `<base>.robots.csv` and `<base>.series.csv`. Each is written whole to a temporary file next to it
// and then renamed over the real name, so a failure never leaves a half-written file under it.
// Returns a message saying what went wrong and which file, or nothing if all three were written.
// The directory of `base` must exist.
[[nodiscard]] std::optional<std::string> writeRunFiles(const std::filesystem::path& base,
                                                       const RunLabel& label,
                                                       const FleetMetrics& metrics);

inline constexpr std::size_t kDashboardColumns = 62;
// The dashboard as lines of text, none longer than kDashboardColumns. The first is the title, which
// says the figures are simulated, and the last the disclaimer.
[[nodiscard]] std::vector<std::string> dashboardLines(const FleetMetrics& metrics);

}  // namespace warehousesim::metrics
