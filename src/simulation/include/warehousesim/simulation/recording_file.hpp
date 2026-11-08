#pragma once

#include <cstddef>
#include <filesystem>
#include <optional>
#include <string>
#include <string_view>

#include "warehousesim/simulation/recording.hpp"

// The recording file: a versioned JSON document with an explicit schema, and the reading,
// validating and writing of it.
//
// Format, version 1 (a JSON object; unknown keys, duplicate keys and wrong types are errors, and so
// is any key listed as required that is missing):
//
//   {
//     "format":  "warehousesim-recording",        required, exactly this text
//     "version": 1,                               required, the integer 1
//     "engine":  1,                               required: the engine revision it was made by
//     "scenario": {                               required: what the simulation is built from
//       "seed":   5,                              the random seed, an integer from 0 to 2^64 - 1
//       "layout": ["#####", "#.D.#", ...],        one text per row, one character per cell:
//                                                 . free  # wall  S shelf  P pickup  D dropoff
//                                                 C charging station; 1 to 512 rows and columns
//       "robots": [[x, y], ...],                  start cells; robot N is the Nth (0 to 4096)
//       "config": { ... }                         every key of the simulation's configuration,
//     },                                          with "robot": { ... } for the robot model
//     "ticks": 300,                               required: steps taken, 0 to 10^9
//     "checkpointInterval": 100,                  optional, default 0: ticks between checkpoints
//     "commands": [                               optional, in the order they were applied:
//       {"tick": 12, "command": "submitTask", "accepted": true,
//        "pickup": [x, y], "dropoff": [x, y], "priority": "low|normal|high|urgent"},
//       {"tick": 40, "command": "setTaskPriority", "accepted": true, "task": 5, "priority":
//       "high"},
//       {"tick": 50, "command": "cancelTask", "accepted": true, "task": 7},
//       {"tick": 60, "command": "setRobotService", "accepted": true, "robot": 2,
//        "inService": false},
//       {"tick": 70, "command": "placeObstacle", "accepted": true, "cell": [x, y],
//        "type": "wall|shelf"},
//       {"tick": 80, "command": "clearObstacle", "accepted": true, "cell": [x, y]}
//     ],
//     "taskEvents": [                             optional: what the run did to tasks, in order.
//     Left
//                                                 out, the tasks a replay makes are not compared
//                                                 with anything; given (even as []), they are
//       {"tick": 0, "event": "created", "task": 1, "source": "command|engine",
//        "pickup": [x, y], "dropoff": [x, y], "priority": "normal"},
//       {"tick": 50, "event": "cancelled", "task": 7, "source": "command|engine"}
//     ],
//     "checkpoints": [{"tick": 0, "checksum": "0123456789abcdef"}, ...],   optional
//     "end": {"tick": 300, "checksum": "..."}     optional: the state after the last command
//   }
//
// Checksums are 16 lowercase hexadecimal digits. Everything but "scenario" and "ticks" may be left
// out of a file made by hand: a recording with commands and no "taskEvents", "checkpoints" or "end"
// is a scenario script. It replays as it says, and only the commands' "accepted" is checked.
//
// A change that old readers cannot understand gets a new version number; a reader that sees a newer
// version than it knows says so instead of guessing.
namespace warehousesim::simulation {

inline constexpr std::string_view kRecordingFormatName = "warehousesim-recording";
inline constexpr int kRecordingFormatVersion = 1;
// A run of a million steps with a task every five and a few commands takes about 30 MB.
inline constexpr std::size_t kMaxRecordingFileBytes = 64U * 1024U * 1024U;
// Document, section, list, entry, pair: the schema never goes deeper than 5; one more is allowed so
// that the error says what is wrong instead of the nesting being the error.
inline constexpr int kMaxRecordingNesting = 6;

// The outcome of reading a recording: the recording if there is one, otherwise the first thing
// that is wrong with the file. Exactly one of the two is set.
struct RecordingLoad {
  std::optional<Recording> recording;
  std::optional<RecordingProblem> problem;

  [[nodiscard]] bool ok() const noexcept { return recording.has_value(); }
};

// Reads a recording from text. Never throws for bad input: a document that is too big, not JSON,
// nested too deeply, or that does not fit the schema or is not consistent with itself (see
// validate) comes back as a problem, and so does a scenario that cannot be built. A recording that
// was read can be replayed.
[[nodiscard]] RecordingLoad parseRecording(std::string_view text);

// The recording as text. The text is canonical: equal recordings give identical text, and
// parseRecording(serializeRecording(r)) gives back a recording that serializes to the same text.
// Throws std::invalid_argument (with the first problem as its message) if validate() finds one.
[[nodiscard]] std::string serializeRecording(const Recording& recording);

// Reads a recording file. Problems with the file itself (missing, not a regular file, too big,
// unreadable) are reported like problems with the content.
[[nodiscard]] RecordingLoad loadRecordingFile(const std::filesystem::path& path);

// Validates and writes a recording file. The text goes to a temporary file next to `path` first and
// is moved into place only when it has been written completely, so a failure never leaves a
// half-written recording under the real name. Returns the problem, or nullopt on success.
[[nodiscard]] std::optional<RecordingProblem> saveRecordingFile(const std::filesystem::path& path,
                                                                const Recording& recording);

}  // namespace warehousesim::simulation
