#!/bin/sh
# Leak check for macOS, where AddressSanitizer has no LeakSanitizer: runs the command line program and
# every test executable under leaks(1) with --atExit, and fails if any of them reports a leak.
#
#   scripts/leaks_macos.sh [BUILD_DIR]            (default: build/release)
#
# BUILD_DIR must hold a build of the project with its tests (for example `cmake --preset release`). Each
# program is bounded by LEAKS_TIME_LIMIT seconds (default 900): one that hangs is killed and reported as a
# failure, not waited for. On Linux use the `asan` preset instead (ASAN_OPTIONS=detect_leaks=1).
#
# Exit status: 0 if every program ran to completion with no leak, 1 otherwise, 2 for a bad setup.
set -u

build=${1:-build/release}
limit=${LEAKS_TIME_LIMIT:-900}

if [ "$(uname -s)" != "Darwin" ] || ! command -v leaks >/dev/null 2>&1; then
  echo "error: this script needs macOS and its leaks(1) tool" >&2
  exit 2
fi
if [ ! -x "$build/apps/warehousesim_cli" ]; then
  echo "error: $build/apps/warehousesim_cli not found; build the project first" >&2
  exit 2
fi
build=$(cd "$build" && pwd)
repo=$(cd "$(dirname "$0")/.." && pwd)
out=${TMPDIR:-/tmp}/warehousesim-leaks.$$
# Every program runs from here: the tests write only below their working directory (ScratchDir).
mkdir -p "$out/cwd"
trap 'rm -rf "$out"' EXIT

# bounded SECONDS COMMAND...: the command's exit status, or 124 if it was killed for taking too long.
bounded() {
  seconds=$1
  shift
  if command -v timeout >/dev/null 2>&1; then
    timeout "$seconds" "$@"
  elif command -v gtimeout >/dev/null 2>&1; then
    gtimeout "$seconds" "$@"
  else
    perl -e 'my $s = shift; my $pid = fork(); if ($pid == 0) { exec @ARGV or exit 127 }
             $SIG{ALRM} = sub { kill "KILL", $pid; waitpid($pid, 0); exit 124 };
             alarm $s; waitpid($pid, 0); exit($? >> 8)' "$seconds" "$@"
  fi
}

failed=0
checked=0

# check LABEL COMMAND...: runs the command under leaks and prints one line.
check() {
  label=$1
  shift
  log="$out/$checked.log"
  checked=$((checked + 1))
  (cd "$out/cwd" && bounded "$limit" leaks --atExit -- "$@") > "$log" 2>&1
  status=$?
  summary=$(grep -E 'Process [0-9]+: [0-9]+ leaks? for [0-9]+ total leaked bytes' "$log" | tail -1 | sed -e 's/^Process [0-9]*: //')
  if [ "$status" -eq 124 ]; then
    printf 'FAIL  %-44s killed after %s s (hang?)\n' "$label" "$limit"
    failed=$((failed + 1))
  elif [ -z "$summary" ]; then
    printf 'FAIL  %-44s no leaks(1) summary, exit status %s\n' "$label" "$status"
    tail -3 "$log" | sed -e 's/^/        /'
    failed=$((failed + 1))
  elif [ "$status" -ne 0 ] && [ "$status" -ne 1 ]; then
    printf 'FAIL  %-44s exit status %s\n' "$label" "$status"
    failed=$((failed + 1))
  else
    case $summary in
      "0 leaks for 0 total leaked bytes"*) printf 'ok    %-44s %s\n' "$label" "$summary" ;;
      *)
        printf 'FAIL  %-44s %s\n' "$label" "$summary"
        grep -E 'Leak:|ROOT LEAK' "$log" | head -5 | sed -e 's/^/        /'
        failed=$((failed + 1))
        ;;
    esac
  fi
}

cli="$build/apps/warehousesim_cli"
check "cli: demo, 300 ticks" "$cli" --layout demo --ticks 300 --seed 1
check "cli: large, 100 robots, 2 planning threads" "$cli" --layout large --robots 100 --spawn 1 --ticks 200 --threads 2
check "cli: --audit on an example map" "$cli" --map "$repo/examples/cross-dock.json" --robots 12 --spawn 3 --ticks 300 --audit
check "cli: --metrics" "$cli" --layout compact --ticks 300 --metrics
check "cli: --record" "$cli" --layout compact --ticks 300 --seed 3 --record "$out/run.json"
check "cli: --replay" "$cli" --replay "$out/run.json"

# The programs that own SDL resources, when the build has them (SDL's dummy driver: no window server needed).
if [ -x "$build/apps/warehousesim_gui" ]; then
  SDL_VIDEODRIVER=dummy
  export SDL_VIDEODRIVER
  check "gui: 30 frames, dummy video driver" "$build/apps/warehousesim_gui" --frames 30 --layout compact
  check "editor: 30 frames, dummy video driver" "$build/apps/warehousesim_editor" --file "$out/cwd/map.json" --layout compact --frames 30
  unset SDL_VIDEODRIVER
fi

tests=0
for exe in "$build"/tests/warehousesim_*tests; do
  [ -x "$exe" ] || continue
  tests=$((tests + 1))
  check "tests: $(basename "$exe")" "$exe"
done
if [ "$tests" -eq 0 ]; then
  echo "FAIL  no test executables in $build/tests (configure with tests on)"
  failed=$((failed + 1))
fi

echo
if [ "$failed" -eq 0 ]; then
  echo "no leaks in any of the $checked programs"
  exit 0
fi
echo "$failed program(s) failed the leak check"
exit 1
