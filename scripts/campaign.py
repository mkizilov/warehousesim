#!/usr/bin/env python3
"""Scenario campaign: layouts x fleet sizes x task loads x seeds, each run audited on every step.

    scripts/campaign.py build/release/apps/warehousesim_cli [--ticks N] [--quick] [--json FILE]

Every run is `warehousesim_cli ... --audit`: the simulator checks itself after every step against the
invariants in src/simulation/audit.hpp (collisions, task ownership, event order, finite numbers).
The campaign fails (exit 1) if any run breaks one, or fails in any way other than being refused
because the layout has no room for the fleet. Throughput is printed as tables, for reading; it is
never an assertion, because it is a property of the model and not a rule.

All figures are simulated: ticks of a model, not seconds or robots of the real world.
Needs only Python 3 and the CLI; run from the repository root.
"""
import argparse
import json
import subprocess
import sys
import time

LAYOUTS = [("demo", ["--layout", "demo"]), ("compact", ["--layout", "compact"]),
           ("fulfilment", ["--layout", "fulfilment"]), ("large", ["--layout", "large"])]
EXAMPLES = ["small-depot", "fulfilment-aisles", "cross-dock", "choke-point", "single-lane",
            "charging-hub"]
LOADS = [(1, "heavy load, a task every tick"), (5, "medium load, a task every 5 ticks"),
         (25, "light load, a task every 25 ticks")]


def run_case(cli, source, fleet, spawn, seed, ticks):
    command = [cli] + source + ["--robots", str(fleet), "--spawn", str(spawn), "--ticks", str(ticks),
                                "--seed", str(seed), "--audit"]
    done = subprocess.run(command, capture_output=True, text=True)
    result = {"fleet": fleet, "spawn": spawn, "seed": seed, "exit": done.returncode}
    lines = done.stderr.strip().splitlines()
    if done.returncode == 0:
        final = json.loads(done.stdout)
        result.update(done=final["completedTasks"], failed=final["failedTasks"],
                      open=len(final["activeTasks"]), gave_up=final["planning"]["abandoned"],
                      rings=final["planning"]["deadlocks"], states=int(lines[-1].split()[1]))
    result["message"] = "\n".join(lines[-6:])
    result["no_room"] = done.returncode != 0 and "has room for only" in done.stderr
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    parser.add_argument("cli", help="path to warehousesim_cli")
    parser.add_argument("--ticks", type=int, default=2000, help="length of every run (default 2000)")
    parser.add_argument("--quick", action="store_true", help="a small campaign, for CI")
    parser.add_argument("--json", help="write every run to this file")
    args = parser.parse_args()

    layouts = list(LAYOUTS) + [(name, ["--map", "examples/%s.json" % name]) for name in EXAMPLES]
    fleets, loads, seeds = [1, 4, 16, 64], [1, 5, 25], [1, 2, 3]
    if args.quick:
        layouts, fleets, loads, seeds = layouts[:3] + layouts[4:6], [1, 8], [5], [1]

    runs = []
    started = time.time()
    for name, source in layouts:
        for fleet in fleets:
            for spawn in loads:
                for seed in seeds:
                    run = run_case(args.cli, source, fleet, spawn, seed, args.ticks)
                    run["layout"] = name
                    runs.append(run)

    ok = [r for r in runs if r["exit"] == 0]
    problems = [r for r in runs if r["exit"] != 0 and not r["no_room"]]
    unclean = [r for r in ok if not r["message"].endswith("no violations")]
    print("campaign: %d runs of %d ticks in %.0f s" % (len(runs), args.ticks, time.time() - started))
    print("  ran and audited: %d (%d states)" % (len(ok), sum(r["states"] for r in ok)))
    print("  refused, the layout has no room for the fleet: %d" % sum(r["no_room"] for r in runs))
    print("  audit violations: %d runs; other failures: %d runs" % (len(unclean), len(problems)))
    print("  tasks completed %d, failed %d; robots gave up a task %d times (a task handed back to the "
          "queue counts too); rings of robots waiting for each other detected %d" % (
              sum(r["done"] for r in ok), sum(r["failed"] for r in ok), sum(r["gave_up"] for r in ok),
              sum(r["rings"] for r in ok)))
    for run in unclean + problems:
        print("\n%s, %d robots, a task every %d ticks, seed %d (exit %d):\n%s" % (
            run["layout"], run["fleet"], run["spawn"], run["seed"], run["exit"], run["message"]))

    if not args.quick:
        for spawn, title in [load for load in LOADS if load[0] in loads]:
            print("\n#### %s: tasks completed in %d ticks, mean of %d seeds (failed in brackets)\n" % (
                title, args.ticks, len(seeds)))
            print("| layout | " + " | ".join("%d robots" % f for f in fleets) + " |")
            print("| --- | " + " | ".join("---:" for _ in fleets) + " |")
            for name, _ in layouts:
                cells = []
                for fleet in fleets:
                    rs = [r for r in ok if r["layout"] == name and r["fleet"] == fleet and r["spawn"] == spawn]
                    if not rs:
                        cells.append("no room")
                        continue
                    failed = sum(r["failed"] for r in rs) / len(rs)
                    cells.append("%.0f" % (sum(r["done"] for r in rs) / len(rs)) +
                                 (" (%.1f failed)" % failed if failed >= 0.05 else ""))
                print("| %s | %s |" % (name, " | ".join(cells)))

    if args.json:
        with open(args.json, "w") as out:
            json.dump(runs, out, indent=1)
    return 1 if unclean or problems else 0


if __name__ == "__main__":
    sys.exit(main())
