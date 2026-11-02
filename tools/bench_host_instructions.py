#!/usr/bin/env python3
"""Count executed host instructions, including startup and JIT emission.

Use matched Release builds with FIL_ENABLE_NATIVE=OFF: Valgrind may not
support the instructions selected by -march=native. No warmup or collection
filter is used, so compilation, generated code, helpers and dispatch all count.
"""

import argparse
import json
from pathlib import Path
import re
import shutil
import subprocess
import sys


def positive_int(value):
    number = int(value)
    if number <= 0:
        raise argparse.ArgumentTypeError("must be positive")
    return number


def measure(binary, args, label):
    prefix = args.output_dir / label
    profile = prefix.with_suffix(".callgrind")
    stdout_path = prefix.with_suffix(".stdout")
    stderr_path = prefix.with_suffix(".stderr")
    command = [
        "valgrind", "--tool=callgrind", f"--callgrind-out-file={profile}",
        str(binary.resolve()), "run-network", str(args.network.resolve()),
        "--duration-ms", str(args.duration_ms), "--max-instructions", "1000000000",
        "--quantum", "1024", "--strict-mmio", "--adc-decimation", "1",
    ]
    with stdout_path.open("w") as stdout, stderr_path.open("w") as stderr:
        subprocess.run(command, stdout=stdout, stderr=stderr, check=True)
    output = stdout_path.read_text()
    counters = {}
    for key in ("instructions", "cycles", "time_ns"):
        match = re.search(rf"^{key}: (\d+)$", output, re.MULTILINE)
        if not match:
            raise ValueError(f"missing {key} in {stdout_path}")
        counters[key] = int(match[1])
    if counters["instructions"] == 0:
        raise ValueError("no guest instructions retired")
    if not re.search(r"^stop: time-budget$", output, re.MULTILINE):
        raise ValueError(f"run did not reach its time budget: {stdout_path}")
    if not re.search(r"\btrace_instructions=[1-9]\d*", output):
        raise ValueError("no native traces executed; this does not measure JIT kernels")
    data = profile.read_text()
    events = re.search(r"^events:\s+(.+)$", data, re.MULTILINE)
    summary = re.search(r"^summary:\s+(.+)$", data, re.MULTILINE)
    if not events or not summary:
        raise ValueError(f"missing Callgrind summary in {profile}")
    totals = dict(zip(events[1].split(), map(int, summary[1].split()), strict=True))
    host = totals["Ir"]
    return {
        "binary": str(binary.resolve()),
        "host_instructions": host,
        "guest_instructions": counters["instructions"],
        "host_per_guest": host / counters["instructions"],
        "counters": counters,
        "profile": str(profile),
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--binary", required=True, type=Path)
    parser.add_argument("--baseline", type=Path)
    parser.add_argument("--network", type=Path, default=Path("configs/networks/per_vehicle.json"))
    parser.add_argument("--duration-ms", type=positive_int, default=100)
    parser.add_argument("--output-dir", type=Path, required=True,
                        help="directory for profiles, stdout, stderr and report.json")
    parser.add_argument("--max-host-per-guest", type=float,
                        help="optional runtime budget; exits 1 when exceeded")
    args = parser.parse_args()
    if args.max_host_per_guest is not None and not 0 < args.max_host_per_guest < float("inf"):
        parser.error("--max-host-per-guest must be finite and positive")
    if not shutil.which("valgrind"):
        parser.error("valgrind is required")
    args.output_dir = args.output_dir.resolve()
    args.output_dir.mkdir(parents=True, exist_ok=True)
    report = {"metric": "executed host Ir / retired guest instructions; includes compilation"}
    if args.baseline:
        report["baseline"] = measure(args.baseline, args, "baseline")
    report["current"] = measure(args.binary, args, "current")
    if args.baseline:
        before, after = report["baseline"], report["current"]
        if before["counters"] != after["counters"]:
            raise ValueError("baseline/current guest counters differ; results are not comparable")
        report["host_instruction_reduction_percent"] = (
            100 * (before["host_instructions"] - after["host_instructions"])
            / before["host_instructions"]
        )
    failed = (args.max_host_per_guest is not None
              and report["current"]["host_per_guest"] > args.max_host_per_guest)
    if args.max_host_per_guest is not None:
        report["budget"] = args.max_host_per_guest
        report["budget_met"] = not failed
    rendered = json.dumps(report, indent=2) + "\n"
    (args.output_dir / "report.json").write_text(rendered)
    print(rendered, end="")
    return int(failed)


if __name__ == "__main__":
    try:
        sys.exit(main())
    except (OSError, ValueError, subprocess.CalledProcessError) as error:
        print(f"bench_host_instructions: {error}", file=sys.stderr)
        sys.exit(2)
