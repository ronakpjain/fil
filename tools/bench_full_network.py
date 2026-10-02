#!/usr/bin/env python3
"""Measure the full PER network with JIT and ADC decimation 1.

Use a Release PGO-built binary. Timings include startup and JIT compilation.
An optional real-firmware trace comparison checks interpreter/JIT equivalence.
"""

import argparse
import filecmp
import pathlib
import re
import statistics
import subprocess
import tempfile
import time

ROOT = pathlib.Path(__file__).resolve().parents[1]
NETWORK = "configs/networks/per_vehicle.json"


def run(binary, duration_ms, *, jit=True, trace=None, no_loop_batching=False,
        deferred_prefixes=False, ram_capsules=False):
    command = [
        str(binary), "run-network", NETWORK,
        "--duration-ms", str(duration_ms),
        "--max-instructions", "1000000000", "--quantum", "1024",
        "--strict-mmio", "--adc-decimation", "1",
    ]
    if jit:
        command.append("--jit")
    if no_loop_batching or ram_capsules:
        command.append("--no-loop-batching")
    if deferred_prefixes and jit:
        command.append("--deferred-prefixes")
    if ram_capsules and jit:
        command.append("--ram-capsules")
    if trace is not None:
        command.extend(["--trace", str(trace)])
    start = time.perf_counter()
    result = subprocess.run(command, cwd=ROOT, stdin=subprocess.DEVNULL,
                            capture_output=True, text=True, check=True)
    wall = time.perf_counter() - start
    summary = {}
    boards = {}
    for line in result.stdout.splitlines():
        key, separator, value = line.partition(": ")
        if separator and key in {"stop", "boards", "time_ns", "instructions",
                                 "cycles", "event_callbacks"}:
            summary[key] = value
        board = re.fullmatch(r"board (.+): stop=([a-z-]+) instructions=(\d+) pc=(0x[\da-fA-F]+)", line)
        if board:
            name, reason, instructions, pc = board.groups()
            if name in boards:
                raise RuntimeError(f"Duplicate board result: {name}")
            boards[name] = (reason, int(instructions), int(pc, 16))
    if (summary.get("stop") != "time-budget" or summary.get("boards") != "6"
            or len(boards) != 6 or any(board[0] != "time-budget" for board in boards.values())):
        raise RuntimeError(f"Incomplete full-network run: {summary}; boards={boards}\n{result.stderr}")
    if int(summary["time_ns"]) < duration_ms * 1_000_000:
        raise RuntimeError(f"Run stopped before the requested duration: {summary}")
    if sum(board[1] for board in boards.values()) != int(summary["instructions"]):
        raise RuntimeError("Board instruction counts do not match the network total")
    summary["board_results"] = boards
    return wall, summary


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--binary", type=pathlib.Path, default=ROOT / "build-pgo" / "fil")
    parser.add_argument("--reference-binary", type=pathlib.Path,
                        help="time a matched baseline in alternating run order")
    parser.add_argument("--duration-ms", type=int, default=5000)
    parser.add_argument("--reps", type=int, default=3)
    parser.add_argument("--check-trace", action="store_true",
                        help="compare 1,000 ms interpreter/JIT traces before timing")
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--no-loop-batching", action="store_true",
                      help="use the compact exact single-instruction scheduler")
    mode.add_argument("--deferred-prefixes", action="store_true",
                      help="use experimental interruptible pure JIT prefixes")
    mode.add_argument("--ram-capsules", action="store_true",
                      help="use experimental private-RAM JIT spans (disables loop batching)")
    args = parser.parse_args()
    if args.duration_ms <= 0 or args.reps <= 0:
        parser.error("duration and repetitions must be positive")
    binary = args.binary.resolve()
    if args.check_trace:
        with tempfile.TemporaryDirectory(prefix="fil-network-check-") as directory:
            reference = pathlib.Path(directory) / "interpreter.jsonl"
            candidate = pathlib.Path(directory) / "jit.jsonl"
            _, reference_summary = run(binary, 1000, jit=False, trace=reference, no_loop_batching=True)
            _, candidate_summary = run(binary, 1000, trace=candidate,
                                       no_loop_batching=args.no_loop_batching,
                                       deferred_prefixes=args.deferred_prefixes,
                                       ram_capsules=args.ram_capsules)
            if reference_summary != candidate_summary:
                raise RuntimeError("Full-network interpreter/JIT counters or final board PCs differ")
            if not filecmp.cmp(reference, candidate, shallow=False):
                raise RuntimeError("Full-network interpreter/JIT traces differ")
        print("1,000 ms full-network traces, counters and final board PCs: identical", flush=True)
    times = []
    reference_times = []
    reference_binary = args.reference_binary.resolve() if args.reference_binary else None
    for repetition in range(args.reps):
        binaries = [("candidate", binary, times)]
        if reference_binary is not None:
            binaries.append(("reference", reference_binary, reference_times))
            if repetition % 2 == 0:
                binaries.reverse()
        summaries = []
        for label, executable, measurements in binaries:
            wall, summary = run(executable, args.duration_ms,
                                no_loop_batching=args.no_loop_batching,
                                deferred_prefixes=args.deferred_prefixes,
                                ram_capsules=args.ram_capsules)
            measurements.append(wall)
            summaries.append(summary)
            print(f"Run {repetition + 1} ({label}): wall={wall:.3f}s "
                  f"simulated={int(summary['time_ns']) / 1e9:.9f}s "
                  f"instructions={summary['instructions']}", flush=True)
        if len(summaries) == 2 and summaries[0] != summaries[1]:
            raise RuntimeError("Reference/candidate counters or final board PCs differ")
    median = statistics.median(times)
    throughput = args.duration_ms / 1000 / median
    if reference_times:
        reference_median = statistics.median(reference_times)
        print(f"Reference median: {reference_median:.3f}s; "
              f"candidate speedup: {reference_median / median:.3f}x", flush=True)
    print(f"ADC decimation: 1; all six boards reached the time budget")
    print(f"Median: {median:.3f}s; throughput: {throughput:.3f}x realtime")
    return 0 if median < args.duration_ms / 1000 else 1


if __name__ == "__main__":
    raise SystemExit(main())
