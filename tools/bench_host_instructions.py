#!/usr/bin/env python3
"""Measure dynamic JIT host instructions per JIT-executed guest instruction.

Includes generated code, its transitive helper calls and amortized MicroJit
compilation API costs. Excludes surrounding dispatch, scheduling and interpreter
work. Use matched portable Release builds (FIL_ENABLE_NATIVE=OFF) with symbols
retained so Callgrind can identify the kernel callers and compilation APIs.
"""

import argparse
from collections import Counter
import json
from pathlib import Path
import re
import shutil
import subprocess
import sys


COMPILE_API = re.compile(r"^fil::cpu::MicroJit::(?:compile|compileNative|compileNativeTrace)\(")
ADDRESS_NAME = re.compile(r"^0x[0-9a-fA-F]+$")
METRIC = "(dynamic JIT kernel inclusive Ir + compilation API inclusive Ir) / JIT-executed guests"


def positive_int(value):
    number = int(value)
    if number <= 0:
        raise argparse.ArgumentTypeError("must be positive")
    return number


def parse_callgrind(data):
    """Read inclusive call-edge costs, not a sum of inclusive function totals.

    Entry-edge costs already include all descendants, including anonymous native
    subroutines and C++ helpers. Adding those descendants again double counts.
    Function/object name IDs are separate Callgrind compression dictionaries.
    """
    names, objects, function_objects = {}, {}, {}
    costs, calls = Counter(), Counter()
    current = callee = current_object = None
    pending_calls = None
    events = positions = summary = None

    def expand(value, dictionary):
        match = re.fullmatch(r"\((\d+)\)(?: (.*))?", value)
        if not match:
            return value
        key, name = match.groups()
        if name is not None:
            dictionary[key] = name
        return dictionary.get(key) if dictionary is objects else key

    for line in data.splitlines():
        line = line.strip()
        if not line or line.startswith("#"):
            continue
        if line.startswith("events:"):
            events = line.split()[1:]
        elif line.startswith("positions:"):
            positions = line.split()[1:]
        elif line.startswith("summary:"):
            summary = [int(value) for value in line.split()[1:]]
        elif line.startswith("ob="):
            current_object = expand(line[3:], objects)
        elif line.startswith("fn="):
            current = expand(line[3:], names)
            function_objects[current] = current_object
        elif line.startswith("cfn="):
            callee = expand(line[4:], names)
        elif line.startswith("calls="):
            if pending_calls is not None or current is None or callee is None:
                raise ValueError("malformed Callgrind call edge")
            pending_calls = int(line[6:].split()[0])
        elif re.match(r"^[0-9*+\-]", line) and pending_calls is not None:
            if not events or not positions or "Ir" not in events:
                raise ValueError("Callgrind positions/events must precede costs")
            fields = line.split()
            event_index = len(positions) + events.index("Ir")
            # Callgrind can omit trailing zero event costs.
            ir = int(fields[event_index]) if len(fields) > event_index else 0
            edge = (current, callee)
            costs[edge] += ir
            calls[edge] += pending_calls
            pending_calls = None
    if pending_calls is not None or not events or "Ir" not in events or summary is None:
        raise ValueError("missing or incomplete Callgrind Ir summary")
    if len(events) != len(summary):
        raise ValueError("Callgrind event/summary length mismatch")

    def name(key):
        return names.get(key, key)

    def is_kernel(key):
        return function_objects.get(key) == "???" and ADDRESS_NAME.fullmatch(name(key))

    def is_compiler(key):
        value = name(key)
        return COMPILE_API.match(value) and "::{" not in value

    kernel_edges = {edge for edge in costs
                    if "fil::cpu::CortexM4::" in name(edge[0]) and is_kernel(edge[1])}
    compiler_edges = {edge for edge in costs if is_compiler(edge[1]) and not is_compiler(edge[0])}
    if not kernel_edges or not compiler_edges:
        raise ValueError("missing JIT entry or compilation call edges; retain symbols and profile from startup")

    # Fail closed if selected inclusive roots can nest. Aggregated Callgrind
    # data cannot safely de-duplicate context-dependent overlapping root costs.
    roots = kernel_edges | compiler_edges
    graph = {}
    for caller, target in costs:
        graph.setdefault(caller, set()).add(target)
    root_callers = {edge[0] for edge in roots}
    for root in {edge[1] for edge in roots}:
        todo, seen = [root], set()
        while todo:
            node = todo.pop()
            if node in seen:
                continue
            seen.add(node)
            if node in root_callers and any((node, target) in roots for target in graph.get(node, ())):
                raise ValueError("overlapping inclusive JIT/compilation roots in Callgrind profile")
            todo.extend(graph.get(node, set()) - seen)

    kernel_ir = sum(costs[edge] for edge in kernel_edges)
    emission_ir = sum(costs[edge] for edge in compiler_edges)
    process_ir = summary[events.index("Ir")]
    if kernel_ir + emission_ir > process_ir:
        raise ValueError("selected JIT instruction costs exceed process total")
    return {
        "kernel_and_helpers_host_instructions": kernel_ir,
        "compilation_host_instructions": emission_ir,
        "host_instructions": kernel_ir + emission_ir,
        "kernel_calls": sum(calls[edge] for edge in kernel_edges),
        "compilation_calls": sum(calls[edge] for edge in compiler_edges),
        "process_host_instructions": process_ir,
    }


def parse_guest_counters(output):
    counters = {}
    for key in ("instructions", "cycles", "time_ns"):
        match = re.search(rf"^{key}: (\d+)$", output, re.MULTILINE)
        if not match:
            raise ValueError(f"missing {key} in simulator output")
        counters[key] = int(match[1])
    if not re.search(r"^stop: time-budget$", output, re.MULTILINE):
        raise ValueError("run did not reach its time budget")
    boards = {}
    for board, fields in re.findall(r"^jit: board=(\S+) (.*)$", output, re.MULTILINE):
        if board in boards:
            raise ValueError(f"duplicate JIT counters for board {board}")
        values = {key: int(value) for key, value in re.findall(r"(\w+)=(\d+)", fields)}
        required = {"native_instructions", "micro_instructions", "trace_instructions"}
        if not required <= values.keys():
            raise ValueError(f"missing JIT guest counters for board {board}")
        if values["trace_instructions"] > values["native_instructions"]:
            raise ValueError("trace instructions must be a subset of native instructions")
        # Native counters already include traces; never add trace_instructions.
        boards[board] = values["native_instructions"] + values["micro_instructions"]
    guest = sum(boards.values())
    if guest <= 0 or guest > counters["instructions"]:
        raise ValueError("no valid JIT-executed guest instruction count")
    return counters, boards


def measure(binary, args, label):
    prefix = args.output_dir / label
    profile = prefix.with_suffix(".callgrind")
    stdout_path = prefix.with_suffix(".stdout")
    stderr_path = prefix.with_suffix(".stderr")
    if not args.analyze_only:
        command = [
            "valgrind", "--tool=callgrind", f"--callgrind-out-file={profile}",
            str(binary.resolve()), "run-network", str(args.network.resolve()),
            "--duration-ms", str(args.duration_ms), "--max-instructions", "1000000000",
            "--quantum", "1024", "--strict-mmio", "--adc-decimation", "1",
        ]
        with stdout_path.open("w") as stdout, stderr_path.open("w") as stderr:
            subprocess.run(command, stdout=stdout, stderr=stderr, check=True)
    counters, boards = parse_guest_counters(stdout_path.read_text())
    result = parse_callgrind(profile.read_text())
    guest = sum(boards.values())
    result.update({
        "binary": str(binary.resolve()),
        "guest_instructions": guest,
        "jit_guest_instructions_by_board": boards,
        "host_per_guest": result["host_instructions"] / guest,
        "execution_host_per_guest": result["kernel_and_helpers_host_instructions"] / guest,
        "amortized_compilation_host_per_guest": result["compilation_host_instructions"] / guest,
        "counters": counters,
        "profile": str(profile),
    })
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--binary", required=True, type=Path)
    parser.add_argument("--baseline", type=Path)
    parser.add_argument("--network", type=Path, default=Path("configs/networks/per_vehicle.json"))
    parser.add_argument("--duration-ms", type=positive_int, default=100)
    parser.add_argument("--output-dir", type=Path, required=True,
                        help="directory for profiles, stdout, stderr and report.json")
    parser.add_argument("--analyze-only", action="store_true",
                        help="reuse existing current/baseline .callgrind and .stdout artifacts")
    parser.add_argument("--max-host-per-guest", type=float,
                        help="optional JIT-inclusive runtime budget; exits 1 when exceeded")
    args = parser.parse_args()
    if args.max_host_per_guest is not None and not 0 < args.max_host_per_guest < float("inf"):
        parser.error("--max-host-per-guest must be finite and positive")
    if not args.analyze_only and not shutil.which("valgrind"):
        parser.error("valgrind is required")
    args.output_dir = args.output_dir.resolve()
    args.output_dir.mkdir(parents=True, exist_ok=True)
    report = {"metric": METRIC}
    if args.baseline:
        report["baseline"] = measure(args.baseline, args, "baseline")
    report["current"] = measure(args.binary, args, "current")
    if args.baseline:
        before, after = report["baseline"], report["current"]
        if (before["counters"] != after["counters"]
                or before["jit_guest_instructions_by_board"] != after["jit_guest_instructions_by_board"]):
            raise ValueError("baseline/current guest or JIT coverage counters differ; results are not comparable")
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
