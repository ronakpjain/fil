"""Tests for alternating, fidelity-matched full-network measurements."""

import contextlib
import importlib.util
import io
import pathlib
import sys
import subprocess
import unittest
from unittest import mock

ROOT = pathlib.Path(__file__).resolve().parents[2]
SPEC = importlib.util.spec_from_file_location("bench_full_network", ROOT / "tools/bench_full_network.py")
BENCH = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(BENCH)


def output(*, board_count=6, board_stop="time-budget", time_ns=1_000_000_000, instructions=60):
    return "\n".join([
        "stop: time-budget", "boards: 6", f"time_ns: {time_ns}",
        f"instructions: {instructions}", "cycles: 120", "event_callbacks: 5",
        *[f"board board{i}: stop={board_stop} instructions=10 pc=0x8000000"
          for i in range(board_count)],
    ])


class FullNetworkValidationTest(unittest.TestCase):
    def run_summary(self, text, **options):
        result = subprocess.CompletedProcess([], 0, stdout=text, stderr="")
        with mock.patch.object(BENCH.subprocess, "run", return_value=result) as run:
            with mock.patch.object(BENCH.time, "perf_counter", side_effect=[10.0, 10.5]):
                measured = BENCH.run(pathlib.Path("/tmp/fil"), 1000, **options)
        return measured, run.call_args

    def test_full_rate_and_completed_boards_are_required(self):
        (wall, summary), call = self.run_summary(output(), no_loop_batching=True)
        self.assertEqual(wall, 0.5)
        self.assertEqual(len(summary["board_results"]), 6)
        command = call.args[0]
        self.assertEqual(command[command.index("--adc-decimation") + 1], "1")
        self.assertIn("--no-loop-batching", command)
        self.assertEqual(call.kwargs["stdin"], subprocess.DEVNULL)
        self.assertTrue(call.kwargs["check"])

    def test_incomplete_or_inconsistent_runs_are_not_throughput_evidence(self):
        for text in [output(board_count=5), output(board_stop="instruction-budget"),
                     output(time_ns=999_999_999), output(instructions=61),
                     output() + "\nboard board0: stop=time-budget instructions=10 pc=0x8000000"]:
            with self.subTest(text=text):
                with self.assertRaises(RuntimeError):
                    self.run_summary(text)

    def test_experimental_modes_keep_full_rate_and_required_flags(self):
        (_, _), call = self.run_summary(output(), ram_capsules=True)
        self.assertIn("--no-loop-batching", call.args[0])
        self.assertIn("--ram-capsules", call.args[0])
        (_, _), call = self.run_summary(output(), jit=False, ram_capsules=True)
        self.assertNotIn("--ram-capsules", call.args[0])
        self.assertNotIn("--jit", call.args[0])


class FullNetworkBenchmarkTest(unittest.TestCase):
    def invoke(self, arguments, runner):
        with mock.patch.object(sys, "argv", ["bench_full_network.py", *arguments]), \
                mock.patch.object(BENCH, "run", side_effect=runner), \
                contextlib.redirect_stdout(io.StringIO()) as output:
            result = BENCH.main()
        return result, output.getvalue()

    def test_reference_alternates_order_and_matches_options(self):
        calls = []

        def run(binary, duration, **options):
            calls.append((binary.name, duration, options))
            return (2.0 if binary.name == "reference" else 1.0), {
                "time_ns": "5000000000", "instructions": "42"}

        result, output = self.invoke([
            "--binary", "candidate", "--reference-binary", "reference",
            "--reps", "2", "--no-loop-batching"], run)
        self.assertEqual(result, 0)
        self.assertEqual([call[0] for call in calls],
                         ["reference", "candidate", "candidate", "reference"])
        self.assertTrue(all(call[1] == 5000 and call[2]["no_loop_batching"] for call in calls))
        self.assertIn("candidate speedup: 2.000x", output)

    def test_mismatched_results_are_rejected(self):
        def run(binary, duration, **options):
            return 1.0, {"time_ns": "5000000000", "instructions": binary.name}

        with self.assertRaisesRegex(RuntimeError, "Reference/candidate"):
            self.invoke(["--binary", "candidate", "--reference-binary", "reference"], run)

    def test_below_realtime_still_fails_even_when_faster_than_reference(self):
        def run(binary, duration, **options):
            return (9.0 if binary.name == "reference" else 6.0), {
                "time_ns": "5000000000", "instructions": "42"}

        result, output = self.invoke([
            "--binary", "candidate", "--reference-binary", "reference", "--reps", "1"], run)
        self.assertEqual(result, 1)
        self.assertIn("candidate speedup: 1.500x", output)
        self.assertIn("throughput: 0.833x realtime", output)


if __name__ == "__main__":
    unittest.main()
