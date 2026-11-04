"""Callgrind accounting regressions; run with python3 -B -m unittest discover -s tests/python."""

import importlib.util
from pathlib import Path
import unittest


SCRIPT = Path(__file__).resolve().parents[2] / "tools" / "bench_host_instructions.py"
SPEC = importlib.util.spec_from_file_location("bench_host_instructions", SCRIPT)
BENCH = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(BENCH)


PROFILE = """positions: line
events: Ir
summary: 1000
ob=(1) fil
fn=(1) fil::cpu::CortexM4::executeNativeTrace()
cob=(2) ???
cfn=(2) 0x1000
calls=2 0
0 200
ob=(2)
fn=(2)
0 80
ob=(1)
fn=(3) driver()
cfn=(4) fil::cpu::MicroJit::compileNativeTrace()
calls=1 0
0 100
fn=(4)
0 100
"""


class GuestCounterParsingTest(unittest.TestCase):
    REPORT = """network: per_vehicle
  stop:                  time-budget
  boards:                2
  instructions:          815098
  cycles:                1920004
  time:                  20.000 ms (20000000 ns)

  board: dashboard
    stop:         time-budget
    instructions: 132339
    pc:           0x0800670a
    clock:        16000000 Hz
    jit:
      native: compilations=85 executions=421 instructions=129075
      single: fast=2 generic=1

  board: main_module
    stop:         time-budget
    instructions: 134342
    pc:           0x080040ca
    clock:        16000000 Hz
    jit:
      native: compilations=47 executions=402 instructions=129755
"""

    def test_report_groups_map_to_flat_guest_counters(self):
        counters, boards = BENCH.parse_guest_counters(self.REPORT)
        self.assertEqual(counters, {
            "instructions": 815098, "cycles": 1920004, "time_ns": 20000000})
        self.assertEqual(boards, {"dashboard": 129075, "main_module": 129755})

    def test_hidden_jit_stats_still_fail_closed(self):
        without_stats = "\n".join(
            line for line in self.REPORT.splitlines()
            if not line.lstrip().startswith(("jit:", "native:", "single:")))
        with self.assertRaisesRegex(ValueError, "no valid JIT-executed guest"):
            BENCH.parse_guest_counters(without_stats)


class HostInstructionAccountingTest(unittest.TestCase):
    def test_object_dictionary_can_be_defined_by_callee(self):
        result = BENCH.parse_callgrind(PROFILE)
        self.assertEqual(result["kernel_and_helpers_host_instructions"], 200)
        self.assertEqual(result["compilation_host_instructions"], 100)
        self.assertEqual(result["host_instructions"], 300)
        self.assertEqual(result["kernel_calls"], 2)
        self.assertEqual(result["compilation_calls"], 1)
        self.assertEqual(result["process_host_instructions"], 1000)

    def test_object_dictionary_defined_by_caller_is_equivalent(self):
        defined_by_ob = PROFILE.replace("ob=(1) fil", "ob=(2) ???\nob=(1) fil", 1)
        self.assertEqual(BENCH.parse_callgrind(PROFILE), BENCH.parse_callgrind(defined_by_ob))

    def test_callee_object_does_not_change_current_object(self):
        # The binary's numeric symbol must not be mistaken for generated code
        # simply because its preceding call introduced the anonymous object.
        profile = PROFILE.replace("ob=(2)\nfn=(2)\n0 80", """fn=(5) 0x2000
0 10
ob=(1)
fn=(1)
cob=(1)
cfn=(5)
calls=1 0
0 90
ob=(2)
fn=(2)
0 80""")
        self.assertEqual(BENCH.parse_callgrind(profile)["host_instructions"], 300)

    def test_inclusive_kernel_helpers_are_not_counted_twice(self):
        profile = PROFILE.replace("fn=(2)\n0 80", """fn=(2)
0 80
cob=(1)
cfn=(6) helper()
calls=2 0
0 120
ob=(1)
fn=(6)
0 120""")
        self.assertEqual(BENCH.parse_callgrind(profile)["host_instructions"], 300)

    def test_missing_kernel_roots_still_fail_closed(self):
        with self.assertRaisesRegex(ValueError, "missing JIT entry"):
            BENCH.parse_callgrind(PROFILE.replace("0x1000", "named_binary_function"))

    def test_costs_cannot_exceed_the_process_total(self):
        with self.assertRaisesRegex(ValueError, "exceed process total"):
            BENCH.parse_callgrind(PROFILE.replace("summary: 1000", "summary: 299"))

    def test_nested_inclusive_roots_still_fail_closed(self):
        profile = PROFILE.replace("fn=(2)\n0 80", """fn=(2)
0 80
cob=(1)
cfn=(3) driver()
calls=1 0
0 100""")
        with self.assertRaisesRegex(ValueError, "overlapping inclusive"):
            BENCH.parse_callgrind(profile)


if __name__ == "__main__":
    unittest.main()
