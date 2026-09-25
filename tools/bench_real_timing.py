#!/usr/bin/env python3
"""Real-timing benchmark for fil (no PER checkout required).

Generates hand-assembled Thumb ELF fixtures, runs fixed-simulation-time
workloads with the given fil binary, and reports instructions / cycles /
CPI / wall time / throughput (sim-s per wall-s).

Workloads:
  idle16  - 4x NOP + B loop in flash at reset 16 MHz, LATENCY=0.
            5 instr / 8 cycles per iteration (B pays +2 taken refill).
  pll170  - firmware programs FLASH_ACR=0x304 (LATENCY=4, PRFTEN, ICEN)
            then a 170 MHz HSE-PLL, and spins the same loop. Each
            iteration pays 4 pipeline + 4 ART-miss stall on the taken
            branch (conservative prefetch-only ART: no I-cache hits).

Each workload runs single-board and as a synthetic 6-board network, with
loop batching on and off (--no-loop-batching measures the interpreter).

Usage:
  python3 tools/bench_real_timing.py ./build-release/fil [--reps N]
"""

import json
import os
import struct
import subprocess
import sys
import tempfile
import time

FLASH_BASE = 0x08000000
SRAM_TOP = 0x20020000

NOP = 0xBF00


def ldr_literal(rt, target_addr, instr_addr):
    pc = (instr_addr + 4) & ~3
    if (target_addr - pc) % 4 != 0 or target_addr < pc:
        raise ValueError("bad literal placement")
    imm8 = (target_addr - pc) // 4
    assert imm8 < 256
    return 0x4800 | (rt << 8) | imm8


def str_imm(rt, rn):
    return 0x6000 | (rn << 3) | rt


def movs(rd, imm8):
    return 0x2000 | (rd << 8) | imm8


def b_to(target, instr_addr):
    off = target - (instr_addr + 4)
    assert off % 2 == 0 and -2048 <= off <= 2046
    return 0xE000 | ((off >> 1) & 0x7FF)


def build_image(code_halfwords, reset_offset=8):
    """Minimal ELF32-LE ARM executable: vector table + code, one PT_LOAD."""
    table = [SRAM_TOP, FLASH_BASE + reset_offset + 1]
    blob = struct.pack("<2I", *table)
    assert reset_offset >= len(blob)
    blob += b"\x00" * (reset_offset - len(blob))
    for h in code_halfwords:
        blob += struct.pack("<H", h)
    e_ident = b"\x7fELF" + bytes([1, 1, 1, 0]) + b"\x00" * 8
    ehdr = e_ident + struct.pack(
        "<HHIIIIIHHHHHH", 2, 40, 1, 0, 52, 0, 0, 52, 32, 1, 0, 0, 0
    )
    phdr = struct.pack(
        "<IIIIIIII", 1, 52 + 32, FLASH_BASE, FLASH_BASE, len(blob), len(blob), 5, 4
    )
    return ehdr + phdr + blob


def idle16_code():
    loop = 0
    code = []
    base = FLASH_BASE + 8
    for _ in range(4):
        code.append(NOP)
    b_addr = base + len(code) * 2
    code.append(b_to(FLASH_BASE + 8 + loop, b_addr))
    return code


def pll170_code():
    setup_count, loop_count = 10, 6
    code_base = FLASH_BASE + 8
    lit_base = code_base + (setup_count + loop_count) * 2

    L_ACR, L_ACRV, L_PLL, L_PLLV, L_CFGR = (lit_base + 4 * i for i in range(5))
    code = []
    a = code_base
    code.append(ldr_literal(0, L_ACR, a)); a += 2
    code.append(ldr_literal(1, L_ACRV, a)); a += 2
    code.append(str_imm(1, 0)); a += 2
    code.append(ldr_literal(0, L_PLL, a)); a += 2
    code.append(ldr_literal(1, L_PLLV, a)); a += 2
    code.append(str_imm(1, 0)); a += 2
    code.append(ldr_literal(0, L_CFGR, a)); a += 2
    code.append(movs(1, 3)); a += 2
    code.append(str_imm(1, 0)); a += 2
    code.append(NOP); a += 2  # padding to keep literal alignment simple
    assert a == code_base + setup_count * 2
    loop_start = a
    for _ in range(4):
        code.append(NOP); a += 2
    code.append(b_to(loop_start, a)); a += 2
    code.append(NOP); a += 2  # dead padding: keeps the literal pool 4-aligned
    assert a == lit_base
    lits = [0x40022000, 0x304, 0x4002100C, 0x5533, 0x40021008]
    for v in lits:
        code.append(v & 0xFFFF)
        code.append((v >> 16) & 0xFFFF)
    return code


def write_configs(root, idle_elf, pll_elf):
    mcu = {
        "schema_version": 1, "name": "stm32g474retx",
        "flash_base": "0x08000000", "flash_size": "512K",
        "sram_base": "0x20000000", "sram_size": "128K",
        "ccm_sram_base": "0x10000000", "ccm_sram_size": "32K",
        "hse_hz": 16000000,
    }
    mcu_path = os.path.join(root, "mcu.json")
    with open(mcu_path, "w") as f:
        json.dump(mcu, f)

    def board(name, elf):
        p = os.path.join(root, name + ".json")
        with open(p, "w") as f:
            json.dump({
                "schema_version": 1, "name": name, "mcu": "mcu.json",
                "elf": os.path.basename(elf), "vector_base": "0x08000000",
            }, f)
        return p

    boards = {}
    for name, elf in (("idle", idle_elf), ("pll", pll_elf)):
        dest = os.path.join(root, name + ".elf")
        with open(elf, "rb") as s, open(dest, "wb") as d:
            d.write(s.read())
        boards[name] = board(name, dest)

    six = [board("node%d" % i, idle_elf) for i in range(6)]
    net = os.path.join(root, "six.json")
    with open(net, "w") as f:
        json.dump({"schema_version": 1, "name": "bench-six", "buses": {},
                   "boards": [os.path.basename(p) for p in six]}, f)
    return boards, net


def parse_output(text):
    vals = {}
    for line in text.splitlines():
        if ":" in line:
            k, _, v = line.partition(":")
            k, v = k.strip(), v.strip()
            if k in ("instructions", "cycles", "time_ns"):
                vals[k] = int(v)
    return vals


def run_case(fil, args, reps):
    walls, last = [], None
    for _ in range(reps):
        start = time.perf_counter()
        proc = subprocess.run([fil] + args, capture_output=True, text=True)
        walls.append(time.perf_counter() - start)
        if proc.returncode != 0:
            print("COMMAND FAILED:", " ".join([fil] + args))
            print(proc.stdout[-2000:])
            print(proc.stderr[-2000:])
            sys.exit(1)
        last = parse_output(proc.stdout)
    walls.sort()
    return walls[len(walls) // 2], last


def main():
    fil = sys.argv[1] if len(sys.argv) > 1 else "./build-release/fil"
    reps = int(sys.argv[3]) if len(sys.argv) > 3 and sys.argv[2] == "--reps" else 3
    with tempfile.TemporaryDirectory(prefix="fil-bench-") as root:
        idle_elf = os.path.join(root, "idle_gen.elf")
        pll_elf = os.path.join(root, "pll_gen.elf")
        with open(idle_elf, "wb") as f:
            f.write(build_image(idle16_code()))
        with open(pll_elf, "wb") as f:
            f.write(build_image(pll170_code()))
        boards, net = write_configs(root, idle_elf, pll_elf)

        cases = [
            ("idle16 x1", ["run", boards["idle"], "--duration-ms", "1000",
                           "--max-instructions", "200000000", "--strict-mmio"]),
            ("idle16 x1 no-batch", ["run", boards["idle"], "--duration-ms", "1000",
                                    "--max-instructions", "200000000", "--strict-mmio",
                                    "--no-loop-batching"]),
            ("idle16 x6", ["run-network", net, "--duration-ms", "1000",
                           "--max-instructions", "200000000", "--quantum", "1024",
                           "--strict-mmio"]),
            ("idle16 x6 no-batch", ["run-network", net, "--duration-ms", "1000",
                                    "--max-instructions", "200000000", "--quantum", "1024",
                                    "--strict-mmio", "--no-loop-batching"]),
            ("pll170 x1", ["run", boards["pll"], "--duration-ms", "1000",
                           "--max-instructions", "400000000", "--strict-mmio"]),
            ("pll170 x1 no-batch", ["run", boards["pll"], "--duration-ms", "1000",
                                    "--max-instructions", "400000000", "--strict-mmio",
                                    "--no-loop-batching"]),
        ]
        print("%-20s %12s %12s %6s %12s %10s %9s" % (
            "case", "instructions", "cycles", "cpi", "time_ns", "wall_s", "x_realtime"))
        print("model: variable CPI + FLASH_ACR wait states + ART; "
              "deadline overshoot < 1 ms is atomic batch completion")
        ok = True
        for name, args in cases:
            wall, vals = run_case(fil, args, reps)
            ins, cyc, tns = vals["instructions"], vals["cycles"], vals["time_ns"]
            cpi = cyc / ins if ins else 0.0
            thr = (tns / 1e9) / wall if wall > 0 else 0.0
            print("%-20s %12d %12d %6.2f %12d %10.3f %9.2f" % (
                name, ins, cyc, cpi, tns, wall, thr))
            if cyc < ins:
                print("  FAIL: cycles < instructions")
                ok = False
            # Runs stop at the first atomic boundary at/after the deadline.
            if not 1_000_000_000 <= tns <= 1_001_000_000:
                print("  FAIL: run missed the 1 s deadline window: %d" % tns)
                ok = False
        sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
