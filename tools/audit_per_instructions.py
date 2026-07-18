#!/usr/bin/env python3
"""Audit every instruction recognized by objdump against fil's Thumb decoder.

The script deliberately uses objdump as the code/data boundary authority. It
groups contiguous instruction lines and asks `fil disasm-window` to decode the
same addresses, then verifies address, raw halfwords, width, and supported status.
No expected instruction counts are embedded; an optional JSON report records the
counts observed from the supplied ELF artifacts.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import hashlib
import json
from pathlib import Path
import re
import subprocess
import sys
from typing import Iterable, Sequence


HALFWORD = re.compile(r"[0-9a-fA-F]{4}")
ADDRESS = re.compile(r"[0-9a-fA-F]+:")


@dataclass(frozen=True)
class ObjdumpInstruction:
    address: int
    halfwords: tuple[str, ...]
    mnemonic: str

    @property
    def size(self) -> int:
        return len(self.halfwords) * 2


def run_checked(command: Sequence[str]) -> str:
    completed = subprocess.run(
        command,
        check=False,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
    )
    if completed.returncode != 0:
        rendered = " ".join(command)
        raise RuntimeError(
            f"command failed with exit {completed.returncode}: {rendered}\n"
            f"stdout:\n{completed.stdout}\n"
            f"stderr:\n{completed.stderr}"
        )
    return completed.stdout


def objdump_instructions(objdump: str, elf: Path) -> list[ObjdumpInstruction]:
    output = run_checked([objdump, "-d", str(elf)])
    instructions: list[ObjdumpInstruction] = []
    seen_addresses: set[int] = set()
    for line in output.splitlines():
        fields = line.split("\t")
        if len(fields) < 3:
            continue
        address_text = fields[0].strip()
        if ADDRESS.fullmatch(address_text) is None:
            continue
        raw_words = fields[1].split()
        if len(raw_words) not in (1, 2) or any(HALFWORD.fullmatch(word) is None for word in raw_words):
            # Data directives are normally emitted as one eight-hex-digit word.
            continue
        assembly = fields[2].strip()
        if not assembly:
            continue
        mnemonic = assembly.split()[0]
        if mnemonic.startswith("."):
            continue
        address = int(address_text[:-1], 16)
        if address in seen_addresses:
            raise RuntimeError(f"objdump emitted duplicate instruction address 0x{address:08x} for {elf}")
        seen_addresses.add(address)
        instructions.append(
            ObjdumpInstruction(
                address=address,
                halfwords=tuple(word.lower() for word in raw_words),
                mnemonic=mnemonic,
            )
        )
    if not instructions:
        raise RuntimeError(f"objdump found no instructions in {elf}")
    return instructions


def contiguous_chunks(
    instructions: Sequence[ObjdumpInstruction], maximum: int
) -> Iterable[Sequence[ObjdumpInstruction]]:
    start = 0
    while start < len(instructions):
        end = start + 1
        while (
            end < len(instructions)
            and end - start < maximum
            and instructions[end].address
            == instructions[end - 1].address + instructions[end - 1].size
        ):
            end += 1
        yield instructions[start:end]
        start = end


def parse_fil_line(line: str) -> tuple[int, tuple[str, ...], str]:
    fields = line.split()
    if len(fields) < 3 or not fields[0].startswith("0x") or not fields[0].endswith(":"):
        raise RuntimeError(f"unexpected fil disassembly line: {line!r}")
    address = int(fields[0][2:-1], 16)
    index = 1
    halfwords: list[str] = []
    while index < len(fields) and len(halfwords) < 2 and HALFWORD.fullmatch(fields[index]):
        halfwords.append(fields[index].lower())
        index += 1
    if not halfwords or index >= len(fields):
        raise RuntimeError(f"missing raw encoding or mnemonic in fil output: {line!r}")
    return address, tuple(halfwords), fields[index]


def audit_elf(fil: str, objdump: str, elf: Path, maximum_chunk: int) -> dict[str, object]:
    expected = objdump_instructions(objdump, elf)
    chunks = 0
    decoded_count = 0
    failures: list[str] = []
    for chunk in contiguous_chunks(expected, maximum_chunk):
        chunks += 1
        output = run_checked(
            [
                fil,
                "disasm-window",
                str(elf),
                "--addr",
                f"0x{chunk[0].address:08x}",
                "--count",
                str(len(chunk)),
            ]
        )
        actual_lines = [line for line in output.splitlines() if line.strip()]
        if len(actual_lines) != len(chunk):
            failures.append(
                f"0x{chunk[0].address:08x}: expected {len(chunk)} decoded lines, got {len(actual_lines)}"
            )
            continue
        for objdump_instruction, actual_line in zip(chunk, actual_lines):
            address, halfwords, mnemonic = parse_fil_line(actual_line)
            if address != objdump_instruction.address:
                failures.append(
                    f"0x{objdump_instruction.address:08x}: fil advanced to 0x{address:08x}"
                )
                continue
            if halfwords != objdump_instruction.halfwords:
                failures.append(
                    f"0x{address:08x}: raw encoding differs: objdump "
                    f"{' '.join(objdump_instruction.halfwords)}, fil {' '.join(halfwords)}"
                )
                continue
            if mnemonic == "unsupported":
                failures.append(
                    f"0x{address:08x}: unsupported objdump mnemonic {objdump_instruction.mnemonic} "
                    f"({' '.join(halfwords)})"
                )
                continue
            decoded_count += 1
    if failures:
        details = "\n".join(failures[:50])
        suffix = "" if len(failures) <= 50 else f"\n... {len(failures) - 50} additional failures"
        raise RuntimeError(f"instruction audit failed for {elf}:\n{details}{suffix}")
    digest = hashlib.sha256()
    with elf.open("rb") as elf_stream:
        for block in iter(lambda: elf_stream.read(1024 * 1024), b""):
            digest.update(block)
    return {
        "name": elf.stem,
        "path": str(elf.resolve()),
        "sha256": digest.hexdigest(),
        "objdump_instructions": len(expected),
        "decoded_instructions": decoded_count,
        "contiguous_chunks": chunks,
        "unsupported": 0,
        "address_or_width_mismatches": 0,
    }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--fil", required=True, help="path to the fil executable")
    parser.add_argument("--objdump", default="arm-none-eabi-objdump", help="ARM objdump executable")
    parser.add_argument("--report", type=Path, help="optional JSON report output")
    parser.add_argument("--maximum-chunk", type=int, default=4096)
    parser.add_argument("elf", nargs="+", type=Path)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.maximum_chunk <= 0:
        raise RuntimeError("--maximum-chunk must be positive")
    reports: list[dict[str, object]] = []
    for elf in args.elf:
        if not elf.is_file():
            raise RuntimeError(f"ELF does not exist: {elf}")
        report = audit_elf(args.fil, args.objdump, elf, args.maximum_chunk)
        reports.append(report)
        print(
            f"{report['name']}: {report['decoded_instructions']} objdump instructions decoded, "
            "0 unsupported, 0 address/width mismatches"
        )
    summary = {
        "schema_version": 1,
        "fil_version": run_checked([args.fil, "--version"]).strip(),
        "objdump_version": run_checked([args.objdump, "--version"]).splitlines()[0],
        "elf_count": len(reports),
        "total_objdump_instructions": sum(int(report["objdump_instructions"]) for report in reports),
        "total_decoded_instructions": sum(int(report["decoded_instructions"]) for report in reports),
        "unsupported": 0,
        "address_or_width_mismatches": 0,
        "elfs": reports,
    }
    if args.report is not None:
        args.report.parent.mkdir(parents=True, exist_ok=True)
        args.report.write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")
    print(
        f"total: {summary['total_decoded_instructions']} instructions across "
        f"{summary['elf_count']} ELFs"
    )
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except (OSError, RuntimeError, ValueError) as error:
        print(f"audit_per_instructions.py: {error}", file=sys.stderr)
        sys.exit(1)
