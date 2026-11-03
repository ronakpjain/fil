#!/usr/bin/env python3
"""Generate a self-contained fil network config from artifact firmware.

Reads the network + board + MCU configs from a fil checkout (any era: the
checkout's own schema is reused verbatim) and repoints only the firmware and
MCU paths at absolute locations:

  board.elf -> <firmware_dir>/<board_name>/<board_name>.elf
  board.mcu -> absolute path of the checkout's MCU config

Boards whose ELF is missing from the firmware directory are dropped (reported
on stderr); the network's buses/stimuli/scheduling are copied unchanged.

Prints KEY=VALUE lines on stdout:
  NETWORK=<generated network config path>
  BOARDS=<space-separated board names kept>
  DROPPED=<space-separated board names dropped, if any>
  EXPECT_BOARDS=<count kept>

Example:
  python3 tools/make_ci_network.py --repo . \
    --firmware-dir /tmp/per-fw/unpacked-firmware-master-abc/output \
    --out-dir /tmp/ci-net
"""

import argparse
import json
import os
import sys


def fail(message):
    print(f"make_ci_network.py: {message}", file=sys.stderr)
    sys.exit(2)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--repo", required=True,
                        help="fil checkout whose configs are the template")
    parser.add_argument("--network", default="configs/networks/per_vehicle.json",
                        help="network config path relative to --repo")
    parser.add_argument("--firmware-dir", required=True,
                        help="unpacked PER firmware root holding <board>/<board>.elf")
    parser.add_argument("--out-dir", required=True,
                        help="directory for the generated configs")
    args = parser.parse_args()

    network_src = os.path.join(args.repo, args.network)
    if not os.path.isfile(network_src):
        fail(f"network config not found: {network_src}")
    if not os.path.isdir(args.firmware_dir):
        fail(f"firmware dir not found: {args.firmware_dir}")

    with open(network_src) as handle:
        network = json.load(handle)
    network_dir = os.path.dirname(os.path.abspath(network_src))
    os.makedirs(args.out_dir, exist_ok=True)

    kept = []
    dropped = []
    board_refs = network.get("boards", [])
    for ref in board_refs:
        board_src = os.path.normpath(os.path.join(network_dir, ref))
        if not os.path.isfile(board_src):
            print(f"warning: board config missing: {board_src}", file=sys.stderr)
            dropped.append(ref)
            continue
        with open(board_src) as handle:
            board = json.load(handle)
        name = board.get("name", os.path.splitext(os.path.basename(board_src))[0])
        elf = os.path.join(args.firmware_dir, name, name + ".elf")
        if not os.path.isfile(elf):
            print(f"warning: no firmware for board '{name}': {elf}",
                  file=sys.stderr)
            dropped.append(name)
            continue
        board["elf"] = elf
        mcu_ref = board.get("mcu")
        if mcu_ref and not os.path.isabs(mcu_ref):
            mcu_abs = os.path.normpath(
                os.path.join(os.path.dirname(os.path.abspath(board_src)),
                             mcu_ref))
            if os.path.isfile(mcu_abs):
                board["mcu"] = mcu_abs
        out_board = os.path.join(args.out_dir, name + ".json")
        with open(out_board, "w") as handle:
            json.dump(board, handle, indent=2)
            handle.write("\n")
        kept.append(name)

    out_network = dict(network)
    out_network["boards"] = [name + ".json" for name in kept]
    out_path = os.path.join(args.out_dir, "network.json")
    with open(out_path, "w") as handle:
        json.dump(out_network, handle, indent=2)
        handle.write("\n")

    print(f"NETWORK={out_path}")
    print(f"BOARDS={' '.join(kept)}")
    if dropped:
        print(f"DROPPED={' '.join(dropped)}")
    print(f"EXPECT_BOARDS={len(kept)}")


if __name__ == "__main__":
    main()
