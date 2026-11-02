#!/usr/bin/env python3
"""Measure a live PER service, including startup, while keeping controls open."""
import argparse
import pathlib
import struct
import subprocess
import time


def read_exact(stream, count):
    result = bytearray()
    while len(result) < count:
        chunk = stream.read(count - len(result))
        if not chunk:
            raise RuntimeError('truncated protocol frame')
        result.extend(chunk)
    return result


def main():
    root = pathlib.Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--binary', default=str(root / 'build-pgo/fil'))
    parser.add_argument('--network', default=str(root / 'configs/networks/per_vehicle.json'))
    parser.add_argument('--duration-ms', type=int, default=10000)
    parser.add_argument('--refresh-ms', type=int, default=100)
    parser.add_argument('--reps', type=int, default=3)
    args = parser.parse_args()
    if min(args.duration_ms, args.refresh_ms, args.reps) <= 0:
        parser.error('duration, refresh, and reps must be positive')
    for repetition in range(args.reps):
        command = [args.binary, 'serve-network', args.network, '--transport', 'stdio',
                   '--duration-ms', str(args.duration_ms), '--refresh-ms', str(args.refresh_ms),
                   '--no-wall-pacing', '--strict-mmio', '--adc-decimation', '1']
        started = time.monotonic()
        # Popen's stdin stays open; communicate() would close the control channel.
        with subprocess.Popen(command, stdin=subprocess.PIPE, stdout=subprocess.PIPE) as process:
            hello = False
            traces = 0
            end = None
            try:
                while True:
                    header = process.stdout.read(16)
                    if not header:
                        break
                    if len(header) < 16:
                        header += read_exact(process.stdout, 16 - len(header))
                    magic, version, kind, flags, length, request = struct.unpack('<4sBBHII', header)
                    if magic != b'FILN' or version != 2 or flags or length > 65536 or request:
                        raise RuntimeError('invalid unsolicited frame header')
                    payload = read_exact(process.stdout, length)
                    if kind == 0x80:
                        name_size, = struct.unpack_from('<H', payload)
                        boards, buses = struct.unpack_from('<II', payload, 2 + name_size)
                        if hello or boards != 6 or buses != 2:
                            raise RuntimeError('expected one six-board, two-bus PER HELLO')
                        hello = True
                    elif kind == 0x82:
                        traces += 1
                    elif kind == 0x83:
                        if end is not None:
                            raise RuntimeError('duplicate END')
                        end = struct.unpack_from('<QBB', payload)
                    else:
                        raise RuntimeError(f'unexpected frame kind {kind}')
                code = process.wait()
            except BaseException:
                process.kill()
                process.wait()
                raise
        elapsed = time.monotonic() - started
        expected_end = (args.duration_ms * 1000000, 2, 0)
        if code or not hello or not traces or end != expected_end:
            raise RuntimeError(f'incomplete service: exit={code}, hello={hello}, traces={traces}, end={end}')
        ratio = args.duration_ms / 1000 / elapsed
        print(f'run {repetition + 1}: {elapsed:.3f} s wall, {ratio:.3f}x real time, '
              f'{traces} trace frames, duration END verified', flush=True)
        if ratio <= 1:
            raise RuntimeError('service did not exceed real time')


if __name__ == '__main__':
    main()
