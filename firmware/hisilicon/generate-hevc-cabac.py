#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-only
"""Generate CV200 HEVC CABAC bytes from the BSP s_InitValue object."""
import argparse
import re
from pathlib import Path

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('hal', type=Path, help='CV200 vdm_hal_hevc.S from the BSP')
parser.add_argument('output', type=Path, help='hardware-ready histb-hevc-cabac.bin')
args = parser.parse_args()
source = args.hal.read_text()
match = re.search(r'^s_InitValue:\s*\n((?:[ \t]*\.word[ \t]+-?\d+[ \t]*\n){480})', source, re.MULTILINE)
if not match:
    parser.error('expected the 480-word CV200 s_InitValue object')
values = [int(value) for value in re.findall(r'\.word\s+(-?\d+)', match.group(1))]
result = bytearray()
for group in range(3):
    for context in range(16 if group == 0 else 0, 160):
        value = values[group * 160 + context]
        if value == -1:
            slope, offset = 0, 0
        elif 0 <= value <= 255:
            slope = (value >> 4) * 5 - 45
            offset = (value & 15) * 8 - 16
        else:
            parser.error(f'invalid CABAC init value {value}')
        result.extend((offset & 255, slope & 255))
args.output.write_bytes(result)
print(f'Wrote {len(result)} bytes to {args.output}')
