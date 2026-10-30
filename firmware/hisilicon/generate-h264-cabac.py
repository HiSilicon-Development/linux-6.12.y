#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-only
"""Generate the CV200 H.264 CABAC table from the VFMW syntax assembly."""
import argparse
import re
from pathlib import Path

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('syntax', type=Path, help='CV200 32-bit core/syntax/h264.S')
parser.add_argument('output', type=Path, help='hardware-ready histb-h264-cabac.bin')
args = parser.parse_args()
source = args.syntax.read_text()
match = re.search(
    r'^g_CabacMN:\s*\n((?:[ \t]*\.word[ \t]+-?\d+[ \t]*\n){1280})',
    source, re.MULTILINE)
if not match:
    parser.error('expected the 1280-word CV200 g_CabacMN object')
values = [int(value) for value in re.findall(r'\.word\s+(-?\d+)', match.group(1))]
if any(value < -(1 << 31) or value > 0xffffffff for value in values):
    parser.error('CABAC table value is outside the 32-bit range')
result = b''.join((value & 0xffffffff).to_bytes(4, 'little') for value in values)
args.output.write_bytes(result)
print(f'Wrote {len(result)} bytes to {args.output}')
