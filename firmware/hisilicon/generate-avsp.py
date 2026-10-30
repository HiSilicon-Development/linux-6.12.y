#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-only
"""Generate the CV200 AVSP image from the SPC060 assembly object."""
import argparse
import re
from pathlib import Path

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("source", type=Path, help="SPC060 HiSCDV200/scd_drv.S")
parser.add_argument("output", type=Path, help="generated histb-avsp.bin")
args = parser.parse_args()
source = args.source.read_text()
match = re.search(
r"^g_avsp_firmware:\s*\n((?:[ \t]*\.word[ \t]+-?\d+[ \t]*\n){4480})",
    source, re.MULTILINE)
if not match:
    parser.error("expected the 4480-word CV200 g_avsp_firmware object")
values = [int(value) for value in re.findall(r"\.word\s+(-?\d+)", match.group(1))]
result = b"".join((value & 0xffffffff).to_bytes(4, "little") for value in values)
args.output.parent.mkdir(parents=True, exist_ok=True)
args.output.write_bytes(result)
print(f"Wrote {len(result)} bytes to {args.output}")
