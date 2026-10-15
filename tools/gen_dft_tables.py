#!/usr/bin/env python3
"""
Extract the DFT block's test patterns from the stock firmware.

They are vectors hard-wired by the chip maker: they have no structure we can
interpret, and copying them by hand would be a source of silent errors. They
are read straight from the ELF's OBJECT symbols, which makes the extraction
reproducible and checkable.

The width of each table CANNOT be deduced from its size: it comes from the way
the firmware uses it. A mistake here is silent at compile time and fatal at run
time (dft_id read as 32 bits gives 255 entries instead of 510, and the sweep
stops halfway). Hence the WIDTH tables below, justified call site by call site.

    python3 tools/gen_dft_tables.py <stock jasminer.elf> [--item4]

tools/prepare_build.sh runs it on the jasminer.elf of Sunlune's official
archive (JasMiner-X16-Q-20241014-141748).
"""

import re
import struct
import subprocess
import sys

# Width of each table, deduced from its use in dft_pattern.c:
#
#   16 bits: dft_id and its remaps     indexed by *(uint16_t *)(&t + i*2)
#            the lr_trim               read as lr_trim[0..3]
#            the MRR cmd               sl_multi_write(..., 0xd20, p, 8)  -> 8 words
#            the DP / HB info cmd      sl_multi_write(..., 0xd08, p, 2)  -> 2 words
#
#   32 bits: the command blocks        11 32-bit words per block (stride 44)
#            the delays                one uint32 per block
WIDTH16 = [
    "dft_id", "dft_id_2_remap_id_even", "dft_id_2_remap_id_odd",
    "dft_normal_lr_trim", "dft_mrr_lr_trim",
    "dft_read_DP_info_cmd", "dft_end_DP_info_cmd",
    "dft_Unit0_read_HB_info_cmd", "dft_Unit1_read_HB_info_cmd",
    "dft_Unit0_MRR_read_pre_cmd", "dft_Unit0_MRR_read_post_even_cmd",
    "dft_Unit0_MRR_read_post_odd_cmd",
    "dft_Unit1_MRR_read_pre_cmd", "dft_Unit1_MRR_read_post_even_cmd",
    "dft_Unit1_MRR_read_post_odd_cmd",
]
WIDTH32 = [
    "dft_DataPathtest_cmd", "dft_Unit0_Hbtest_cmd", "dft_Unit1_Hbtest_cmd",
    "dft_Hbtest_timeout",
    "dft_Unit0_initial_pre_cmd", "dft_Unit1_initial_pre_cmd",
    "dft_Unit0_initial_post_cmd", "dft_Unit1_initial_post_cmd",
    "dft_initial_pre_timeout", "dft_initial_post_timeout",
    "dft_Unit0_Item3_pre_cmd", "dft_Unit1_Item3_pre_cmd",
    "dft_Unit0_Item3_post_even_cmd", "dft_Unit0_Item3_post_odd_cmd",
    "dft_Unit1_Item3_post_even_cmd", "dft_Unit1_Item3_post_odd_cmd",
    "dft_Item3_pre_timeout", "dft_Item3_post_timeout",
]

# ---------------------------------------------------------------------------
# ITEM4: the MARGIN test left by Sunlune, never called by anyone.
#
# Same shape as Item3 (blocks of 11 32-bit words, one delay per block) with TWO
# additions: a sweep direction (up / down) and, above all, ONE TRIM PER BLOCK.
# 447 blocks against 138 for Item3, and 12 distinct trim combinations: a 12-point
# shmoo, repeated in both directions.
#
# The nominal trim (dft_normal_lr_trim = 0000,0000,0300,4000) is where the sweep
# starts and the value it returns to at the last block. up and down are the same
# sweep shifted by a nibble: these registers pack 4-bit fields and the two
# directions aim at two different fields.
#
# Extraction behind --item4: these tables weigh 286 KB and only matter for a
# characterization campaign.
# ---------------------------------------------------------------------------
ITEM4_16 = [
    f"dft_Unit{u}_Item4_{ph}_{d}_lr_trim"
    for u in (0, 1) for ph in ("pre", "post_even", "post_odd") for d in ("up", "down")
]
ITEM4_32 = [
    f"dft_Unit{u}_Item4_{ph}_{d}_cmd"
    for u in (0, 1) for ph in ("pre", "post_even", "post_odd") for d in ("up", "down")
] + [
    f"dft_Item4_{ph}_{d}_timeout"
    for ph in ("pre", "post") for d in ("up", "down")
]


def load_segments(elf):
    out = subprocess.run(["readelf", "-lW", elf], capture_output=True, text=True).stdout
    segs = []
    for m in re.finditer(
        r"LOAD\s+0x([0-9a-f]+)\s+0x([0-9a-f]+)\s+0x[0-9a-f]+\s+0x([0-9a-f]+)", out
    ):
        off, va, fsz = (int(x, 16) for x in m.groups())
        segs.append((va, off, fsz))
    return segs


def main(elf, item4=False):
    blob = open(elf, "rb").read()
    segs = load_segments(elf)

    def read(va, n):
        for v, o, f in segs:
            if v <= va < v + f:
                return blob[o + (va - v) : o + (va - v) + n]
        raise KeyError(f"0x{va:x} outside the segments")

    syms = {}
    out = subprocess.run(["readelf", "-sW", elf], capture_output=True, text=True).stdout
    for line in out.splitlines():
        p = line.split()
        if len(p) >= 8 and p[3] == "OBJECT" and p[7].startswith("dft_"):
            syms.setdefault(p[7], (int(p[1], 16), int(p[2])))

    src = [
        "/*",
        " * DFT block test patterns — GENERATED TABLE, DO NOT EDIT BY HAND.",
        " *",
        " * Produced by tools/gen_dft_tables.py from the stock firmware.",
        " * Rerun that tool rather than fixing this file.",
        " */",
        "",
        '#include "dft_tables.h"',
        "",
    ]
    hdr = ["#ifndef DFT_TABLES_H", "#define DFT_TABLES_H", "#include <stdint.h>", ""]
    total = 0

    w16 = WIDTH16 + (ITEM4_16 if item4 else [])
    w32 = WIDTH32 + (ITEM4_32 if item4 else [])
    for name in w16 + w32:
        if name not in syms:
            print(f"  MISSING {name}", file=sys.stderr)
            continue
        va, size = syms[name]
        data = read(va, size)
        total += size
        if name in w16:
            vals = struct.unpack(f"<{size // 2}H", data)
            typ, per, fmt = "uint16_t", 8, "0x%04x"
        else:
            vals = struct.unpack(f"<{size // 4}I", data)
            typ, per, fmt = "uint32_t", 6, "0x%08x"
        hdr.append(f"extern const {typ} {name}[{len(vals)}];")
        src.append(f"const {typ} {name}[{len(vals)}] = {{")
        for i in range(0, len(vals), per):
            src.append("    " + ", ".join(fmt % v for v in vals[i : i + per]) + ",")
        src.append("};\n")
        print(f"  {name:36s} {len(vals):6d} x {typ}")

    hdr += ["", "#endif"]
    open("core/dft_tables.c", "w").write("\n".join(src) + "\n")
    open("include/dft_tables.h", "w").write("\n".join(hdr) + "\n")
    print(f"\n  {len(w16) + len(w32)} tables, {total} bytes")


if __name__ == "__main__":
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    main(args[0], item4="--item4" in sys.argv)
