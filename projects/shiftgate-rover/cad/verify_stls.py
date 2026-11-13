#!/usr/bin/env python3
"""Check every exported STL is a watertight volume and write a mass report.

Usage (from this folder, with trimesh installed):

    python verify_stls.py
"""
from __future__ import annotations

import json
import sys
from pathlib import Path

import trimesh

HERE = Path(__file__).resolve().parent
STL = HERE / "stl"
PLA_DENSITY_G_CM3 = 1.24

QTY = {
    "chassis_base": 1,
    "motor_clamp": 2,
    "wheel": 2,
    "caster_cup": 1,
    "sensor_bracket_us": 1,
    "sensor_bracket_ir": 2,
    "electronics_cover": 1,
    "battery_retainer": 1,
    "wall_clip": 24,
    "wall_post": 12,
    "wall_segment": 0,
    "gate_segment": 1,
    "alignment_jig": 1,
}


def load_mesh(path: Path) -> trimesh.Trimesh:
    m = trimesh.load(str(path), force="mesh")
    if isinstance(m, trimesh.Scene):
        geoms = list(m.geometry.values())
        m = trimesh.util.concatenate(geoms)
    return m


def main() -> int:
    paths = sorted(STL.glob("*.stl"))
    if not paths:
        print(f"no STLs in {STL}", file=sys.stderr)
        return 2
    rows = []
    bad = []
    print(f"{'part':22s} {'wt':>3} {'cm^3':>8} {'g PLA':>7} {'qty':>4} {'g*qty':>7} extents (mm)")
    total = 0.0
    for p in paths:
        m = load_mesh(p)
        wt = bool(m.is_watertight)
        if not wt:
            bad.append(p.name)
        vol_cm3 = float(abs(m.volume)) / 1000.0
        grams = vol_cm3 * PLA_DENSITY_G_CM3
        qty = QTY.get(p.stem, 1)
        total += grams * qty
        ext = [round(float(x), 1) for x in m.extents]
        print(f"{p.stem:22s} {'yes' if wt else 'NO':>3} {vol_cm3:8.2f} {grams:7.2f} {qty:4d} {grams*qty:7.1f} {ext}")
        rows.append(
            {
                "name": p.stem,
                "watertight": wt,
                "volume_cm3": round(vol_cm3, 3),
                "grams_pla_solid": round(grams, 2),
                "qty": qty,
                "extents_mm": ext,
                "faces": int(len(m.faces)),
            }
        )
    print(f"\nSolid-volume upper bound for the default kit (wall_segment qty=0): {total:.0f} g")
    print("Sliced mass is typically 70–90% of that, depending on infill.")
    (STL / "verification.json").write_text(json.dumps(rows, indent=2))
    if bad:
        print("NOT watertight:", ", ".join(bad), file=sys.stderr)
        return 1
    print("all watertight")
    return 0


if __name__ == "__main__":
    sys.exit(main())
