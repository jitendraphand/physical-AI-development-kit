# Print settings

All parts are designed for **PLA**, a **0.4 mm nozzle**, and a bed ≥ 220 mm (Ender 3 class).  No part requires support if you use the orientation below, except the optional flag on `gate_segment`.

The authoritative table, including estimated time and filament, is [`cad/print_settings.yaml`](cad/print_settings.yaml).  Masses labelled *solid* are watertight CAD volume × 1.24 g/cm³ (an upper bound).  *Typical sliced* accounts for infill.

## Defaults (every part unless the YAML says otherwise)

| Setting | Value |
|---|---|
| Material | PLA, 1.75 mm |
| Nozzle / bed | 210 °C / 60 °C |
| Layer height | 0.20 mm (0.16 mm for wheels, caster, clamps) |
| Walls | 3 (4 on the wheels) |
| Infill | 30 % gyroid on the chassis; 15–20 % on clips, posts, cover |
| Supports | None (gate flag: touching buildplate only, if needed) |
| Adhesion | Skirt; a 3 mm brim on `sensor_bracket_ir` |

## Orientation cheat-sheet

- **chassis_base** — floor on the bed, motor pockets up.  No supports.
- **wheel** — *face* on the bed, never on the tread.  Stretch a rubber band into the groove after printing.
- **caster_cup** — retention opening on the bed.
- **wall_clip** — base down; pack a full plate (≈ 16 per 220 mm bed).
- **wall_post** — 14 × 14 face down, slots vertical.
- **gate_segment** — large wall face down, handle pointing up, **contrasting colour**.

## How much filament and time?

Default kit (24 clips, 12 posts, one gate, no spare `wall_segment`):

- Typical sliced mass ≈ **250 g** (solid upper bound 327 g).
- Typical time ≈ **14–18 hours** across several plates.
- Filament cost ≈ **₹175–250** of a 1 kg Indian PLA roll.

Regenerate the mass table any time:

```bash
python cad/verify_stls.py
```
