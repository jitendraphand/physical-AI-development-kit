# Media

Every file in this folder is a **simulation or CAD render**, not a photograph or a camera recording of a physical robot.

| File | What it is |
|---|---|
| `shiftgate_sim.mp4` | PyBullet TinyRenderer playback of the curriculum mission (explore, silent gate shift, discrepancy, replan, home).  1280×720, 20 fps, ≈ 25 s. |
| `shiftgate_sim_poster.png` | Still from that render, at the `GATE SHIFT DETECTED` moment. |
| `shiftgate_arena_overview.png` | Still from the explore phase, gate still open. |
| `shiftgate_cad_render.mp4` | Blender 4.5 EEVEE CAD render of the assembled STL parts in the stock 5×5 maze.  Camera starts on the rover at `(0,0)` and pulls back to show the orange gate and the package cell.  1280×720, 16 fps, 8 s.  The HUD line “CAD render (not real footage)” is burned into every frame. |
| `shiftgate_cad_render_poster.png` | Still from the end of that CAD render (full maze in frame). |

The HUD line “simulation render (not real footage)” is burned into every frame of the PyBullet video on purpose.

To regenerate the CAD render (Blender 4.2+ with EEVEE Next):

```bash
blender --background --python cad/renders/cad_render.py -- \
    --stl-dir cad/stl --out media/shiftgate_cad_render.mp4
# then mux the PNG sequence in /tmp/shiftgate_cad_frames with ffmpeg and the caption above
```
