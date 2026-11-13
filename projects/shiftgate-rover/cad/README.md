# CAD

Two generators produce the same part names:

| Script | Run with | What it did in this repo |
|---|---|---|
| [`cadquery/generate_parts.py`](cadquery/generate_parts.py) | CPython + CadQuery 2.x | **This is what exported the shipped STL and STEP files.**  Verified watertight with [`verify_stls.py`](verify_stls.py). |
| [`freecad/generate_shiftgate_parts.py`](freecad/generate_shiftgate_parts.py) | `freecadcmd` | Provided so a lab that already runs FreeCAD can tweak parts in the GUI.  FreeCAD was **not** available on the authoring machine. |

```bash
python cadquery/generate_parts.py --outdir ../stl --stepdir ../step
python verify_stls.py
```

Print settings: [`print_settings.yaml`](print_settings.yaml) and [../PRINT_SETTINGS.md](../PRINT_SETTINGS.md).

Dimensions live next to the algorithm, in `software/shiftgate_core/shiftgate_core/params.py`.  Keep the three files in sync.
