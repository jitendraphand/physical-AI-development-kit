"""Blender (4.x) headless turntable of the assembled ShiftGate chassis + wheels.

    /path/to/blender --background --python turntable.py -- \
        --stl-dir ../stl --out ../../media/shiftgate_turntable.mp4

This is a *render*, not a photograph of a physical robot.  The output
file should be labelled as such wherever it is shown.
"""
from __future__ import annotations

import math
import sys
from pathlib import Path

import bpy

# Args after "--"
argv = sys.argv[sys.argv.index("--") + 1 :] if "--" in sys.argv else []

def _arg(flag, default):
    if flag in argv:
        i = argv.index(flag)
        return argv[i + 1]
    return default

HERE = Path(__file__).resolve().parent
stl_dir = Path(_arg("--stl-dir", str(HERE.parent / "stl")))
out = Path(_arg("--out", str(HERE.parents[1] / "media" / "shiftgate_turntable.mp4")))
frames = int(_arg("--frames", "72"))
res = int(_arg("--res", "720"))

# Reset
bpy.ops.wm.read_factory_settings(use_empty=True)
scene = bpy.context.scene
# EEVEE Next is the realtime engine in Blender 4.2+; fall back to Cycles.
for eng in ("BLENDER_EEVEE_NEXT", "BLENDER_EEVEE", "CYCLES"):
    try:
        scene.render.engine = eng
        break
    except Exception:
        continue
if scene.render.engine == "CYCLES":
    scene.cycles.device = "CPU"
    scene.cycles.samples = 16
scene.render.resolution_x = res
scene.render.resolution_y = res
scene.render.fps = 24
scene.frame_start = 1
scene.frame_end = frames
scene.render.film_transparent = False

# World
world = bpy.data.worlds.new("World")
scene.world = world
world.use_nodes = True
bg = world.node_tree.nodes["Background"]
bg.inputs[0].default_value = (0.06, 0.07, 0.08, 1)
bg.inputs[1].default_value = 1.0

def new_mat(name, color, roughness=0.45):
    m = bpy.data.materials.new(name)
    m.use_nodes = True
    bsdf = m.node_tree.nodes["Principled BSDF"]
    bsdf.inputs["Base Color"].default_value = (*color, 1)
    bsdf.inputs["Roughness"].default_value = roughness
    return m

mat_ch = new_mat("chassis", (0.12, 0.35, 0.48), 0.4)
mat_wh = new_mat("wheel", (0.05, 0.05, 0.05), 0.55)
mat_ac = new_mat("accent", (0.16, 0.55, 0.62), 0.35)
mat_or = new_mat("gate", (0.95, 0.45, 0.12), 0.4)

def import_stl(path, mat, loc=(0, 0, 0), rot=(0, 0, 0), scale=0.001):
    # Blender 4.x
    if hasattr(bpy.ops.wm, "stl_import"):
        bpy.ops.wm.stl_import(filepath=str(path))
    else:
        bpy.ops.import_mesh.stl(filepath=str(path))
    obj = bpy.context.selected_objects[0]
    obj.location = loc
    obj.rotation_euler = rot
    obj.scale = (scale, scale, scale)
    if obj.data.materials:
        obj.data.materials[0] = mat
    else:
        obj.data.materials.append(mat)
    bpy.ops.object.transform_apply(location=False, rotation=False, scale=True)
    return obj

# Assemble in metres.  Chassis was modelled with floor at z=0, centred in XY.
# Wheels: CadQuery wheel is along Z, so rotate onto the Y axle.
chassis = import_stl(stl_dir / "chassis_base.stl", mat_ch, loc=(0, 0, 0.010))
# Axle at x = -118/2 + 28 = -31 mm, z = 24 mm, y = ±51 mm
wheel_l = import_stl(
    stl_dir / "wheel.stl",
    mat_wh,
    loc=(-0.031, 0.051, 0.024),
    rot=(math.pi / 2, 0, 0),
)
wheel_r = import_stl(
    stl_dir / "wheel.stl",
    mat_wh,
    loc=(-0.031, -0.051, 0.024),
    rot=(math.pi / 2, 0, 0),
)
cover = import_stl(stl_dir / "electronics_cover.stl", mat_ac, loc=(0.025, 0, 0.025))
us = import_stl(stl_dir / "sensor_bracket_us.stl", mat_ac, loc=(0.055, 0, 0.028))

# Ground plane
bpy.ops.mesh.primitive_plane_add(size=0.6, location=(0, 0, 0))
ground = bpy.context.active_object
ground.data.materials.append(new_mat("floor", (0.85, 0.83, 0.78), 0.7))

# Camera on a turntable empty
bpy.ops.object.empty_add(type="PLAIN_AXES", location=(0, 0, 0.03))
pivot = bpy.context.active_object
pivot.name = "turntable"
bpy.ops.object.camera_add(location=(0.22, -0.28, 0.16))
cam = bpy.context.active_object
cam.data.lens = 50
scene.camera = cam
cam.parent = pivot
# Aim at the chassis
track = cam.constraints.new(type="TRACK_TO")
track.target = chassis
track.track_axis = "TRACK_NEGATIVE_Z"
track.up_axis = "UP_Y"

# Keyframe the pivot
pivot.rotation_euler = (0, 0, 0)
pivot.keyframe_insert("rotation_euler", frame=1)
pivot.rotation_euler = (0, 0, 2 * math.pi)
pivot.keyframe_insert("rotation_euler", frame=frames)
if pivot.animation_data and pivot.animation_data.action:
    for fc in pivot.animation_data.action.fcurves:
        for kp in fc.keyframe_points:
            kp.interpolation = "LINEAR"

# Sun
bpy.ops.object.light_add(type="AREA", location=(0.3, -0.4, 0.5))
light = bpy.context.active_object
light.data.energy = 80
light.data.size = 0.4
bpy.ops.object.light_add(type="AREA", location=(-0.3, 0.3, 0.4))
bpy.context.active_object.data.energy = 30

out.parent.mkdir(parents=True, exist_ok=True)
# Render to a temp frame sequence then we can use ffmpeg, but Blender can
# write FFmpeg directly.
scene.render.image_settings.file_format = "FFMPEG"
scene.render.ffmpeg.format = "MPEG4"
scene.render.ffmpeg.codec = "H264"
scene.render.ffmpeg.constant_rate_factor = "MEDIUM"
scene.render.filepath = str(out)
print("rendering turntable to", out)
bpy.ops.render.render(animation=True)
print("done", out)
