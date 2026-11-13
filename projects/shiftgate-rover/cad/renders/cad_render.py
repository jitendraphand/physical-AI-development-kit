"""Blender 4.5 headless CAD render of the assembled ShiftGate Rover in its maze.

    blender --background --python cad_render.py -- \
        --stl-dir ../stl \
        --out ../../media/shiftgate_cad_render.mp4

This is a *CAD render*, not a photograph or a camera recording of a
physical robot.  Caption text is burned in after the render with ffmpeg.
"""
from __future__ import annotations

import math
import shutil
import sys
from pathlib import Path

import bpy

argv = sys.argv[sys.argv.index("--") + 1 :] if "--" in sys.argv else []


def arg(flag, default):
    if flag in argv:
        return argv[argv.index(flag) + 1]
    return default


HERE = Path(__file__).resolve().parent
STL = Path(arg("--stl-dir", str(HERE.parent / "stl")))
OUT = Path(arg("--out", str(HERE.parents[1] / "media" / "shiftgate_cad_render.mp4")))
FRAMES = int(arg("--frames", "128"))  # 8 s at 16 fps
FPS = int(arg("--fps", "16"))
RESX = int(arg("--resx", "1280"))
RESY = int(arg("--resy", "720"))
SAMPLES = int(arg("--samples", "8"))
PREVIEW = "--preview" in argv

CELL, T, H, N = 0.180, 0.004, 0.050, 5
SCALE = 0.001  # STL units are millimetres


def hline_y(i: int) -> float:
    return T / 2 + i * (CELL + T)


def vline_x(i: int) -> float:
    return T / 2 + i * (CELL + T)


def cell_center(x: int, y: int) -> tuple[float, float]:
    return T + CELL * (x + 0.5), T + CELL * (y + 0.5)


def arena_outer() -> float:
    return N * CELL + (N + 1) * T


def zigzag_blocked():
    """Internal E–W walls of the stock maze, minus the centre shaft and the gate."""
    blocked = set()
    for y in range(N - 1):
        open_x = (N - 1) if (y % 2 == 0) else 0
        for x in range(N):
            if x != open_x:
                blocked.add((x, y))
    shaft_x, gate_door = 2, (2, 2)
    return {(x, y) for (x, y) in blocked if not (x == shaft_x and (x, y) != gate_door)}


def new_mat(name, color, roughness=0.45, metallic=0.0):
    m = bpy.data.materials.new(name)
    m.use_nodes = True
    bsdf = m.node_tree.nodes["Principled BSDF"]
    bsdf.inputs["Base Color"].default_value = (*color, 1.0)
    bsdf.inputs["Roughness"].default_value = roughness
    if "Metallic" in bsdf.inputs:
        bsdf.inputs["Metallic"].default_value = metallic
    return m


def assign(obj, mat):
    if obj.data.materials:
        obj.data.materials[0] = mat
    else:
        obj.data.materials.append(mat)


def box(name, size, loc, mat, rot=(0, 0, 0)):
    bpy.ops.mesh.primitive_cube_add(size=1, location=loc, rotation=rot)
    obj = bpy.context.active_object
    obj.name = name
    obj.scale = size
    bpy.ops.object.transform_apply(location=False, rotation=False, scale=True)
    assign(obj, mat)
    return obj


def import_stl(path, mat, loc=(0, 0, 0), rot=(0, 0, 0), scale=SCALE):
    if hasattr(bpy.ops.wm, "stl_import"):
        bpy.ops.wm.stl_import(filepath=str(path))
    else:
        bpy.ops.import_mesh.stl(filepath=str(path))
    obj = bpy.context.selected_objects[0]
    bpy.context.view_layer.objects.active = obj
    obj.scale = (scale, scale, scale)
    bpy.ops.object.transform_apply(location=False, rotation=False, scale=True)
    obj.location = loc
    obj.rotation_euler = rot
    assign(obj, mat)
    return obj


def build_scene():
    bpy.ops.wm.read_factory_settings(use_empty=True)
    scene = bpy.context.scene

    mat_ch = new_mat("chassis", (0.04, 0.16, 0.24), 0.42)
    mat_wh = new_mat("wheel", (0.05, 0.05, 0.05), 0.70)
    mat_ac = new_mat("accent", (0.06, 0.32, 0.36), 0.40)
    mat_or = new_mat("gate", (0.85, 0.30, 0.05), 0.42)
    mat_wall = new_mat("wall", (0.24, 0.26, 0.29), 0.65)
    mat_floor = new_mat("floor", (0.36, 0.34, 0.30), 0.75)
    mat_start = new_mat("start", (0.22, 0.46, 0.28), 0.62)
    mat_pkg = new_mat("package", (0.78, 0.58, 0.10), 0.50)
    mat_tile_a = new_mat("tile_a", (0.58, 0.56, 0.50), 0.70)
    mat_tile_b = new_mat("tile_b", (0.52, 0.50, 0.45), 0.70)

    outer = arena_outer()
    box("studio_floor", (outer + 0.40, outer + 0.40, 0.012), (outer / 2, outer / 2, -0.006), mat_floor)

    for x in range(N):
        for y in range(N):
            cx, cy = cell_center(x, y)
            if (x, y) == (0, 0):
                mat = mat_start
            elif (x, y) == (4, 4):
                mat = mat_pkg
            else:
                mat = mat_tile_a if (x + y) % 2 == 0 else mat_tile_b
            half = CELL / 2 - 0.003
            box(f"tile_{x}_{y}", (2 * half, 2 * half, 0.002), (cx, cy, 0.001), mat)

    def wall(name, cx, cy, sx, sy, mat=mat_wall, z=None):
        if z is None:
            z = H / 2
        return box(name, (sx, sy, H), (cx, cy, z), mat)

    for x in range(N):
        cx = T + CELL * (x + 0.5)
        wall(f"south_{x}", cx, hline_y(0), CELL + T, T)
        wall(f"north_{x}", cx, hline_y(N), CELL + T, T)
    for y in range(N):
        cy = T + CELL * (y + 0.5)
        wall(f"west_{y}", vline_x(0), cy, T, CELL + T)
        wall(f"east_{y}", vline_x(N), cy, T, CELL + T)
    for x, y in sorted(zigzag_blocked()):
        wall(f"h_{x}_{y}", T + CELL * (x + 0.5), hline_y(y + 1), CELL + T, T)

    # Printed orange gate, shown closed, in the centre shaft (between (2,2) and (2,3)).
    gate_path = STL / "gate_segment.stl"
    gx, gy = T + CELL * (2 + 0.5), hline_y(3)
    if gate_path.exists():
        import_stl(gate_path, mat_or, loc=(gx, gy, 0.0))
    else:
        wall("gate", gx, gy, CELL + T, 0.006, mat=mat_or)

    # Yellow "package" token on the far cell so it reads in the overview.
    px, py = cell_center(4, 4)
    box("package", (0.040, 0.040, 0.030), (px, py, 0.016), mat_pkg)

    # --- robot at start cell, heading east (+X) ---
    # Chassis CAD: floor underside at z=0, +X forward, motors at the rear.
    rx, ry = cell_center(0, 0)
    z0 = 0.010
    import_stl(STL / "chassis_base.stl", mat_ch, loc=(rx, ry, z0))
    axle_x, axle_z = rx - 0.031, 0.024
    half = 0.051
    import_stl(STL / "wheel.stl", mat_wh, loc=(axle_x, ry + half, axle_z), rot=(math.pi / 2, 0, 0))
    import_stl(STL / "wheel.stl", mat_wh, loc=(axle_x, ry - half, axle_z), rot=(math.pi / 2, 0, 0))
    if (STL / "electronics_cover.stl").exists():
        # Cover holes match ESP standoffs at x = +29 mm, sitting on the 3 mm floor.
        import_stl(STL / "electronics_cover.stl", mat_ac, loc=(rx + 0.029, ry, z0 + 0.003))
    if (STL / "sensor_bracket_us.stl").exists():
        # Plate stands at the front bumper, transducers facing +X, eyes along Y.
        # XYZ (90, 0, 90): 3 mm thick in X (facing forward), eyes along Y, ~32 mm tall.
        import_stl(
            STL / "sensor_bracket_us.stl",
            mat_ac,
            loc=(rx + 0.061, ry, z0 + 0.012),
            rot=(math.pi / 2, 0, math.pi / 2),
        )
    if (STL / "sensor_bracket_ir.stl").exists():
        import_stl(STL / "sensor_bracket_ir.stl", mat_ac, loc=(rx + 0.030, ry + 0.045, z0 + 0.003))
        import_stl(
            STL / "sensor_bracket_ir.stl",
            mat_ac,
            loc=(rx + 0.030, ry - 0.045, z0 + 0.003),
            rot=(0, 0, math.pi),
        )
    if (STL / "caster_cup.stl").exists():
        import_stl(STL / "caster_cup.stl", mat_ac, loc=(rx + 0.043, ry, 0.0))
    if (STL / "battery_retainer.stl").exists():
        import_stl(STL / "battery_retainer.stl", mat_ac, loc=(rx + 0.013, ry, z0 + 0.016))

    # World / lighting
    world = bpy.data.worlds.new("World")
    scene.world = world
    world.use_nodes = True
    bg = world.node_tree.nodes["Background"]
    bg.inputs[0].default_value = (0.30, 0.33, 0.36, 1)
    bg.inputs[1].default_value = 0.35

    bpy.ops.object.light_add(type="AREA", location=(rx + 0.25, ry - 0.55, 0.70))
    key = bpy.context.active_object
    key.data.energy = 90
    key.data.size = 0.80
    key.rotation_euler = (math.radians(55), 0, math.radians(20))

    bpy.ops.object.light_add(type="AREA", location=(outer * 0.85, outer * 0.85, 0.80))
    fill = bpy.context.active_object
    fill.data.energy = 40
    fill.data.size = 1.0
    fill.rotation_euler = (math.radians(40), 0, math.radians(-130))

    bpy.ops.object.light_add(type="AREA", location=(outer * 0.5, outer * 0.15, 1.10))
    rim = bpy.context.active_object
    rim.data.energy = 25
    rim.data.size = 1.2
    rim.rotation_euler = (0, 0, 0)

    # Camera: close hero on the rover, then pull back and up to a
    # south-east 3/4 of the whole maze (start, orange gate, package all in frame).
    bpy.ops.object.empty_add(type="PLAIN_AXES", location=(rx + 0.05, ry + 0.02, 0.035))
    aim = bpy.context.active_object
    aim.name = "cam_aim"

    bpy.ops.object.camera_add(location=(rx + 0.30, ry - 0.24, 0.15))
    cam = bpy.context.active_object
    cam.name = "Camera"
    cam.data.lens = 35
    cam.data.clip_start = 0.02
    cam.data.clip_end = 10.0
    scene.camera = cam

    track = cam.constraints.new(type="TRACK_TO")
    track.target = aim
    track.track_axis = "TRACK_NEGATIVE_Z"
    track.up_axis = "UP_Y"

    scene.frame_start = 1
    scene.frame_end = FRAMES
    scene.render.fps = FPS
    mid = max(2, FRAMES // 2)

    cam.location = (rx + 0.30, ry - 0.24, 0.15)
    cam.keyframe_insert("location", frame=1)
    cam.location = (rx + 0.55, ry - 0.42, 0.42)
    cam.keyframe_insert("location", frame=mid)
    cam.location = (outer * 0.82, -0.52, 1.08)
    cam.keyframe_insert("location", frame=FRAMES)

    cam.data.lens = 35
    cam.data.keyframe_insert("lens", frame=1)
    cam.data.lens = 32
    cam.data.keyframe_insert("lens", frame=mid)
    cam.data.lens = 28
    cam.data.keyframe_insert("lens", frame=FRAMES)

    aim.location = (rx + 0.05, ry + 0.02, 0.035)
    aim.keyframe_insert("location", frame=1)
    aim.location = (outer * 0.28, outer * 0.22, 0.04)
    aim.keyframe_insert("location", frame=mid)
    aim.location = (outer * 0.50, outer * 0.40, 0.02)
    aim.keyframe_insert("location", frame=FRAMES)

    def ease(obj, data_path="location"):
        ad = obj.animation_data
        if not ad or not ad.action:
            return
        for fc in ad.action.fcurves:
            if data_path in fc.data_path:
                for kp in fc.keyframe_points:
                    kp.interpolation = "BEZIER"
                    kp.handle_left_type = "AUTO_CLAMPED"
                    kp.handle_right_type = "AUTO_CLAMPED"

    ease(cam)
    ease(aim)
    if cam.data.animation_data:
        ease(cam.data, "lens")

    return scene


def configure_eevee(scene):
    scene.render.engine = "BLENDER_EEVEE_NEXT"
    ev = scene.eevee
    ev.taa_render_samples = SAMPLES
    if hasattr(ev, "taa_samples"):
        ev.taa_samples = SAMPLES
    if hasattr(ev, "use_shadows"):
        ev.use_shadows = True
    if hasattr(ev, "use_raytracing"):
        ev.use_raytracing = False
    scene.render.resolution_x = RESX
    scene.render.resolution_y = RESY
    scene.render.resolution_percentage = 100
    scene.render.film_transparent = False
    scene.render.fps = FPS
    scene.frame_start = 1
    scene.frame_end = FRAMES
    scene.view_settings.view_transform = "AgX"
    try:
        scene.view_settings.look = "AgX - Medium High Contrast"
    except TypeError:
        pass
    scene.view_settings.exposure = -0.15
    print("engine", scene.render.engine, "samples", SAMPLES, "frames", FRAMES, "fps", FPS)
    # Image-only denoise keeps 8-sample EEVEE close-ups from looking sandy.
    scene.use_nodes = True
    scene.render.use_compositing = True
    tree = scene.node_tree
    tree.nodes.clear()
    rl = tree.nodes.new("CompositorNodeRLayers")
    dn = tree.nodes.new("CompositorNodeDenoise")
    comp = tree.nodes.new("CompositorNodeComposite")
    tree.links.new(rl.outputs["Image"], dn.inputs["Image"])
    tree.links.new(dn.outputs["Image"], comp.inputs["Image"])


def main():
    scene = build_scene()
    configure_eevee(scene)
    OUT.parent.mkdir(parents=True, exist_ok=True)
    if PREVIEW:
        stills = [
            (1, OUT.with_name(OUT.stem + "_start.preview.png")),
            (max(2, FRAMES // 2), OUT.with_name(OUT.stem + "_mid.preview.png")),
            (FRAMES, OUT.with_name(OUT.stem + "_end.preview.png")),
        ]
        scene.render.image_settings.file_format = "PNG"
        for frame, path in stills:
            scene.frame_set(frame)
            scene.render.filepath = str(path)
            bpy.ops.render.render(write_still=True)
            print("preview", frame, path)
        return
    seq = Path("/tmp/shiftgate_cad_frames")
    if seq.exists():
        shutil.rmtree(seq)
    seq.mkdir(parents=True)
    scene.render.image_settings.file_format = "PNG"
    scene.render.filepath = str(seq / "f_")
    print("rendering", FRAMES, "frames to", seq)
    bpy.ops.render.render(animation=True)
    print("frames done")


if __name__ == "__main__":
    main()
