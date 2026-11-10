#!/usr/bin/env python3
"""Generates the cell's meshes and its static scene from one layout.

    python3 tools/generate_scene.py

writes

    shared_folder/meshes/*.stl      binary STL, in millimetres (mesh_scale 0.001)
    shared_folder/scenario/*.json   one frame per file, the layout
                                    TransformsManager::load_transforms_from_path reads

Everything here is in metres, in the `world` frame. The UR5 stands at the
origin: simple_robot_simulator publishes `base_link` under `world`, which is the
root of the tree and deliberately has no file of its own.

The dice are not in here. The cell simulator (`dice_cell`'s `cell` binary)
creates them at startup, because how many there are is a setting (N_DICE). The
geometry it relies on - the die size, the arena's inner half-size and the frame
names - is mirrored in dice_cell/src/layout.rs. Change one, change the other.
"""

import json
import math
import os
import struct

import numpy as np

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
MESH_DIR = os.path.join(ROOT, "shared_folder", "meshes")
SCENE_DIR = os.path.join(ROOT, "shared_folder", "scenario")

# --- Layout (mirrored in dice_cell/src/layout.rs) --------------------------------

DIE = 0.04  # edge length
MAX_DICE = 6
MAX_PADS = 3

FEEDER_Y = -0.38
FEEDER_X0 = 0.20
FEEDER_PITCH = 0.06
RAIL_TOP = 0.01

ARENA_XY = (0.42, 0.0)
ARENA_INNER_HALF = 0.13  # inner half-size of the tray
ARENA_FLOOR = 0.005
ARENA_WALL_H = 0.05
ARENA_WALL_T = 0.008
ARENA_DROP_Z = 0.14  # where a throw lets go, die centre above the floor frame

TOWER_XY = (0.22, 0.32)
PLATE_T = 0.01

PAD_X0 = 0.38
PAD_PITCH = 0.09
PAD_Y = 0.32

TCP_Z = 0.125  # die centre between the fingers, along tool0's z
# The approach TCP sits this far *beyond* the real one, along the tool: put it on
# a target and the fingers stop this far short of it. (It used to sit this far
# back towards the flange, which put the fingers 10 cm *through* the target.)
APPROACH = 0.10

# Every transfer travels with the TCP at this height, above anything in the cell:
# a full six-die tower is 0.25 high, and a held die hangs 0.02 below the TCP.
SAFE_Z = 0.34

# The flip station: a post the robot sets a die on with the tool tipped over, so
# the die lands on its side. Tall enough that the arm, reaching down to it with
# the tool horizontal, stays clear of the table. Its frame's +y points away from
# the robot, which is the way the tool points while it places.
STATION_XY = (0.42, -0.24)
STATION_H = 0.18
STATION_HALF = 0.025


# --- STL ------------------------------------------------------------------------


def write_stl(name, triangles):
    """Binary STL. `triangles` is an (n, 3, 3) array in metres; written in mm."""
    tris = np.asarray(triangles, dtype=np.float64) * 1000.0
    path = os.path.join(MESH_DIR, name)
    with open(path, "wb") as f:
        f.write(b"dice_cell generate_scene.py".ljust(80, b" "))
        f.write(struct.pack("<I", len(tris)))
        for a, b, c in tris:
            n = np.cross(b - a, c - a)
            norm = np.linalg.norm(n)
            n = n / norm if norm > 0 else n
            f.write(struct.pack("<3f", *n))
            for v in (a, b, c):
                f.write(struct.pack("<3f", *v))
            f.write(b"\0\0")
    print(f"  {os.path.relpath(path, ROOT)}  ({len(tris)} triangles)")


def box(lo, hi):
    """Twelve outward-facing triangles of an axis-aligned box."""
    x0, y0, z0 = lo
    x1, y1, z1 = hi
    v = np.array(
        [
            [x0, y0, z0], [x1, y0, z0], [x1, y1, z0], [x0, y1, z0],
            [x0, y0, z1], [x1, y0, z1], [x1, y1, z1], [x0, y1, z1],
        ]
    )
    faces = [
        (0, 2, 1), (0, 3, 2),  # bottom
        (4, 5, 6), (4, 6, 7),  # top
        (0, 1, 5), (0, 5, 4),  # -y
        (2, 3, 7), (2, 7, 6),  # +y
        (1, 2, 6), (1, 6, 5),  # +x
        (3, 0, 4), (3, 4, 7),  # -x
    ]
    return [v[list(f)] for f in faces]


def disc(center, normal, radius, height, segments=20):
    """A closed cylinder of `height` standing on `center`, along `normal`."""
    center = np.asarray(center, dtype=float)
    normal = np.asarray(normal, dtype=float)
    normal /= np.linalg.norm(normal)
    helper = np.array([1.0, 0, 0]) if abs(normal[0]) < 0.9 else np.array([0, 1.0, 0])
    u = np.cross(normal, helper)
    u /= np.linalg.norm(u)
    w = np.cross(normal, u)
    top = center + normal * height
    ring = [
        (math.cos(2 * math.pi * i / segments), math.sin(2 * math.pi * i / segments))
        for i in range(segments)
    ]
    bottom_ring = [center + radius * (c * u + s * w) for c, s in ring]
    top_ring = [top + radius * (c * u + s * w) for c, s in ring]
    tris = []
    for i in range(segments):
        j = (i + 1) % segments
        tris.append([top, top_ring[i], top_ring[j]])
        tris.append([center, bottom_ring[j], bottom_ring[i]])
        tris.append([bottom_ring[i], bottom_ring[j], top_ring[j]])
        tris.append([bottom_ring[i], top_ring[j], top_ring[i]])
    return tris


def rounded_cube(half, exponent=0.12, n=24):
    """A superquadric: a cube with softened edges, flat where the pips go."""

    def f(angle, e):
        c = math.cos(angle)
        return math.copysign(abs(c) ** e, c)

    def g(angle, e):
        s = math.sin(angle)
        return math.copysign(abs(s) ** e, s)

    verts = []
    for i in range(n + 1):
        eta = -math.pi / 2 + math.pi * i / n
        row = []
        for j in range(2 * n):
            omega = -math.pi + 2 * math.pi * j / (2 * n)
            row.append(
                [
                    half * f(eta, exponent) * f(omega, exponent),
                    half * f(eta, exponent) * g(omega, exponent),
                    half * g(eta, exponent),
                ]
            )
        verts.append(row)
    verts = np.array(verts)
    tris = []
    for i in range(n):
        for j in range(2 * n):
            k = (j + 1) % (2 * n)
            a, b, c, d = verts[i][j], verts[i][k], verts[i + 1][k], verts[i + 1][j]
            tris.append([a, b, c])
            tris.append([a, c, d])
    return tris


# Which way each face points in the body frame. Opposite faces add up to seven;
# dice_cell/src/layout.rs turns "face f is up" into a rotation from this table.
FACE_NORMALS = {
    1: (0, 0, 1),
    6: (0, 0, -1),
    2: (1, 0, 0),
    5: (-1, 0, 0),
    3: (0, 1, 0),
    4: (0, -1, 0),
}

PIP_LAYOUT = {
    1: [(0, 0)],
    2: [(-1, -1), (1, 1)],
    3: [(-1, -1), (0, 0), (1, 1)],
    4: [(-1, -1), (-1, 1), (1, -1), (1, 1)],
    5: [(-1, -1), (-1, 1), (0, 0), (1, -1), (1, 1)],
    6: [(-1, -1), (-1, 0), (-1, 1), (1, -1), (1, 0), (1, 1)],
}


def pips(half):
    spacing = half * 0.5
    radius = half * 0.17
    tris = []
    for face, normal in FACE_NORMALS.items():
        normal = np.array(normal, dtype=float)
        helper = np.array([0, 0, 1.0]) if abs(normal[2]) < 0.9 else np.array([1.0, 0, 0])
        u = np.cross(helper, normal)
        u /= np.linalg.norm(u)
        w = np.cross(normal, u)
        for a, b in PIP_LAYOUT[face]:
            center = normal * (half - 0.0004) + spacing * (a * u + b * w)
            tris += disc(center, normal, radius, 0.001)
    return tris


# --- Meshes ---------------------------------------------------------------------


def meshes():
    print("Meshes:")
    half = DIE / 2
    write_stl("die_body.stl", rounded_cube(half))
    write_stl("die_pips.stl", pips(half))

    write_stl("table.stl", box((-0.55, -0.6, -0.02), (0.75, 0.6, 0.0)))

    # Centred on the `feeder` frame, which sits in the middle of the six slots;
    # a divider between every two of them.
    rail_half = MAX_DICE * FEEDER_PITCH / 2
    rail = box((-rail_half, -0.035, 0.0), (rail_half, 0.035, RAIL_TOP))
    for i in range(MAX_DICE + 1):
        x = -rail_half + i * FEEDER_PITCH
        rail += box((x - 0.002, -0.035, RAIL_TOP), (x + 0.002, 0.035, RAIL_TOP + 0.008))
    write_stl("feeder.stl", rail)

    h = ARENA_INNER_HALF
    t = ARENA_WALL_T
    arena = box((-h - t, -h - t, 0.0), (h + t, h + t, ARENA_FLOOR))
    arena += box((-h - t, -h - t, ARENA_FLOOR), (h + t, -h, ARENA_WALL_H))
    arena += box((-h - t, h, ARENA_FLOOR), (h + t, h + t, ARENA_WALL_H))
    arena += box((-h - t, -h, ARENA_FLOOR), (-h, h, ARENA_WALL_H))
    arena += box((h, -h, ARENA_FLOOR), (h + t, h, ARENA_WALL_H))
    write_stl("arena.stl", arena)

    write_stl("plate.stl", box((-0.035, -0.035, 0.0), (0.035, 0.035, PLATE_T)))

    station = box((-STATION_HALF, -STATION_HALF, 0.0), (STATION_HALF, STATION_HALF, STATION_H - 0.004))
    station += box((-STATION_HALF - 0.004, -STATION_HALF - 0.004, STATION_H - 0.004), (STATION_HALF + 0.004, STATION_HALF + 0.004, STATION_H))
    station += box((-0.05, -0.05, 0.0), (0.05, 0.05, 0.008))
    write_stl("station.stl", station)

    gripper = disc((0, 0, 0), (0, 0, 1), 0.035, 0.02, 32)
    gripper += box((-0.035, -0.03, 0.02), (0.035, 0.03, 0.065))
    for sign in (-1, 1):
        x_in = sign * (half + 0.001)
        x_out = sign * (half + 0.013)
        gripper += box(
            (min(x_in, x_out), -0.012, 0.065), (max(x_in, x_out), 0.012, TCP_Z + 0.018)
        )
    write_stl("gripper.stl", gripper)


# --- Scene ----------------------------------------------------------------------

Z_DOWN = {"x": 1.0, "y": 0.0, "z": 0.0, "w": 0.0}  # 180 deg about x
IDENTITY = {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}


def quat(axis, angle):
    s = math.sin(angle / 2)
    return {"x": axis[0] * s, "y": axis[1] * s, "z": axis[2] * s, "w": math.cos(angle / 2)}


def quat_mul(a, b):
    return {
        "w": a["w"] * b["w"] - a["x"] * b["x"] - a["y"] * b["y"] - a["z"] * b["z"],
        "x": a["w"] * b["x"] + a["x"] * b["w"] + a["y"] * b["z"] - a["z"] * b["y"],
        "y": a["w"] * b["y"] - a["x"] * b["z"] + a["y"] * b["w"] + a["z"] * b["x"],
        "z": a["w"] * b["z"] + a["x"] * b["y"] - a["y"] * b["x"] + a["z"] * b["w"],
    }


# Feeder slots are turned a quarter: the fingers then close across the rail, not
# along it, and clear the dividers between the slots.
FEEDER_SLOT_ROTATION = quat((0, 0, 1), math.pi / 2)
STATION_YAW = math.atan2(STATION_XY[1], STATION_XY[0]) - math.pi / 2


def frame(name, parent, xyz, rotation=IDENTITY, mesh=None, color=None):
    """One frame file. `xyz` is relative to `parent`."""
    metadata = {"enable_transform": True, "active_transform": False}
    if mesh:
        r, g, b = color
        metadata.update(
            {
                "visualize_mesh": True,
                "mesh_type": 10,
                "mesh_file": mesh,
                "mesh_scale": 0.001,
                "mesh_r": r,
                "mesh_g": g,
                "mesh_b": b,
                "mesh_a": 1.0,
            }
        )
    doc = {
        "child_frame_id": name,
        "parent_frame_id": parent,
        "transform": {
            "translation": {"x": round(xyz[0], 6), "y": round(xyz[1], 6), "z": round(xyz[2], 6)},
            "rotation": rotation,
        },
        "metadata": metadata,
    }
    with open(os.path.join(SCENE_DIR, f"{name}.json"), "w") as f:
        json.dump(doc, f, indent=4)
        f.write("\n")
    return name


def scene():
    for old in os.listdir(SCENE_DIR):
        if old.endswith(".json"):
            os.remove(os.path.join(SCENE_DIR, old))

    names = []
    grey = (0.32, 0.33, 0.36)
    steel = (0.62, 0.64, 0.68)
    dark = (0.18, 0.19, 0.21)

    names.append(frame("table", "world", (0, 0, 0), mesh="table.stl", color=(0.22, 0.23, 0.26)))

    # The feeder: where dice arrive, in a random order and showing random faces.
    names.append(
        frame("feeder", "world", (FEEDER_X0 + 2.5 * FEEDER_PITCH, FEEDER_Y, 0.0), mesh="feeder.stl", color=steel)
    )
    for i in range(1, MAX_DICE + 1):
        x = FEEDER_X0 + (i - 1) * FEEDER_PITCH
        names.append(frame(f"feeder_slot_{i}", "world", (x, FEEDER_Y, RAIL_TOP + DIE / 2), FEEDER_SLOT_ROTATION))

    # The arena: a walled tray the robot throws into. Where a die lands and what
    # it shows is up to the cell simulator.
    names.append(frame("arena", "world", (ARENA_XY[0], ARENA_XY[1], 0.0), mesh="arena.stl", color=grey))
    names.append(frame("arena_drop", "arena", (0.0, 0.0, ARENA_DROP_Z)))
    names.append(frame("arena_drop_transit", "world", (ARENA_XY[0], ARENA_XY[1], SAFE_Z)))

    # The tower, and the positions its levels take.
    names.append(
        frame("tower_base", "world", (TOWER_XY[0], TOWER_XY[1], 0.0), mesh="plate.stl", color=dark)
    )
    for k in range(1, MAX_DICE + 1):
        names.append(frame(f"level_{k}", "tower_base", (0, 0, PLATE_T + DIE / 2 + (k - 1) * DIE)))
        names.append(frame(f"level_{k}_transit", "world", (TOWER_XY[0], TOWER_XY[1], SAFE_Z)))

    # Buffer pads: somewhere to park a die without losing the face it shows.
    for j in range(1, MAX_PADS + 1):
        x = PAD_X0 + (j - 1) * PAD_PITCH
        names.append(frame(f"pad_{j}_plate", "world", (x, PAD_Y, 0.0), mesh="plate.stl", color=steel))
        names.append(frame(f"pad_{j}", f"pad_{j}_plate", (0, 0, PLATE_T + DIE / 2)))
        names.append(frame(f"pad_{j}_transit", "world", (x, PAD_Y, SAFE_Z)))

    # The flip station. `flip_station` is where a die sits on it, upright;
    # `flip_station_tilt` is the TCP pose that sets a die down there on its side:
    # the station frame turned a quarter about x, so the tool points along the
    # station's +y (away from the robot) and whatever faced the TCP's +y comes up.
    sx, sy = STATION_XY
    station_rotation = quat((0, 0, 1), STATION_YAW)
    tilt_rotation = quat_mul(station_rotation, quat((1, 0, 0), math.pi / 2))
    names.append(frame("flip_station_post", "world", (sx, sy, 0.0), station_rotation, mesh="station.stl", color=steel))
    names.append(frame("flip_station", "world", (sx, sy, STATION_H + DIE / 2), station_rotation))
    names.append(frame("flip_station_transit", "world", (sx, sy, SAFE_Z), station_rotation))
    names.append(frame("flip_station_tilt", "world", (sx, sy, STATION_H + DIE / 2), tilt_rotation))
    names.append(frame("flip_station_tilt_above", "world", (sx, sy, STATION_H + DIE / 2 + APPROACH), tilt_rotation))

    # The gripper, on the UR5's flange. `tool0` belongs to the robot simulator;
    # these hang off it and move with it.
    names.append(frame("gripper", "tool0", (0, 0, 0), mesh="gripper.stl", color=(0.12, 0.12, 0.13)))
    names.append(frame("gripper_tcp", "tool0", (0, 0, TCP_Z), rotation=Z_DOWN))
    names.append(frame("gripper_tcp_above", "tool0", (0, 0, TCP_Z + APPROACH), rotation=Z_DOWN))

    print(f"Scene: {len(names)} frames in {os.path.relpath(SCENE_DIR, ROOT)}")


if __name__ == "__main__":
    os.makedirs(MESH_DIR, exist_ok=True)
    os.makedirs(SCENE_DIR, exist_ok=True)
    meshes()
    scene()
