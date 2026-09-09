"""Generate all F1TENTH component meshes as binary STL files.

Models a realistic 1/10-scale off-road RC buggy with twin-vertical-plate
chassis, A-arm suspension, coil-over shocks, motor cans, Jetson compute
module, battery, camera, and hubs. All geometry authored in base_link
coordinates to match the fixed URDF joint offsets:

  wheelbase 0.3302 m, track 0.2032 m (0.12065 m each side),
  wheel radius 0.0508 m (center at z=0.0508).

Usage:
    python3 generate_f1tenth_meshes.py
"""

import math
import struct
from pathlib import Path

# ---------------------------------------------------------------------------
# Vehicle constants (must match URDF joint offsets exactly)
# ---------------------------------------------------------------------------
WHEELBASE = 0.3302
TRACK = 0.2032
HALF_TRACK = TRACK / 2
WHEEL_R = 0.0508
REAR_AXLE_X = -0.0508
FRONT_AXLE_X = REAR_AXLE_X + WHEELBASE
WHEEL_Z = WHEEL_R

# Chassis geometry
CHASSIS_PLATE_THICK = 0.002
CHASSIS_SEP = 0.038
CHASSIS_PLATE_W = CHASSIS_SEP + 2 * CHASSIS_PLATE_THICK
CHASSIS_TOP = 0.075
CHASSIS_BOT = 0.038
CHASSIS_PLATE_H = CHASSIS_TOP - CHASSIS_BOT
CHASSIS_REAR = REAR_AXLE_X - 0.03
CHASSIS_FRONT = FRONT_AXLE_X + 0.035
NOSE_X = CHASSIS_FRONT + 0.015
TAPER_START_X = 0.22

# Front shock tower
TOWER_BASE_X = FRONT_AXLE_X - 0.005
TOWER_TOP_Z = 0.118
TOWER_BOT_Z = CHASSIS_TOP
TOWER_THICK = 0.003

# Rear shock tower
REAR_TOWER_X = REAR_AXLE_X + 0.015
REAR_TOWER_TOP_Z = 0.110

# Front bumper
BUMPER_FRONT_X = NOSE_X + 0.025
BUMPER_BOT_Z = 0.025

# A-arm dimensions
ARM_W = 0.005
ARM_H = 0.004

# Hub dimensions
HUB_R = 0.007
HUB_W = 0.010

# Shock dimensions
SHOCK_BODY_R = 0.003
SHOCK_ROD_R = 0.0012

# Motor dimensions
MOTOR_R = 0.009
MOTOR_LEN = 0.022

# Jetson board
JETSON_W = 0.045
JETSON_D = 0.035
JETSON_PCB_H = 0.003
JETSON_HEATSINK_H = 0.008

# Battery
BATT_W = 0.035
BATT_D = 0.025
BATT_H = 0.012

# Camera
CAM_W = 0.013
CAM_D = 0.010
CAM_H = 0.009
LENS_R = 0.004

# ---------------------------------------------------------------------------
# Triangle accumulator
# ---------------------------------------------------------------------------
triangles = []


def _tri(a, b, c):
    triangles.append((a, b, c))


def _quad(a, b, c, d):
    _tri(a, b, c)
    _tri(a, c, d)


def _add_box(x0, y0, z0, x1, y1, z1):
    """Axis-aligned box from (x0,y0,z0) to (x1,y1,z1)."""
    v = [
        (x0, y0, z0),
        (x1, y0, z0),
        (x1, y1, z0),
        (x0, y1, z0),
        (x0, y0, z1),
        (x1, y0, z1),
        (x1, y1, z1),
        (x0, y1, z1),
    ]
    _quad(v[0], v[3], v[2], v[1])  # bottom
    _quad(v[4], v[5], v[6], v[7])  # top
    _quad(v[0], v[1], v[5], v[4])  # front
    _quad(v[2], v[3], v[7], v[6])  # back
    _quad(v[3], v[0], v[4], v[7])  # left
    _quad(v[1], v[2], v[6], v[5])  # right


def _add_box_centered(cx, cy, cz, hw, hd, hh):
    """Box centered at (cx,cy,cz) with half-widths hw,hd,hh."""
    _add_box(cx - hw, cy - hd, cz - hh, cx + hw, cy + hd, cz + hh)


def _add_cylinder_z(cx, cy, z0, z1, r, n_seg=10):
    """Vertical cylinder along Z axis at (cx,cy), from z0 to z1."""
    for i in range(n_seg):
        a0 = 2 * math.pi * i / n_seg
        a1 = 2 * math.pi * (i + 1) / n_seg
        x0 = cx + r * math.cos(a0)
        y0 = cy + r * math.sin(a0)
        x1 = cx + r * math.cos(a1)
        y1 = cy + r * math.sin(a1)
        _tri((x0, y0, z0), (x1, y1, z0), (x1, y1, z1))
        _tri((x0, y0, z0), (x1, y1, z1), (x0, y0, z1))
        _tri((cx, cy, z0), (x1, y1, z0), (x0, y0, z0))
        _tri((cx, cy, z1), (x0, y0, z1), (x1, y1, z1))


def _add_cylinder_x(cx, y0, z0, x0, x1, r, n_seg=10):
    """Horizontal cylinder along X axis, from x0 to x1, at (y0,z0)."""
    for i in range(n_seg):
        a0 = 2 * math.pi * i / n_seg
        a1 = 2 * math.pi * (i + 1) / n_seg
        dy0 = r * math.cos(a0)
        dz0 = r * math.sin(a0)
        dy1 = r * math.cos(a1)
        dz1 = r * math.sin(a1)
        _tri(
            (x0, y0 + dy0, z0 + dz0),
            (x1, y0 + dy0, z0 + dz0),
            (x1, y0 + dy1, z0 + dz1),
        )
        _tri(
            (x0, y0 + dy0, z0 + dz0),
            (x1, y0 + dy1, z0 + dz1),
            (x0, y0 + dy1, z0 + dz1),
        )
        _tri((x0, y0, z0), (x0, y0 + dy1, z0 + dz1), (x0, y0 + dy0, z0 + dz0))
        _tri((x1, y0, z0), (x1, y0 + dy0, z0 + dz0), (x1, y0 + dy1, z0 + dz1))


# ---------------------------------------------------------------------------
# Component generators
# ---------------------------------------------------------------------------


def gen_chassis():
    """Twin-vertical-plate RC buggy chassis with belly pan, shock towers,
    front bumper, cross-members, and rear cage section."""
    plate_x0 = CHASSIS_REAR
    plate_x1 = CHASSIS_FRONT
    plate_y_outer = CHASSIS_PLATE_W / 2
    plate_y_inner = CHASSIS_SEP / 2

    # --- Twin vertical plates (left = +Y, right = -Y) ---
    for sign in (1, -1):
        y0 = sign * plate_y_inner
        y1 = sign * plate_y_outer
        _add_box(plate_x0, y0, CHASSIS_BOT, plate_x1, y1, CHASSIS_TOP)

    # --- Belly pan (flat plate at bottom connecting both sides) ---
    _add_box(
        CHASSIS_REAR,
        -plate_y_outer,
        CHASSIS_BOT - 0.002,
        CHASSIS_FRONT,
        plate_y_outer,
        CHASSIS_BOT,
    )

    # --- Top plate / deck ---
    _add_box(
        CHASSIS_REAR + 0.02,
        -plate_y_inner,
        CHASSIS_TOP,
        CHASSIS_FRONT - 0.01,
        plate_y_inner,
        CHASSIS_TOP + 0.0015,
    )

    # --- Cross-members (3 braces connecting left and right plates) ---
    for cx in [CHASSIS_REAR + 0.02, 0.10, 0.20]:
        _add_box(
            cx - 0.004,
            -plate_y_inner,
            CHASSIS_BOT + 0.003,
            cx + 0.004,
            plate_y_inner,
            CHASSIS_BOT + 0.012,
        )

    # --- Front bulkhead (thick box connecting plates at front) ---
    bh_x0 = FRONT_AXLE_X - 0.012
    bh_x1 = FRONT_AXLE_X + 0.012
    _add_box(
        bh_x0,
        -plate_y_outer,
        CHASSIS_BOT,
        bh_x1,
        plate_y_outer,
        CHASSIS_TOP + 0.005,
    )

    # --- Front shock tower (tall plate extending upward from bulkhead) ---
    tower_x0 = TOWER_BASE_X - TOWER_THICK
    tower_x1 = TOWER_BASE_X + TOWER_THICK
    _add_box(
        tower_x0,
        -plate_y_outer,
        CHASSIS_TOP,
        tower_x1,
        plate_y_outer,
        TOWER_TOP_Z,
    )
    # Tower top cross-bar (shock mount visual)
    _add_box(
        tower_x0 - 0.003,
        -plate_y_outer - 0.002,
        TOWER_TOP_Z - 0.003,
        tower_x1 + 0.003,
        plate_y_outer + 0.002,
        TOWER_TOP_Z + 0.001,
    )

    # --- Rear shock tower ---
    rt_x0 = REAR_TOWER_X - TOWER_THICK
    rt_x1 = REAR_TOWER_X + TOWER_THICK
    _add_box(
        rt_x0,
        -plate_y_outer,
        CHASSIS_TOP,
        rt_x1,
        plate_y_outer,
        REAR_TOWER_TOP_Z,
    )
    _add_box(
        rt_x0 - 0.003,
        -plate_y_outer - 0.002,
        REAR_TOWER_TOP_Z - 0.003,
        rt_x1 + 0.003,
        plate_y_outer + 0.002,
        REAR_TOWER_TOP_Z + 0.001,
    )

    # --- Rear bulkhead ---
    rbh_x0 = REAR_AXLE_X - 0.012
    rbh_x1 = REAR_AXLE_X + 0.012
    _add_box(
        rbh_x0,
        -plate_y_outer,
        CHASSIS_BOT,
        rbh_x1,
        plate_y_outer,
        CHASSIS_TOP + 0.003,
    )

    # --- Front bumper (curved skid plate made of stacked boxes) ---
    bumper_y = plate_y_outer + 0.010
    _add_box(
        FRONT_AXLE_X + 0.03,
        -bumper_y,
        BUMPER_BOT_Z,
        BUMPER_FRONT_X,
        bumper_y,
        BUMPER_BOT_Z + 0.003,
    )
    _add_box(
        BUMPER_FRONT_X - 0.003,
        -bumper_y,
        BUMPER_BOT_Z,
        BUMPER_FRONT_X,
        bumper_y,
        CHASSIS_BOT + 0.012,
    )
    _add_box(
        FRONT_AXLE_X + 0.015,
        -bumper_y + 0.002,
        CHASSIS_BOT,
        FRONT_AXLE_X + 0.03,
        bumper_y - 0.002,
        CHASSIS_BOT + 0.005,
    )
    _add_box(
        FRONT_AXLE_X + 0.025,
        -bumper_y + 0.001,
        0.022,
        BUMPER_FRONT_X,
        bumper_y - 0.001,
        BUMPER_BOT_Z,
    )

    # --- Rear bumper/cage (tubular frame at the back) ---
    rear_bumper_x = CHASSIS_REAR - 0.015
    for sign in (1, -1):
        ry = sign * (plate_y_outer + 0.008)
        _add_box(
            rear_bumper_x,
            ry - 0.002,
            CHASSIS_BOT,
            CHASSIS_REAR + 0.01,
            ry + 0.002,
            CHASSIS_BOT + 0.012,
        )
    # Cross-bar at the very back
    _add_box(
        rear_bumper_x,
        -plate_y_outer - 0.008,
        CHASSIS_BOT,
        rear_bumper_x + 0.004,
        plate_y_outer + 0.008,
        CHASSIS_BOT + 0.012,
    )
    # Upright bars at back
    for sign in (1, -1):
        ry = sign * (plate_y_outer + 0.008)
        _add_box(
            rear_bumper_x,
            ry - 0.002,
            CHASSIS_BOT,
            rear_bumper_x + 0.004,
            ry + 0.002,
            CHASSIS_TOP - 0.005,
        )
    # Top cross-bar
    _add_box(
        rear_bumper_x,
        -plate_y_outer - 0.008,
        CHASSIS_TOP - 0.007,
        rear_bumper_x + 0.004,
        plate_y_outer + 0.008,
        CHASSIS_TOP - 0.003,
    )

    # --- Motor mount pads (near rear axle) ---
    for sign in (1, -1):
        my = sign * (CHASSIS_SEP / 2 + CHASSIS_PLATE_THICK + 0.004)
        _add_box(
            REAR_AXLE_X - MOTOR_LEN / 2,
            my - 0.004,
            CHASSIS_BOT + 0.002,
            REAR_AXLE_X + MOTOR_LEN / 2,
            my + 0.004,
            CHASSIS_BOT + 0.008,
        )

    # --- LIDAR mount pad ---
    _add_box(
        FRONT_AXLE_X - 0.012,
        -0.012,
        CHASSIS_TOP + 0.001,
        FRONT_AXLE_X + 0.012,
        0.012,
        CHASSIS_TOP + 0.006,
    )


def gen_front_suspension_left():
    """Left front lower A-arm + upper arm + hub carrier (C-hub)."""
    hub_x = FRONT_AXLE_X
    hub_y = HALF_TRACK
    # Lower A-arm
    arm_x0 = FRONT_AXLE_X - 0.025
    arm_x1 = hub_x + 0.003
    _add_box(
        arm_x0,
        hub_y - 0.004,
        CHASSIS_BOT + 0.004,
        arm_x1,
        hub_y + 0.004,
        CHASSIS_BOT + 0.004 + ARM_H,
    )
    # Upper A-arm
    upper_arm_x0 = FRONT_AXLE_X - 0.010
    upper_arm_x1 = hub_x
    upper_arm_z = CHASSIS_TOP - 0.005
    _add_box(
        upper_arm_x0,
        hub_y - 0.003,
        upper_arm_z,
        upper_arm_x1,
        hub_y + 0.003,
        upper_arm_z + 0.003,
    )
    # Hub carrier
    hub_cz = WHEEL_Z
    _add_box_centered(hub_x + 0.002, hub_y, hub_cz, 0.004, 0.005, 0.014)
    # Steering knuckle arm
    _add_box(
        hub_x,
        hub_y - 0.006,
        hub_cz + 0.008,
        hub_x + 0.008,
        hub_y - 0.002,
        hub_cz + 0.011,
    )


def gen_front_suspension_right():
    """Mirror of left front suspension about Y=0."""
    hub_x = FRONT_AXLE_X
    hub_y = -HALF_TRACK
    arm_x0 = FRONT_AXLE_X - 0.025
    arm_x1 = hub_x + 0.003
    _add_box(
        arm_x0,
        hub_y - 0.004,
        CHASSIS_BOT + 0.004,
        arm_x1,
        hub_y + 0.004,
        CHASSIS_BOT + 0.004 + ARM_H,
    )
    upper_arm_x0 = FRONT_AXLE_X - 0.010
    upper_arm_x1 = hub_x
    upper_arm_z = CHASSIS_TOP - 0.005
    _add_box(
        upper_arm_x0,
        hub_y - 0.003,
        upper_arm_z,
        upper_arm_x1,
        hub_y + 0.003,
        upper_arm_z + 0.003,
    )
    hub_cz = WHEEL_Z
    _add_box_centered(hub_x + 0.002, hub_y, hub_cz, 0.004, 0.005, 0.014)
    _add_box(
        hub_x,
        hub_y + 0.002,
        hub_cz + 0.008,
        hub_x + 0.008,
        hub_y + 0.006,
        hub_cz + 0.011,
    )


def gen_rear_suspension_left():
    """Left rear lower A-arm + upper arm."""
    hub_x = REAR_AXLE_X
    hub_y = HALF_TRACK
    arm_x0 = REAR_AXLE_X + 0.020
    arm_x1 = hub_x - 0.003
    _add_box(
        arm_x1,
        hub_y - 0.004,
        CHASSIS_BOT + 0.004,
        arm_x0,
        hub_y + 0.004,
        CHASSIS_BOT + 0.004 + ARM_H,
    )
    upper_arm_x0 = REAR_AXLE_X + 0.012
    upper_arm_x1 = hub_x
    upper_arm_z = CHASSIS_TOP - 0.008
    _add_box(
        upper_arm_x1,
        hub_y - 0.003,
        upper_arm_z,
        upper_arm_x0,
        hub_y + 0.003,
        upper_arm_z + 0.003,
    )
    _add_box_centered(hub_x - 0.002, hub_y, WHEEL_Z, 0.004, 0.005, 0.014)


def gen_rear_suspension_right():
    """Mirror of left rear suspension about Y=0."""
    hub_x = REAR_AXLE_X
    hub_y = -HALF_TRACK
    arm_x0 = REAR_AXLE_X + 0.020
    arm_x1 = hub_x - 0.003
    _add_box(
        arm_x1,
        hub_y - 0.004,
        CHASSIS_BOT + 0.004,
        arm_x0,
        hub_y + 0.004,
        CHASSIS_BOT + 0.004 + ARM_H,
    )
    upper_arm_x0 = REAR_AXLE_X + 0.012
    upper_arm_x1 = hub_x
    upper_arm_z = CHASSIS_TOP - 0.008
    _add_box(
        upper_arm_x1,
        hub_y - 0.003,
        upper_arm_z,
        upper_arm_x0,
        hub_y + 0.003,
        upper_arm_z + 0.003,
    )
    _add_box_centered(hub_x - 0.002, hub_y, WHEEL_Z, 0.004, 0.005, 0.014)


def _gen_shock(tower_x, tower_z, arm_x, arm_z, y_offset, lean_x=-0.008):
    """Generate a coil-over shock absorber from tower mount to arm mount."""
    tx = tower_x + lean_x
    tz = tower_z - 0.005
    bx = arm_x
    bz = arm_z + 0.002
    y = y_offset
    dx = bx - tx
    dz = bz - tz
    n_seg = 5
    for i in range(n_seg):
        frac0 = i / n_seg
        frac1 = (i + 1) / n_seg
        sx0 = tx + dx * frac0
        sz0 = tz + dz * frac0
        sx1 = tx + dx * frac1
        sz1 = tz + dz * frac1
        r = SHOCK_BODY_R if frac0 < 0.5 else SHOCK_ROD_R
        _add_box(
            min(sx0, sx1) - 0.001,
            y - r,
            min(sz0, sz1) - r,
            max(sx0, sx1) + 0.001,
            y + r,
            max(sz0, sz1) + r,
        )
    # Spring coil rings (3 rings)
    for ring in range(3):
        frac = 0.2 + ring * 0.15
        rx = tx + dx * frac
        rz = tz + dz * frac
        ring_r = SHOCK_BODY_R + 0.001
        _add_box(
            rx - 0.001,
            y - ring_r,
            rz - 0.001,
            rx + 0.001,
            y + ring_r,
            rz + 0.001,
        )
    # Mount eyelets
    _add_box(
        tx - 0.002, y - 0.003, tz - 0.003, tx + 0.002, y + 0.003, tz + 0.003
    )
    _add_box(
        bx - 0.002, y - 0.003, bz - 0.003, bx + 0.002, y + 0.003, bz + 0.003
    )


def gen_front_shock_left():
    _gen_shock(
        TOWER_BASE_X,
        TOWER_TOP_Z,
        FRONT_AXLE_X - 0.015,
        CHASSIS_BOT + 0.004,
        HALF_TRACK - 0.006,
        lean_x=-0.010,
    )


def gen_front_shock_right():
    _gen_shock(
        TOWER_BASE_X,
        TOWER_TOP_Z,
        FRONT_AXLE_X - 0.015,
        CHASSIS_BOT + 0.004,
        -(HALF_TRACK - 0.006),
        lean_x=-0.010,
    )


def gen_rear_shock_left():
    _gen_shock(
        REAR_TOWER_X,
        REAR_TOWER_TOP_Z,
        REAR_AXLE_X + 0.015,
        CHASSIS_BOT + 0.004,
        HALF_TRACK - 0.006,
        lean_x=0.010,
    )


def gen_rear_shock_right():
    _gen_shock(
        REAR_TOWER_X,
        REAR_TOWER_TOP_Z,
        REAR_AXLE_X + 0.015,
        CHASSIS_BOT + 0.004,
        -(HALF_TRACK - 0.006),
        lean_x=0.010,
    )


def _gen_hub(cx, cy):
    """Wheel hub: cylindrical body + flange + axle stub."""
    cz = WHEEL_Z
    _add_cylinder_z(cx, cy, cz - HUB_W / 2, cz + HUB_W / 2, HUB_R, 10)
    _add_cylinder_z(
        cx,
        cy,
        cz - HUB_W / 2 - 0.001,
        cz - HUB_W / 2 + 0.001,
        HUB_R + 0.003,
        10,
    )
    inner_y = cy - (1 if cy > 0 else -1) * 0.008
    _add_cylinder_x(
        cx, (cy + inner_y) / 2, cz, min(cy, inner_y), max(cy, inner_y), 0.002, 8
    )


def gen_front_hub_left():
    _gen_hub(FRONT_AXLE_X, HALF_TRACK)


def gen_front_hub_right():
    _gen_hub(FRONT_AXLE_X, -HALF_TRACK)


def gen_rear_hub_left():
    _gen_hub(REAR_AXLE_X, HALF_TRACK)


def gen_rear_hub_right():
    _gen_hub(REAR_AXLE_X, -HALF_TRACK)


def _gen_motor(cx, cy):
    """Brushless motor can + gearbox housing + drive shaft."""
    cz = CHASSIS_BOT + 0.006
    # Motor can (cylinder along X axis)
    _add_cylinder_x(
        cx, cy, cz, cx - MOTOR_LEN / 2, cx + MOTOR_LEN / 2, MOTOR_R, 12
    )
    # Endbell (slightly larger disk at rear end)
    _add_cylinder_x(
        cx,
        cy,
        cz,
        cx - MOTOR_LEN / 2 - 0.001,
        cx - MOTOR_LEN / 2 + 0.001,
        MOTOR_R + 0.002,
        12,
    )
    # Drive shaft stub
    shaft_dir = 1 if cy > 0 else -1
    shaft_y_end = cy + shaft_dir * 0.015
    _add_cylinder_x(
        cx,
        (cy + shaft_y_end) / 2,
        cz,
        min(cy, shaft_y_end),
        max(cy, shaft_y_end),
        0.0015,
        8,
    )
    # Gearbox housing
    _add_box(
        cx - 0.006,
        min(cy, shaft_y_end),
        cz - MOTOR_R,
        cx + 0.006,
        max(cy, shaft_y_end),
        cz + MOTOR_R,
    )
    # Spur gear disk
    _add_cylinder_z(
        cx, cy, cz - MOTOR_R - 0.001, cz - MOTOR_R + 0.001, 0.008, 10
    )
    # Motor mount bracket
    _add_box(
        cx - MOTOR_LEN / 2,
        cy - 0.004,
        cz - MOTOR_R - 0.002,
        cx + MOTOR_LEN / 2,
        cy + 0.004,
        cz - MOTOR_R,
    )


def gen_motor_left():
    _gen_motor(REAR_AXLE_X, HALF_TRACK - 0.004)


def gen_motor_right():
    _gen_motor(REAR_AXLE_X, -(HALF_TRACK - 0.004))


def gen_jetson_board():
    """Jetson compute module: PCB + heatsink fins + fan + connectors."""
    px, py = 0.12, 0.0
    pz = CHASSIS_TOP + JETSON_PCB_H / 2 + 0.001
    # PCB
    _add_box_centered(px, py, pz, JETSON_W / 2, JETSON_D / 2, JETSON_PCB_H / 2)
    # Heatsink base
    hs_z = pz + JETSON_PCB_H / 2
    _add_box_centered(
        px,
        py,
        hs_z + JETSON_HEATSINK_H / 2,
        JETSON_W * 0.4,
        JETSON_D * 0.5,
        JETSON_HEATSINK_H / 2,
    )
    # Heatsink fins (7 thin plates)
    fin_w = JETSON_W * 0.38
    fin_h = JETSON_HEATSINK_H * 0.8
    for i in range(7):
        fx = px - fin_w / 2 + (i + 0.5) * fin_w / 7
        _add_box(
            fx - fin_w / 14,
            py - JETSON_D * 0.45,
            hs_z + 0.001,
            fx + fin_w / 14,
            py + JETSON_D * 0.45,
            hs_z + 0.001 + fin_h,
        )
    # Fan
    fan_z = hs_z + JETSON_HEATSINK_H + 0.002
    _add_cylinder_z(px + JETSON_W * 0.25, py, fan_z, fan_z + 0.004, 0.010, 12)
    # GPIO header
    for i in range(10):
        gx = px - JETSON_W * 0.4 + i * 0.003
        _add_box(
            gx,
            py + JETSON_D * 0.42,
            pz + JETSON_PCB_H / 2,
            gx + 0.002,
            py + JETSON_D * 0.46,
            pz + JETSON_PCB_H / 2 + 0.006,
        )
    # USB/Ethernet connectors
    _add_box(
        px + JETSON_W * 0.2,
        py - JETSON_D / 2 - 0.003,
        pz + JETSON_PCB_H / 2,
        px + JETSON_W * 0.45,
        py - JETSON_D / 2 + 0.001,
        pz + JETSON_PCB_H / 2 + 0.006,
    )
    # CSI camera port
    _add_box(
        px - 0.005,
        py + JETSON_D / 2,
        pz + JETSON_PCB_H / 2,
        px + 0.005,
        py + JETSON_D / 2 + 0.003,
        pz + JETSON_PCB_H / 2 + 0.004,
    )
    # Power connector
    _add_box(
        px - JETSON_W * 0.4,
        py - JETSON_D * 0.2,
        pz + JETSON_PCB_H / 2,
        px - JETSON_W * 0.35,
        py - JETSON_D * 0.1,
        pz + JETSON_PCB_H / 2 + 0.005,
    )


def gen_battery_pack():
    """LiPo battery pack with strap and cable exit."""
    bx, by = 0.04, 0.0
    bz = CHASSIS_BOT + BATT_H / 2 + 0.001
    _add_box_centered(bx, by, bz, BATT_W / 2, BATT_D / 2, BATT_H / 2)
    # Strap
    _add_box(
        bx - BATT_W / 2 - 0.001,
        by - BATT_D / 2 - 0.001,
        bz,
        bx + BATT_W / 2 + 0.001,
        by - BATT_D / 2 + 0.001,
        bz + BATT_H / 2 + 0.001,
    )
    # Cable exit
    _add_box(
        bx + BATT_W / 2,
        by - 0.004,
        bz - 0.002,
        bx + BATT_W / 2 + 0.008,
        by + 0.004,
        bz + 0.002,
    )


def gen_camera_box():
    """Front-facing camera with lens barrel and mount bracket."""
    cx, cy = FRONT_AXLE_X + 0.025, 0.0
    cz = CHASSIS_TOP + CAM_H / 2 + 0.002
    _add_box_centered(cx, cy, cz, CAM_W / 2, CAM_D / 2, CAM_H / 2)
    # Lens barrel (cylinder along X axis, pointing forward)
    _add_cylinder_x(
        cx, cy, cz, cx + CAM_W / 2, cx + CAM_W / 2 + 0.006, LENS_R, 10
    )
    # Mount bracket vertical arm
    _add_box(
        cx - 0.002,
        cy - CAM_D / 2 - 0.001,
        CHASSIS_TOP,
        cx + 0.002,
        cy + CAM_D / 2 + 0.001,
        cz - CAM_H / 2,
    )
    # Mount bracket horizontal foot
    _add_box(
        cx - 0.005,
        cy - CAM_D / 2 - 0.001,
        CHASSIS_TOP - 0.001,
        cx + 0.005,
        cy + CAM_D / 2 + 0.001,
        CHASSIS_TOP + 0.001,
    )


# ---------------------------------------------------------------------------
# Main: generate all meshes
# ---------------------------------------------------------------------------
def write_stl(path, tris, header=b"generated mesh"):
    """Write triangles to a binary STL file."""
    with open(path, "wb") as f:
        f.write(header.ljust(80, b"\x00"))
        f.write(struct.pack("<I", len(tris)))
        for a, b, c in tris:
            ux, uy, uz = b[0] - a[0], b[1] - a[1], b[2] - a[2]
            vx, vy, vz = c[0] - a[0], c[1] - a[1], c[2] - a[2]
            nx = uy * vz - uz * vy
            ny = uz * vx - ux * vz
            nz = ux * vy - uy * vx
            mag = (nx**2 + ny**2 + nz**2) ** 0.5 or 1.0
            f.write(struct.pack("<fff", nx / mag, ny / mag, nz / mag))
            for v in (a, b, c):
                f.write(struct.pack("<fff", *v))
            f.write(struct.pack("<H", 0))


MESH_DIR = Path(__file__).resolve().parent.parent / "meshes"

GENERATORS = [
    ("chassis.stl", gen_chassis, b"tvp chassis, generated"),
    ("front_susp_left.stl", gen_front_suspension_left, b"front susp L"),
    ("front_susp_right.stl", gen_front_suspension_right, b"front susp R"),
    ("rear_susp_left.stl", gen_rear_suspension_left, b"rear susp L"),
    ("rear_susp_right.stl", gen_rear_suspension_right, b"rear susp R"),
    ("front_shock_left.stl", gen_front_shock_left, b"front shock L"),
    ("front_shock_right.stl", gen_front_shock_right, b"front shock R"),
    ("rear_shock_left.stl", gen_rear_shock_left, b"rear shock L"),
    ("rear_shock_right.stl", gen_rear_shock_right, b"rear shock R"),
    ("front_hub_left.stl", gen_front_hub_left, b"front hub L"),
    ("front_hub_right.stl", gen_front_hub_right, b"front hub R"),
    ("rear_hub_left.stl", gen_rear_hub_left, b"rear hub L"),
    ("rear_hub_right.stl", gen_rear_hub_right, b"rear hub R"),
    ("motor_left.stl", gen_motor_left, b"motor L, generated"),
    ("motor_right.stl", gen_motor_right, b"motor R, generated"),
    ("jetson_board.stl", gen_jetson_board, b"jetson board, generated"),
    ("battery_pack.stl", gen_battery_pack, b"battery pack, generated"),
    ("camera_box.stl", gen_camera_box, b"camera box, generated"),
]


if __name__ == "__main__":
    for name, gen_fn, header in GENERATORS:
        triangles.clear()
        gen_fn()
        out = MESH_DIR / name
        write_stl(out, triangles, header)
        print(f"  {name:30s} {len(triangles):5d} triangles")
    print(f"\nWrote {len(GENERATORS)} meshes to {MESH_DIR}")
