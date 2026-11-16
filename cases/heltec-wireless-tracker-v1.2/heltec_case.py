"""
Parametric case for the Heltec Wireless Tracker V1.2 (no battery).
Makes the base and the lid as STL files (and, optionally, 2 button pins).
Use: pip install manifold3d trimesh && python3 heltec_case.py

Positions measured from a photo with a bank card for scale (estimated error ±0.5 mm).
Axes: X = length, x=0 at the PCB edge on the USB-C side; Y = width, y=0 at the centre, +Y = USER button side;
Z = height. Everything in mm.
"""
from manifold3d import Manifold, CrossSection, JoinType
import trimesh

P = dict(
    board_l=65.5, board_w=28.1,  # PCB (official 65.48 x 28.06)
    board_h=13.6,                # official total height (13.52), includes the underside
    usb_z=3.2,                   # USB-C centre above the bottom of the PCB (estimated)
    lcd_x=23.4, lcd_y=0.8,       # centre of the screen's visible area
    rst_x=3.4, rst_y=-8.0,       # RST button
    usr_x=3.4, usr_y=8.7,        # USER button
    led_x=8.9, led_y=6.4,        # charge LED
    btn_top=3.6,                 # top of the buttons above the bottom of the PCB (estimated)
    ant_dia=6.6, ant_y=0.0,      # hole for a panel-mount SMA connector (1/4" thread, 8 mm nut)
)

WALL = 1.8        # walls
FLOOR = 1.6       # floor
LID_T = 1.6       # lid
LIP_H = 4.0       # lid's snap lip (taller = flexes better and holds more)
SNAP_R = 0.6      # radius of the snap rib on the lip (sticks out 0.4 mm past the clearance)
SNAP_LEN = 18.0   # length of each rib
PRY_W = 10.0      # width of the notch to open it with a fingernail/screwdriver
CLR = 0.4         # clearance around the PCB
CLR_USB = 0.1     # clearance at the USB end: the board rests against the wall so the connector is flush
FIT = 0.2         # lid fit clearance
STANDOFF = 2.0    # height of the supports under the PCB (parts on the underside)
TOP_GAP = 0.6     # clearance above the tallest part
USB_W, USB_H = 10.0, 4.4    # tight opening around the USB-C connector (8.9 x 3.3)
PLUG_W, PLUG_H, PLUG_D = 12.5, 7.0, 1.2  # outside recess for the cable plug's plastic
LCD_W, LCD_H = 22.0, 11.5   # screen window (active area ~21.7 x 10.8)
BTN_HOLE = 3.4    # button pin hole
PIN_D = 3.0       # pin diameter
LED_D = 1.8       # LED hole
R = 2.0           # corner radius
LCD_WINDOW = False   # True = a window for the screen in the lid
BUTTON_PINS = False  # True = holes + pins for RST/USER
LED_HOLE = False     # small hole to see the charge LED
ANT_SPACE = 12.0     # free space after the PCB for the antenna adapter/pigtail


def rounded_rect(l, w, r):
    r = max(r, 0.01)
    return CrossSection.square([l - 2 * r, w - 2 * r], center=True).offset(r, JoinType.Round, circular_segments=32)


def box(l, w, h, r):
    return rounded_rect(l, w, r).extrude(h)


def cube(sx, sy, sz, cx, cy, z0):
    return Manifold.cube([sx, sy, sz]).translate([cx - sx / 2, cy - sy / 2, z0])


def cyl(d, h, x, y, z0, seg=40):
    return Manifold.cylinder(h, d / 2, d / 2, seg).translate([x, y, z0])


def build(p):
    in_l = p["board_l"] + CLR + CLR_USB + ANT_SPACE
    in_w = p["board_w"] + 2 * CLR
    in_h = p["board_h"] + TOP_GAP          # board_h already includes the space under the PCB
    out_l, out_w = in_l + 2 * WALL, in_w + 2 * WALL
    base_h = FLOOR + in_h

    # x (case coordinates) of the PCB edge on the USB side
    pcb_x0 = -in_l / 2 + CLR_USB
    X = lambda x: pcb_x0 + x
    pcb_bottom = FLOOR + STANDOFF

    # ---------- BASE ----------
    base = box(out_l, out_w, base_h, R) - box(in_l, in_w, base_h, 1.0).translate([0, 0, FLOOR])
    # Supports at the 4 corners (under the header holes, where there are no parts)
    for x in (X(2.0), X(p["board_l"] - 2.0)):
        for sy in (-1, 1):
            base += cube(3.5, 3.0, STANDOFF + 0.01, x, sy * (p["board_w"] / 2 - 1.5), FLOOR - 0.01)
    # USB-C
    uz = pcb_bottom + p["usb_z"]
    base -= cube(WALL * 3, USB_W, USB_H, -out_l / 2, 0, uz - USB_H / 2)
    base -= cube(PLUG_D * 2, PLUG_W, PLUG_H, -out_l / 2, 0, uz - PLUG_H / 2)
    # Antenna: SMA hole at half height, so the 8 mm nut clears both the floor and the lid
    base -= Manifold.cylinder(WALL * 3, p["ant_dia"] / 2, p["ant_dia"] / 2, 48) \
        .rotate([0, 90, 0]).translate([out_l / 2 - WALL * 1.5, p["ant_y"], FLOOR + in_h / 2])
    # Stops right after the PCB: the board can't slide into the antenna space
    # and the USB-C always stays against the wall
    stop_x = X(p["board_l"]) + CLR / 2 + 1.0
    for sy in (-1, 1):
        base += cube(2.0, 1.6, STANDOFF + 1.6 + 1.5, stop_x, sy * (p["board_w"] / 2 - 0.6), FLOOR - 0.01)

    # Snap grooves in the long walls (lined up with the lid's ribs)
    groove_z = base_h - LIP_H / 2
    for sx in (-1, 1):
        for sy in (-1, 1):
            g = Manifold.cylinder(SNAP_LEN + 1.0, SNAP_R + 0.15, SNAP_R + 0.15, 24, center=True).rotate([0, 90, 0])
            base -= g.translate([sx * (in_l - 2 * FIT) / 4, sy * in_w / 2, groove_z])
    # Notch in the outside rim of one long side, to open the lid
    base -= cube(PRY_W, WALL * 3, 1.2, 0, -out_w / 2, base_h - 1.2)

    # ---------- LID (printed upside down: when fitted it turns over X, so Y is mirrored) ----------
    lid = box(out_l, out_w, LID_T, R)
    lo_l, lo_w = in_l - 2 * FIT, in_w - 2 * FIT
    lid += (box(lo_l, lo_w, LIP_H, 1.0) - box(lo_l - 2.4, lo_w - 2.4, LIP_H, 0.5)).translate([0, 0, LID_T - 0.01])
    # Cut-out in the lip next to the antenna nut
    lid -= cube(4, 12.0, LIP_H + 1, lo_l / 2, 0, LID_T)
    # Snap ribs on the long sides (2 per side), they go into the base's grooves
    snap_z = LID_T + LIP_H / 2
    for sx in (-1, 1):
        for sy in (-1, 1):
            rib = Manifold.cylinder(SNAP_LEN, SNAP_R, SNAP_R, 24, center=True).rotate([0, 90, 0])
            lid += rib.translate([sx * lo_l / 4, sy * (lo_w / 2 - 0.05), snap_z])

    # distance from the inside of the lid to the top of the buttons
    lid_inner_z = FLOOR + in_h                    # on the assembled base
    btn_top_z = pcb_bottom + p["btn_top"]
    gap = lid_inner_z - btn_top_z                 # vertical space down to the button
    sleeve_h = min(4.0, gap - 2.0)

    # Screen window
    if LCD_WINDOW:
        lid -= cube(LCD_W, LCD_H, LID_T + LIP_H + 2, X(p["lcd_x"]), -p["lcd_y"], -1)
    # LED
    if LED_HOLE:
        lid -= cyl(LED_D, LID_T + 2, X(p["led_x"]), -p["led_y"], -1, 24)
    # Buttons: hole + guide sleeve on the inside
    for k in (("rst", "usr") if BUTTON_PINS else ()):
        x, y = X(p[k + "_x"]), -p[k + "_y"]
        lid += cyl(BTN_HOLE + 2.4, sleeve_h, x, y, LID_T - 0.01)
        lid -= cyl(BTN_HOLE, LID_T + sleeve_h + 2, x, y, -1)

    # ---------- BUTTON PINS ----------
    # Flange at the bottom (held under the sleeve), the shaft goes through the lid and sticks out 1 mm
    flange_t, flange_d = 0.8, BTN_HOLE + 1.6
    shaft_len = (gap - 0.3 - flange_t) + LID_T + 1.0
    pin = cyl(flange_d, flange_t, 0, 0, 0) + cyl(PIN_D, shaft_len + 0.01, 0, 0, flange_t - 0.01)
    pins = pin.translate([-5, 0, 0]) + pin.translate([5, 0, 0])

    return base, lid, pins, (out_l, out_w, base_h + LID_T), gap


def save(m, path):
    mesh = m.to_mesh()
    t = trimesh.Trimesh(vertices=mesh.vert_properties[:, :3], faces=mesh.tri_verts)
    t.export(path)
    return t


if __name__ == "__main__":
    base, lid, pins, dims, gap = build(P)
    ok = [save(base, "heltec_tracker_v1_2_base.stl").is_watertight,
          save(lid, "heltec_tracker_v1_2_lid.stl").is_watertight]
    if BUTTON_PINS:
        ok.append(save(pins, "heltec_tracker_v1_2_button_pins.stl").is_watertight)
    print("outside %.1f x %.1f x %.1f mm" % dims, "| space to the buttons %.1f mm" % gap, "| watertight", ok)
