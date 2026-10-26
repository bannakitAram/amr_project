#!/usr/bin/env python3
"""Touchscreen control panel for the AMR (1024x600 onboard display).

Screen layout follows document/ux_ui_design_specification_navigation_
control_systems_for_amr.md:

  Home ─┬─ General ─┬─ Auto   (pick a saved station, START / PAUSE / CANCEL)
        │           └─ Manual (external/rosboard joystick, v/w telemetry)
        └─ Developer (PIN) ─┬─ Mapping    (SLAM bringup, live map, save)
                            ├─ Navigation (tap-to-goal map, motion tests, log)
                            └─ Settings   (general info / read-only params)

Start/stop of the bringup is still done by spawning `ros2 launch` /
`ros2 run` subprocesses exactly as before. Live data (map, pose, odom,
cmd_vel, watchdogs, rosout) comes from a small rclpy node spun in a
background thread (RosBridge) - it only subscribes and publishes the same
goal/cancel topics the rosboard goal_picker uses, it runs no control loop.

The header's STOP button is a SOFTWARE stop (cancel Nav2 goal + motion
preset, publish zero /cmd_vel). It is NOT a safety-rated emergency stop -
the hardware E-stop circuit remains the ISO 3691-4 / ISO 13850 stop.
"""
import collections
import json
import glob
import math
import os
import re
import signal
import socket
import subprocess
import sys
import threading
import time
import tkinter as tk
from tkinter import font as tkfont
from tkinter import messagebox as _tk_messagebox, ttk

from urllib.parse import quote

import numpy as np
import qrcode
import yaml
from PIL import Image, ImageTk

from ament_index_python.packages import get_package_share_directory

import amr_calib_panel

MAPS_DIR = os.path.join(
    get_package_share_directory('amr_navigation'), '..', '..', '..', '..',
    'src', 'amr_navigation', 'maps')
MAPS_DIR = os.path.normpath(MAPS_DIR)
NAV2_PARAMS = os.path.normpath(os.path.join(MAPS_DIR, '..', 'config', 'nav2_params.yaml'))
LASER_FILTER = os.path.normpath(os.path.join(
    MAPS_DIR, '..', '..', 'amr_bringup', 'config', 'laser_filter.yaml'))
REPO_DIR = os.path.normpath(os.path.join(MAPS_DIR, '..', '..', '..', '..'))
MAP_EDIT = os.path.normpath(os.path.join(MAPS_DIR, '..', 'scripts', 'map_edit.py'))

ROSBOARD_PORT = 8888

# Developer-mode PIN (4-6 digits) is read from this file; DEV_PIN_DEFAULT is
# used only while the file is missing, and the PIN modal says so.
DEV_PIN_FILE = os.path.expanduser('~/.amr_dev_pin')
DEV_PIN_DEFAULT = '1234'

# Auto-mode "arrived" heuristic (Nav2's own result is not published by
# nav_goal_executor_node): within this distance of the goal and nearly still.
ARRIVE_DIST_M = 0.30
ARRIVE_SPEED = 0.03

# "Move forward" straight-line command (Control panel button, not the
# rosboard joystick). Closed-loop on /odometry/filtered, not open-loop
# timing, so it isn't thrown off by wheel slip/PID variance. Speed matches
# the rosboard joystick's walking-pace cap (see joystick.js) for consistency.
MOVE_FORWARD_SPEED_DEFAULT = 0.15  # m/s - matches the rosboard joystick's walking-pace cap
MOVE_FORWARD_SPEED_MIN = 0.05     # m/s - stepper floor
MOVE_FORWARD_SPEED_MAX = 0.40     # m/s - diagnostic ceiling; ISO 3691-4 review this before raising
MOVE_FORWARD_SPEED_STEP = 0.05    # m/s - +/- button increment
MOVE_FORWARD_STEP_M = 0.5        # m - +/- button increment
MOVE_FORWARD_MIN_M = MOVE_FORWARD_STEP_M  # m - stepper floor (one step)
MOVE_FORWARD_MAX_M = 5.0         # m - hard UI cap; raised from 3.0 - review path length vs. sensor range/stopping distance under ISO 3691-4 before raising further
MOVE_FORWARD_TIMEOUT_FACTOR = 3.0  # abort if it takes > (distance/speed)*this - odom stall/stuck robot

# "Square path" command: 4x {straight leg, 90 degree CCW turn}, closed-loop
# the same way as "Move forward" (see square_path_node.py's module
# docstring). Side length capped lower than a single straight move's cap -
# it is 4 legs run back-to-back with no operator confirmation in between,
# so review ISO 3691-4 stopping-distance/path-length implications before
# raising it.
SQUARE_SIDE_DEFAULT = 1.0  # m
SQUARE_SIDE_MIN = 0.5      # m - stepper floor
SQUARE_SIDE_MAX = 2.0      # m - hard UI cap per side (4 sides = 8m path)
SQUARE_SIDE_STEP = 0.5     # m - +/- button increment
SQUARE_TURN_SPEED = 0.3    # rad/s - matches square_path_node.py's default turn rate

# "Circle path" command: constant-curvature arc, closed-loop on
# /odometry/filtered yaw (see circle_path_node.py's module docstring).
# Angular speed is fixed (not operator-adjustable) per spec - radius is the
# only speed-affecting control, so its UI cap is chosen to keep
# linear.x = CIRCLE_ANGULAR_SPEED * radius under MOVE_FORWARD_SPEED_MAX.
CIRCLE_ANGULAR_SPEED = 0.2   # rad/s - fixed
CIRCLE_RADIUS_DEFAULT = 0.8  # m
CIRCLE_RADIUS_MIN = 0.3      # m - stepper floor
CIRCLE_RADIUS_MAX = 1.5      # m - hard UI cap; 1.5*0.2=0.30 m/s, under MOVE_FORWARD_SPEED_MAX (0.40 m/s)
CIRCLE_RADIUS_STEP = 0.1     # m - +/- button increment
CIRCLE_ANGLE_DEFAULT = 360   # deg - full circle
CIRCLE_ANGLE_MIN = 45        # deg - stepper floor
CIRCLE_ANGLE_MAX = 360       # deg - hard UI cap per run (one full circle)
CIRCLE_ANGLE_STEP = 45       # deg - +/- button increment

# foxglove_bridge WebSocket port (see amr_bringup bringup.launch.py's
# 'foxglove_port' launch arg, default 8765) and the static file server that
# hosts the self-built Lichtblick web bundle (built once from
# lichtblick-suite/lichtblick's `web:build:prod`; see AMR_Project docs) so
# phones on the same LAN can open a full Foxglove-style map view with no
# internet access required.
FOXGLOVE_WS_PORT = 8765
LICHTBLICK_PORT = 8098
LICHTBLICK_DIST_DIR = '/home/nvidia/lichtblick_web/dist'

# Combined mobile page: Lichtblick's map view on top + rosboard's existing,
# already touch-safe virtual joystick (Pointer Events, deadman-switch,
# ISO 3691-4 speed clamp - see rosboard/html/js/joystick.js) cropped in at
# the bottom. Lichtblick's own stock Teleop panel is NOT used for driving -
# it only handles mouse events (mousedown/mouseup/mouseleave), which mobile
# browsers do not reliably fire for a press-and-hold drag, so a released
# finger could fail to send the stop command until the STM32's 1s cmd_vel
# timeout kicks in. rosboard's joystick has no such gap.
MOBILE_PORT = 8099
MOBILE_DIST_DIR = '/home/nvidia/lichtblick_web/mobile'

# Topics auto-subscribed on the rosboard page so the operator's phone shows
# the right panels the moment the QR code is scanned - no manual picking.
MAPPING_MONITOR_TOPICS = {
    # /map opens rosboard's combined Map+Scan viewer (see MapScanViewer.js),
    # which pulls /scan in on its own via TF - no need to list it here too.
    '/map': 'nav_msgs/msg/OccupancyGrid',
    # NOTE: the EKF (ekf.yaml) publishes fused odometry on /odometry/filtered,
    # not /odom - using the wrong name here would just show "Waiting for data".
    '/odometry/filtered': 'nav_msgs/msg/Odometry',
}


def build_rosboard_url(ip, topics):
    base = f'http://{ip}:{ROSBOARD_PORT}/'
    if not topics:
        return base
    params = '&'.join(
        f'{quote(name.replace("/", chr(92)))}={quote(msg_type.replace("/", chr(92)))}'
        for name, msg_type in topics.items())
    return f'{base}?{params}'


def build_lichtblick_url(ip):
    # ds=foxglove-websocket + ds.url auto-opens the connection on load, so
    # scanning the QR code lands straight on the live map - no manual
    # "Open connection" step needed on the phone.
    return (f'http://{ip}:{LICHTBLICK_PORT}/'
            f'?ds=foxglove-websocket&ds.url=ws://{ip}:{FOXGLOVE_WS_PORT}')


def build_mobile_url(ip):
    # The mobile wrapper page reads window.location.hostname itself to
    # build its Lichtblick/rosboard iframe URLs, so it needs no query
    # params - it works the same regardless of which IP the phone used.
    return f'http://{ip}:{MOBILE_PORT}/'


def build_goal_picker_url(ip):
    # Served by rosboard itself (html/goal_picker/, same origin as its
    # websocket at /rosboard/v1) rather than a separate static server/port -
    # see goal_picker/index.html's module docstring for why that matters
    # (same-origin websockets, no CORS/check_origin complications).
    return f'http://{ip}:{ROSBOARD_PORT}/goal_picker/'


# UI scale: the layout is designed at 1024x600 (S=1). On the 7" 1920x1080
# panel S=1.8, so every widget keeps the same physical size it had on a
# 7" 1024x600 screen (touch targets ~7 mm). Override with AMR_PANEL_SCALE.
_SCALE = {'v': 1.0}


def px(n):
    return int(round(n * _SCALE['v']))


# Dark industrial theme (spec section 3): green = ready/safe, amber =
# warning/manual override, red = stop/fault.
BG = '#12161c'
SURFACE = '#1c222b'
SURFACE_2 = '#262e39'
BORDER = '#364152'
TEXT_MAIN = '#e8ecf1'
TEXT_MUTED = '#93a0b0'
GREEN = '#2e9e48'
GREEN_DARK = '#237a37'
AMBER = '#e89a1c'
AMBER_DARK = '#b97a12'
RED = '#d93636'
RED_DARK = '#a82828'
BLUE = '#2f7fd8'
BLUE_DARK = '#2263ab'
DISABLED_BG = '#2a313b'
DISABLED_FG = '#5d6876'
# Station pin colours, by position in the stations file (cycles after 8).
# Green / amber / red are left out: robot arrow, goal / selection, faults.
# Sent with /amr_stations so the phone goal_picker uses the same colours.
STATION_COLORS = ('#1e88e5', '#8e24aa', '#00acc1', '#d81b60',
                  '#3949ab', '#8d6e63', '#c0ca33', '#00897b')


def station_color(i):
    return STATION_COLORS[i % len(STATION_COLORS)]

BTN_KINDS = {
    'neutral': (SURFACE_2, '#313b48', TEXT_MAIN),
    'primary': (GREEN, GREEN_DARK, 'white'),
    'info': (BLUE, BLUE_DARK, 'white'),
    'warn': (AMBER, AMBER_DARK, '#1a1a1a'),
    'danger': (RED, RED_DARK, 'white'),
}

# Map canvas colours (occupancy grid -> RGB)
MAP_UNKNOWN = (28, 34, 42)
MAP_FREE = (64, 74, 88)
MAP_OCC = (232, 237, 243)


def sanitize_map_name(raw):
    name = raw.strip().replace(' ', '_')
    name = re.sub(r'[^A-Za-z0-9_\-]', '', name)
    return name


def get_local_ip():
    """Best-effort LAN IP of this machine (no packets actually sent - UDP
    connect() just makes the kernel pick the outbound interface/address)."""
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect(('10.255.255.255', 1))
        return s.getsockname()[0]
    except OSError:
        return '127.0.0.1'
    finally:
        s.close()


def get_rosboard_pin():
    """Reads the access PIN rosboard.py generates and writes to disk on
    startup (see rosboard.py's __init__). Returns None if rosboard hasn't
    written one yet (not running, or old version without the PIN gate)."""
    try:
        with open(os.path.expanduser('~/.rosboard_pin')) as f:
            return f.read().strip()
    except OSError:
        return None


def get_dev_pin():
    """Returns (pin, is_default)."""
    try:
        with open(DEV_PIN_FILE) as f:
            pin = f.read().strip()
        if re.fullmatch(r'\d{4,6}', pin):
            return pin, False
    except OSError:
        pass
    return DEV_PIN_DEFAULT, True


def is_port_open(host, port, timeout=0.5):
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.settimeout(timeout)
    try:
        return s.connect_ex((host, port)) == 0
    finally:
        s.close()


def stations_path(map_name):
    return os.path.join(MAPS_DIR, f'{map_name}_stations.yaml')


def load_stations(map_name):
    """Stations saved for a map: list of {name, x, y, theta} (map frame)."""
    try:
        with open(stations_path(map_name)) as f:
            data = yaml.safe_load(f) or {}
        out = []
        for s in data.get('stations', []):
            out.append({'name': str(s['name']), 'x': float(s['x']),
                        'y': float(s['y']), 'theta': float(s.get('theta', 0.0))})
        return out
    except (OSError, KeyError, TypeError, ValueError, yaml.YAMLError):
        return []


def save_stations(map_name, stations):
    with open(stations_path(map_name), 'w') as f:
        yaml.safe_dump({'stations': stations}, f, sort_keys=False)


def load_map_image(yaml_path):
    """(meta, RGB PIL image) of a saved map_server map, in the panel's map
    colours (same thresholds as map_server's trinary mode)."""
    with open(yaml_path) as f:
        meta = yaml.safe_load(f)
    img_path = meta['image']
    if not os.path.isabs(img_path):
        img_path = os.path.join(os.path.dirname(yaml_path), img_path)
    g = np.asarray(Image.open(img_path).convert('L'), dtype=np.float32) / 255.0
    occ = g if meta.get('negate', 0) else 1.0 - g
    rgb = np.empty(g.shape + (3,), dtype=np.uint8)
    rgb[:] = MAP_UNKNOWN
    rgb[occ < meta.get('free_thresh', 0.25)] = MAP_FREE
    rgb[occ > meta.get('occupied_thresh', 0.65)] = MAP_OCC
    return meta, Image.fromarray(rgb, 'RGB')


# Nav2 parameters editable from Settings > Parameters. Each is edited as
# text in nav2_params.yaml (comments kept) and every related copy is kept in
# sync: speed -> MPPI + velocity_smoother (+ behavior_server for w);
# inflation -> local + global; footprint -> local + global + the
# collision_monitor FrontStop polygon, which starts at the front bumper,
# + the laser_filter.yaml self-hit box (footprint + LASER_BOX_MARGIN).
# The footprint is edited as 4 edge distances from base_footprint so a
# payload mounted off-centre gives an asymmetric rectangle.
# The min/max bounds are hard limits - review ISO 3691-4 before widening:
# FrontStop (0.35 m deep) and the decel limits were sized for 0.2 m/s.
NAV_PARAM_SPECS = [
    # key, label, unit, min, max, step, fmt
    ('v_max', 'v_max', 'm/s', 0.05, 0.30, 0.01, '{:.2f}'),
    ('w_max', 'ω_max', 'rad/s', 0.10, 1.00, 0.05, '{:.2f}'),
    ('inflation', 'Inflation radius', 'm', 0.30, 1.30, 0.05, '{:.2f}'),
    ('goal_tol', 'xy_goal_tolerance', 'm', 0.10, 0.50, 0.05, '{:.2f}'),
    # footprint edges, distance from base_footprint (wheel axle centre).
    # Lower bounds = bare chassis (0.49 / 0.26): smaller would cut the
    # self-hit box into the body and re-expose its returns. Upper bounds:
    # local costmap is 3 x 3 m, so edges past ~1.0 m leave MPPI too little
    # window ahead - grow width/height there first if more is needed.
    ('fp_front', 'กลางรถ→ขอบหน้า (+x)', 'm', 0.49, 1.00, 0.01, '{:.2f}'),
    ('fp_rear', 'กลางรถ→ขอบหลัง (-x)', 'm', 0.49, 1.00, 0.01, '{:.2f}'),
    ('fp_left', 'กลางรถ→ขอบซ้าย (+y)', 'm', 0.26, 0.70, 0.01, '{:.2f}'),
    ('fp_right', 'กลางรถ→ขอบขวา (-y)', 'm', 0.26, 0.70, 0.01, '{:.2f}'),
    # velocity_smoother max_accel / max_decel [x, -, theta]. Upper bounds =
    # STM32 ramp (MAX_ACCEL 0.5 / MAX_DECEL 1.2 m/s^2 per wheel; /0.236 m
    # half track for theta) - beyond that the firmware caps it anyway.
    ('acc_lin', 'เร่ง (accel)', 'm/s²', 0.10, 0.50, 0.05, '{:.2f}'),
    ('dec_lin', 'เบรก (decel)', 'm/s²', 0.30, 1.20, 0.05, '{:.2f}'),
    ('acc_rot', 'เร่งหมุน (α)', 'rad/s²', 0.30, 2.00, 0.10, '{:.2f}'),
    ('dec_rot', 'เบรกหมุน', 'rad/s²', 0.50, 2.00, 0.10, '{:.2f}'),
]
# "ค่า Default" button: last committed nav2_params.yaml set (9f81a2e,
# bare chassis footprint; FrontStop/decel were sized for this 0.2 m/s).
# Loads into the editor only - Save still asks for confirmation. Update
# this when a new tested set is tagged.
NAV_PARAM_DEFAULTS = {
    'v_max': 0.20, 'w_max': 0.50, 'inflation': 0.55, 'goal_tol': 0.10,
    'fp_front': 0.49, 'fp_rear': 0.49, 'fp_left': 0.26, 'fp_right': 0.26,
    'acc_lin': 0.30, 'dec_lin': 0.50, 'acc_rot': 1.00, 'dec_rot': 1.50,
}
STOP_LATENCY_S = 0.2      # cmd -> wheels, used for the stop-distance note
FRONT_STOP_DEPTH = 0.35   # m ahead of the bumper (matches nav2_params.yaml)
FRONT_STOP_MARGIN = 0.03  # m each side beyond the footprint half-width
LASER_BOX_MARGIN = 0.01   # m self-hit box beyond the footprint (laser_filter.yaml)

_NUM = r'(-?\d+(?:\.\d+)?)'
_PARAM_RES = {
    'vx_max': re.compile(r'^(\s*vx_max:\s*)' + _NUM, re.M),
    'wz_max': re.compile(r'^(\s*wz_max:\s*)' + _NUM, re.M),
    'max_vel': re.compile(r'^(\s*max_velocity:\s*\[)' + _NUM + r',\s*' + _NUM + r',\s*' + _NUM + r'\]', re.M),
    'min_vel': re.compile(r'^(\s*min_velocity:\s*\[)' + _NUM + r',\s*' + _NUM + r',\s*' + _NUM + r'\]', re.M),
    'rot_vel': re.compile(r'^(\s*max_rotational_vel:\s*)' + _NUM, re.M),
    'inflation': re.compile(r'^(\s*inflation_radius:\s*)' + _NUM, re.M),
    'goal_tol': re.compile(r'^(\s*xy_goal_tolerance:\s*)' + _NUM, re.M),
    'max_acc': re.compile(r'^(\s*max_accel:\s*\[)' + _NUM + r',\s*' + _NUM + r',\s*' + _NUM + r'\]', re.M),
    'max_dec': re.compile(r'^(\s*max_decel:\s*\[)' + _NUM + r',\s*' + _NUM + r',\s*' + _NUM + r'\]', re.M),
    'footprint': re.compile(r'^(\s*footprint:\s*)"\[\[([^"]*)\]\]"', re.M),
    'front_stop': re.compile(r'^(\s*points:\s*)\[' + ',\\s*'.join([_NUM] * 8) + r'\]', re.M),
}
_PARAM_COUNTS = {'inflation': 2, 'footprint': 2}


def read_nav_params(text):
    """Current editable values from nav2_params.yaml text (raises
    ValueError if the file no longer has the expected layout)."""
    for k, rx in _PARAM_RES.items():
        n = len(rx.findall(text))
        if n != _PARAM_COUNTS.get(k, 1):
            raise ValueError(f'nav2_params.yaml: expected {_PARAM_COUNTS.get(k, 1)} x {k}, found {n}')
    pts = [[float(v) for v in p.split(',')]
           for p in _PARAM_RES['footprint'].search(text).group(2).split('], [')]
    xs, ys = [p[0] for p in pts], [p[1] for p in pts]
    return {
        'v_max': float(_PARAM_RES['vx_max'].search(text).group(2)),
        'w_max': float(_PARAM_RES['wz_max'].search(text).group(2)),
        'inflation': float(_PARAM_RES['inflation'].search(text).group(2)),
        'goal_tol': float(_PARAM_RES['goal_tol'].search(text).group(2)),
        'fp_front': round(max(xs), 3),
        'fp_rear': round(-min(xs), 3),
        'fp_left': round(max(ys), 3),
        'fp_right': round(-min(ys), 3),
        'acc_lin': float(_PARAM_RES['max_acc'].search(text).group(2)),
        'acc_rot': float(_PARAM_RES['max_acc'].search(text).group(4)),
        'dec_lin': -float(_PARAM_RES['max_dec'].search(text).group(2)),
        'dec_rot': -float(_PARAM_RES['max_dec'].search(text).group(4)),
    }


def write_nav_params(text, v):
    """Returns text with every copy of the edited values replaced.
    Footprint is written as a rectangle that may be off-centre."""
    read_nav_params(text)  # layout check before touching anything
    f2 = lambda x: f'{x:.2f}'  # noqa: E731
    fr, rr, lf, rt = v['fp_front'], v['fp_rear'], v['fp_left'], v['fp_right']
    sl, sr = lf + FRONT_STOP_MARGIN, rt + FRONT_STOP_MARGIN
    text = _PARAM_RES['vx_max'].sub(lambda m: m.group(1) + f2(v['v_max']), text)
    text = _PARAM_RES['wz_max'].sub(lambda m: m.group(1) + f2(v['w_max']), text)
    text = _PARAM_RES['max_vel'].sub(
        lambda m: f"{m.group(1)}{f2(v['v_max'])}, {m.group(3)}, {f2(v['w_max'])}]", text)
    text = _PARAM_RES['min_vel'].sub(
        lambda m: f"{m.group(1)}{f2(-v['v_max'])}, {m.group(3)}, {f2(-v['w_max'])}]", text)
    text = _PARAM_RES['max_acc'].sub(
        lambda m: f"{m.group(1)}{f2(v['acc_lin'])}, {m.group(3)}, {f2(v['acc_rot'])}]", text)
    text = _PARAM_RES['max_dec'].sub(
        lambda m: f"{m.group(1)}{f2(-v['dec_lin'])}, {m.group(3)}, {f2(-v['dec_rot'])}]", text)
    text = _PARAM_RES['rot_vel'].sub(lambda m: m.group(1) + f2(v['w_max']), text)
    text = _PARAM_RES['inflation'].sub(lambda m: m.group(1) + f2(v['inflation']), text)
    text = _PARAM_RES['goal_tol'].sub(lambda m: m.group(1) + f2(v['goal_tol']), text)
    fp = (f'[[{fr:.2f}, {lf:.2f}], [{fr:.2f}, {-rt:.2f}], '
          f'[{-rr:.2f}, {-rt:.2f}], [{-rr:.2f}, {lf:.2f}]]')
    text = _PARAM_RES['footprint'].sub(lambda m: f'{m.group(1)}"{fp}"', text)
    fx = fr + FRONT_STOP_DEPTH
    text = _PARAM_RES['front_stop'].sub(
        lambda m: (f'{m.group(1)}[{fx:.2f}, {sl:.2f}, {fx:.2f}, {-sr:.2f}, '
                   f'{fr:.2f}, {-sr:.2f}, {fr:.2f}, {sl:.2f}]'), text)
    return text


_BOX_KEYS = ('min_x', 'max_x', 'min_y', 'max_y')
_BOX_RES = {k: re.compile(r'^(\s*' + k + r':\s*)' + _NUM, re.M) for k in _BOX_KEYS}


def laser_box_for(v):
    """Self-hit box (laser_filter.yaml) for footprint edges v."""
    m = LASER_BOX_MARGIN
    return {'min_x': round(-(v['fp_rear'] + m), 2), 'max_x': round(v['fp_front'] + m, 2),
            'min_y': round(-(v['fp_right'] + m), 2), 'max_y': round(v['fp_left'] + m, 2)}


def read_laser_box(text):
    for k, rx in _BOX_RES.items():
        if len(rx.findall(text)) != 1:
            raise ValueError(f'laser_filter.yaml: expected 1 x {k}')
    return {k: float(rx.search(text).group(2)) for k, rx in _BOX_RES.items()}


def write_laser_box(text, v):
    read_laser_box(text)
    for k, val in laser_box_for(v).items():
        text = _BOX_RES[k].sub(lambda m, val=val: m.group(1) + f'{val:.2f}', text)
    return text


def find_key(tree, key):
    """First value of `key` anywhere in a nested dict (nav2_params.yaml)."""
    if isinstance(tree, dict):
        if key in tree:
            return tree[key]
        for v in tree.values():
            r = find_key(v, key)
            if r is not None:
                return r
    return None


# ---------------------------------------------------------------------
# UI language (Thai / English). Widgets are built with Thai text; T()
# translates a whole string, or failing that each known Thai fragment in
# it (longest first), so f-strings with numbers/names still translate.
# AmrControlPanel._i18n_walk() re-applies T() to every widget's text.
LANG_FILE = os.path.expanduser('~/.amr_panel_lang')
_LANG = {'cur': 'th'}

EN = {
    # Home / menus
    'เลือกโหมดการทำงาน': 'Select operating mode',
    'Auto: ส่งหุ่นไปยัง Station · Manual: ขับด้วย Joystick':
        'Auto: send robot to a station · Manual: drive by joystick',
    'Mapping · Navigation Test · Settings (ต้องใส่ PIN)':
        'Mapping · Navigation Test · Settings (PIN required)',
    'เลือกจุดหมาย (Station) แล้วกด START MISSION': 'Pick a station, then START MISSION',
    'ขับด้วย Joystick ภายนอก / มือถือ (rosboard)': 'Drive with external / phone joystick (rosboard)',
    '‹  กลับ': '‹  Back',
    'พร้อมใช้งาน': 'Ready',
    'กำลังทำงาน': 'Running',
    'กำลังเริ่มระบบ...': 'Starting...',
    'กำลังปิดระบบ...': 'Shutting down...',
    'ระบบหยุดทำงานผิดปกติ': 'System stopped unexpectedly',
    'โหมดสร้างแผนที่ (Mapping)': 'Mapping mode',
    'โหมดนำทาง (Navigation)': 'Navigation mode',
    'โหมดควบคุมเอง (Manual)': 'Manual mode',
    'แผนที่: ': 'Map: ',
    'ยกเลิก goal + ส่ง cmd_vel=0 แล้ว · ไม่ใช่ E-stop ระดับ safety':
        'Goal cancelled + cmd_vel=0 sent · not a safety-rated E-stop',
    # PIN
    'ใส่ PIN เพื่อเข้าโหมดนักพัฒนา': 'Enter PIN for Developer Mode',
    'ยังไม่ได้ตั้ง PIN - ใช้ค่าเริ่มต้น (ตั้งได้ที่ ': 'No PIN set - using default (set in ',
    '4-6 หลัก': '4-6 digits',
    'PIN ไม่ถูกต้อง': 'Wrong PIN',
    '▼  เลื่อนลงเพื่อดูค่าเพิ่มเติม': '▼  Scroll down for more',
    'เริ่มสแกน': 'Start scan',
    'หยุดสแกนชั่วคราว': 'Pause scan',
    'กำลังเริ่ม SLAM...': 'Starting SLAM...',
    'พร้อม - กดเริ่มสแกน': 'Ready - press Start scan',
    'ไม่พบ scan gate - เริ่มสแกนไม่ได้': 'scan gate not found - cannot scan',
    'กำลังสแกน': 'Scanning',
    'สั่ง scan gate ไม่สำเร็จ': 'scan gate call failed',
    'ปิด Mapping จากมือถือ': 'Mapping closed from phone',
    '● กำลังสแกน · ': '● SCANNING · ',
    '❚❚ ไม่ได้สแกน · ': '❚❚ NOT SCANNING · ',
    'กำลังบันทึก ': 'Saving ',
    'บันทึกแล้ว: ': 'Saved: ',
    'บันทึกแผนที่ไม่สำเร็จ': 'Map save failed',
    ' อยู่แล้ว - ใช้ชื่ออื่น': ' exists - use another name',
    'กลางรถ→ขอบหน้า (+x)': 'Centre→front (+x)',
    'กลางรถ→ขอบหลัง (-x)': 'Centre→rear (-x)',
    'กลางรถ→ขอบซ้าย (+y)': 'Centre→left (+y)',
    'กลางรถ→ขอบขวา (-y)': 'Centre→right (-y)',
    'กล่องกรอง LiDAR (self-hit)': 'LiDAR self-hit box',
    'Footprint รวม': 'Footprint overall',
    'Footprint = ระยะจากกลางรถ (กลางล้อขับ) ถึงขอบนอกสุด ของรถหรือ payload ที่ยื่นมากกว่า - ไม่ใช่ระยะที่ยื่นออกจากตัวรถ':
        'Footprint = distance from robot centre (drive-wheel midpoint) to the outermost edge of '
        'robot or payload - not how far the payload sticks out',
    'บันทึกลงไฟล์': 'Save to file',
    'คืนค่าจากไฟล์': 'Revert',
    'ค่า Default': 'Defaults',
    'อ่านไฟล์ไม่ได้ - แก้ผ่านไฟล์เท่านั้น': 'Cannot parse file - edit the file directly',
    'รัศมีมุมหุ่น': 'Corner radius',
    '⚠ inflation น้อยกว่ารัศมีมุมหุ่น - หุ่นอาจวิ่งเฉียดสิ่งกีดขวางตอนหมุน':
        '⚠ inflation < corner radius - robot may clip obstacles when turning',
    'FrontStop เริ่มที่หน้ากันชน': 'FrontStop starts at bumper',
    'ลึก': 'depth',
    'ค่าที่จะเปลี่ยน:': 'Pending changes:',
    'มีผลเมื่อเริ่ม Navigation/Mapping ครั้งถัดไป · ': 'Applies at next Navigation/Mapping start · ',
    'ต้อง commit + ทดสอบ + tag ก่อนใช้บนไลน์': 'commit + test + tag before line use',
    'บันทึก Parameter': 'Save parameters',
    'ค่าเหล่านี้เกี่ยวกับ Safety (ISO 3691-4) - หลังบันทึกต้องทดสอบระยะหยุด/':
        'These are safety-related (ISO 3691-4) - after saving, test stopping distance/',
    'การหลบสิ่งกีดขวางบนหุ่นจริง แล้ว commit + tag ก่อนใช้บนไลน์':
        'obstacle avoidance on the real robot, then commit + tag before line use',
    'บันทึก?': 'Save?',
    'เขียน nav2_params.yaml แล้ว (git tree เป็น -dirty)\n': 'nav2_params.yaml written (git tree is now -dirty)\n',
    'ระบบกำลังทำงาน - ต้องปิดแล้วเริ่มใหม่เพื่อใช้ค่าใหม่': 'System running - restart it to apply',
    'ค่าใหม่มีผลเมื่อเริ่มระบบครั้งถัดไป': 'Applies at next system start',
    'ใส่ PIN ปัจจุบัน': 'Enter current PIN',
    'ค่าเริ่มต้น 1234': 'default 1234',
    'ใส่ PIN ใหม่': 'Enter new PIN',
    'ยืนยัน PIN ใหม่อีกครั้ง': 'Confirm new PIN',
    'ต้องเป็นตัวเลข 4-6 หลัก': 'Must be 4-6 digits',
    'PIN ไม่ตรงกัน': 'PINs do not match',
    'เปลี่ยน PIN โหมดนักพัฒนา': 'Change Developer PIN',
    'เปลี่ยน PIN ไม่สำเร็จ': 'PIN change failed',
    'เปลี่ยน PIN แล้ว': 'PIN changed',
    'ใช้ PIN ใหม่ตั้งแต่ครั้งถัดไป': 'The new PIN applies from next login',
    # Auto
    'เริ่มระบบนำทาง (เลือกแผนที่)': 'Start navigation (select map)',
    'จุดหมาย (Destination)': 'Destination',
    'เริ่มระบบนำทางก่อน': 'Start navigation first',
    ' ยังไม่มี Station\n': ' has no stations yet\n',
    '(เพิ่มได้ที่ Developer › Navigation Test)': '(add in Developer › Navigation Test)',
    'ยังไม่มีภารกิจ': 'No mission',
    'กลับ Base': 'Return to Base',
    'เลือก: ': 'Selected: ',
    'เป้าหมาย: ': 'Target: ',
    'เหลือ ': 'remaining ',
    'หยุดชั่วคราว': 'Paused',
    'กำลังไป...': 'En route...',
    'ถึงแล้ว ✓': 'Arrived ✓',
    'ยืนยันภารกิจ': 'Confirm mission',
    'แบบอัตโนมัติ?': 'autonomously?',
    'ส่งหุ่นยนต์ไปที่ ': 'Send robot to ',
    'ส่งหุ่นยนต์กลับ ': 'Send robot back to ',
    'ตรวจสอบเส้นทางว่าไม่มีคน/สิ่งกีดขวางก่อนยืนยัน (ISO 3691-4)':
        'Check the route is clear of people/obstacles before confirming (ISO 3691-4)',
    # Manual
    'Manual Override - ควบคุมด้วยรีโมท/จอยภายนอก ขับช้าและมองเส้นทางตลอดเวลา':
        'Manual Override - external remote/joystick in control; drive slowly and watch the path',
    'Controller: ACTIVE (มีคำสั่ง)': 'Controller: ACTIVE (input)',
    'เริ่มระบบ Manual': 'Start Manual',
    'QR มือถือ': 'Phone QR',
    'ปิดระบบ (Stop)': 'Shut down (Stop)',
    'ปิดระบบ': 'Shut down',
    # Dev
    'สร้างแผนที่': 'Create map',
    'ทดสอบนำทาง': 'Navigation test',
    'ตั้งค่า': 'Settings',
    'ใช้งานทั่วไป': 'General use',
    'โหมดนักพัฒนา': 'Developer',
    'อัตโนมัติ': 'Automatic',
    'ควบคุมเอง': 'Manual control',
    # Mapping
    'ใช้ Joystick (ฮาร์ดแวร์/มือถือ) ขับหุ่นยนต์ช้าๆ ให้ทั่วพื้นที่':
        'Use the joystick (hardware/phone) to drive slowly over the whole area',
    'SAVE MAP (บันทึกแผนที่)': 'SAVE MAP',
    'QR มือถือ (แผนที่ + Joystick)': 'Phone QR (map + joystick)',
    'ไม่มีข้อมูล': 'No data',
    'หยุด SLAM และทิ้งแผนที่ที่ยังไม่ได้บันทึก?': 'Stop SLAM and discard the unsaved map?',
    'กำลังบันทึก...': 'Saving...',
    'รอข้อมูลแผนที่ (/map)...': 'Waiting for map (/map)...',
    # Nav test
    'เริ่ม Navigation': 'Start Navigation',
    'แตะบนแผนที่ = ตั้งจุดหมาย, ลาก = กำหนดทิศ\n': 'Tap map = set goal, drag = set heading\n',
    'ยืนยันก่อนหุ่นยนต์ออกตัวทุกครั้ง': 'Always confirmed before the robot moves',
    'ยกเลิก Goal': 'Cancel goal',
    '＋ บันทึกตำแหน่งนี้เป็น Station': '＋ Save current pose as station',
    'ลบ Station...': 'Delete station...',
    'ลบ Station': 'Delete station',
    'QR ตั้งจุดหมายจากมือถือ': 'Phone QR: set goal',
    'Planner (อ่านอย่างเดียว - แก้ใน nav2_params.yaml)': 'Planner (read-only - edit nav2_params.yaml)',
    'เฉพาะ WARN/ERROR': 'WARN/ERROR only',
    'ตรง': 'Straight', 'สี่เหลี่ยม': 'Square', 'วงกลม': 'Circle',
    'ระยะ:': 'Dist:', 'ความเร็ว:': 'Speed:', 'ด้าน:': 'Side:', 'รัศมี:': 'Radius:', 'มุม:': 'Angle:',
    ' ม./ด้าน': ' m/side', ' ม.': ' m',
    'เดินหน้า (Move Forward)': 'Move Forward',
    'วิ่งเป็นสี่เหลี่ยม (Square Path)': 'Square Path',
    'วิ่งวงกลม (Circle Path)': 'Circle Path',
    'ยกเลิก (Cancel)': 'Cancel',
    'ตรวจพื้นที่ด้านหน้าก่อนกด (ISO 3691-4)': 'check the area ahead first (ISO 3691-4)',
    'ด้านละ ': 'Side ',
    ' ใช้ความเร็วจากแท็บ "ตรง" ': ', speed from "Straight" tab, ',
    'หมุนทวนเข็ม 90° ต่อมุม - ตรวจพื้นที่โดยรอบก่อนกด (ISO 3691-4)':
        '90° CCW per corner - check surroundings first (ISO 3691-4)',
    'ω คงที่ ': 'Fixed ω ',
    ' - ตรวจพื้นที่โดยรอบก่อนกด (ISO 3691-4)': ' - check surroundings first (ISO 3691-4)',
    'ยืนยัน 2D Goal': 'Confirm 2D Goal',
    'ส่งหุ่นยนต์ไป ': 'Send robot to ',
    'ตรวจสอบเส้นทางว่าปลอดภัยก่อนยืนยัน (ISO 3691-4)': 'Check the route is safe before confirming (ISO 3691-4)',
    'ยังไม่พร้อม': 'Not ready',
    'เริ่ม Navigation ก่อนตั้งจุดหมาย': 'Start Navigation before setting a goal',
    'เริ่ม Navigation (เลือกแผนที่) ก่อน': 'Start Navigation (select a map) first',
    'ไม่พบตำแหน่ง': 'No pose',
    'ยังไม่มี TF map → base_footprint (AMCL)': 'No TF map → base_footprint yet (AMCL)',
    'ตั้งชื่อ Station': 'Station name',
    'บันทึกแล้ว': 'Saved',
    'ไม่มี Station': 'No stations',
    'แผนที่นี้ยังไม่มี Station': 'This map has no stations',
    'ลบ ': 'Delete ',
    # Settings
    'เปลี่ยนภาษาที่หน้า Home · ความสว่างปรับที่ OS': 'Switch on Home page · brightness: set in OS',
    'ปรับเทียบ Localization / Wheel (Calibrate)': 'Calibrate localization / wheels',
    'อ่านอย่างเดียว - ค่าที่เกี่ยวกับ Safety ต้องแก้ผ่านไฟล์ + commit + tag ใหม่':
        'Read-only - safety-related values change via file + commit + new tag',
    'อยู่ใน firmware stm/ - แก้ + commit ก่อน flash': 'in firmware stm/ - edit + commit before flashing',
    # Dialogs / QR
    'ตั้งชื่อแผนที่': 'Map name',
    'ใช้ตัวอักษร ตัวเลข - และ _ เท่านั้น': 'Letters, digits, - and _ only',
    'กรุณาใส่ชื่อที่ถูกต้อง': 'Please enter a valid name',
    'บันทึก': 'Save', 'ยกเลิก': 'Cancel', 'เลือก': 'Select', 'ปิด': 'Close',
    '   (ล่าสุด)': '   (latest)',
    'เลือกแผนที่สำหรับนำทาง': 'Select map for navigation',
    'เลือกแผนที่นี้': 'Use this map',
    'แก้ไขล่าสุด ': 'Modified ',
    ' จุด': '',
    ' ม./px': ' m/px',
    'ขนาด ': 'Size ',
    'ไม่พบแผนที่': 'No map found',
    'ยังไม่มีไฟล์แผนที่ที่บันทึกไว้ กรุณาสร้างแผนที่ก่อน (โหมด Mapping)':
        'No saved map yet - create one first (Mapping mode)',
    'ชื่อซ้ำ': 'Name exists',
    'อยู่แล้ว ต้องการเขียนทับหรือไม่?': 'already exists. Overwrite?',
    'มีแผนที่ชื่อ ': 'Map ',
    'บันทึกสำเร็จ': 'Saved',
    'บันทึกแผนที่แล้ว: ': 'Map saved: ',
    'บันทึกไม่สำเร็จ': 'Save failed',
    'ไม่สามารถบันทึกแผนที่ได้ ลองใหม่อีกครั้ง': 'Could not save the map, please retry',
    'ปิดโปรแกรม': 'Quit',
    'ระบบกำลังทำงานอยู่ ต้องการหยุดและปิดโปรแกรมหรือไม่?': 'System is running. Stop it and quit?',
    'ยังไม่ได้เริ่มระบบ': 'System not started',
    'กรุณาเริ่มระบบ (Mapping / Navigation / Manual) ': 'Start the system (Mapping / Navigation / Manual) ',
    'เพื่อเชื่อมต่อ STM32/odometry ก่อน แล้วค่อยเปิดหน้าปรับเทียบ':
        'to connect STM32/odometry before opening calibration',
    'ยืนยันการเคลื่อนที่': 'Confirm motion',
    'ให้หุ่นยนต์เดินหน้าตรง ': 'Drive robot straight ',
    'ให้หุ่นยนต์วิ่งเป็นสี่เหลี่ยมด้านละ ': 'Drive robot in a square, side ',
    'ให้หุ่นยนต์วิ่งเป็นวงกลมรัศมี ': 'Drive robot in a circle, radius ',
    ' เมตร ': ' m ',
    'ด้วยความเร็วเชิงมุม ': 'at angular speed ',
    'ด้วยความเร็ว ': 'at ',
    ' แล้วกลับมาจุดเดิมใช่หรือไม่?': ' and return to start?',
    ' ใช่หรือไม่?': '?',
    ' องศา ': ' deg ',
    'ตรวจสอบว่าด้านหน้าหุ่นยนต์ไม่มีคน/สิ่งกีดขวางก่อนยืนยัน':
        'Make sure nobody/nothing is in front of the robot before confirming',
    'ตรวจสอบว่าโดยรอบหุ่นยนต์ไม่มีคน/สิ่งกีดขวางตลอดเส้นทางก่อนยืนยัน':
        'Make sure the whole path around the robot is clear before confirming',
    'หยุดกลางทาง': 'Stopped midway',
    'ใช้เวลานานเกินคาดในช่วงเดินหน้า (odom นิ่ง/หุ่นติดขัด) - หยุดคำสั่งแล้ว':
        'Straight leg timed out (odom stalled/robot stuck) - command stopped',
    'ใช้เวลานานเกินคาดในช่วงหมุน (หุ่นติดขัด) - หยุดคำสั่งแล้ว':
        'Turn timed out (robot stuck) - command stopped',
    'ใช้เวลานานเกินคาด (odom นิ่ง/หุ่นติดขัด) - หยุดคำสั่งแล้ว':
        'Timed out (odom stalled/robot stuck) - command stopped',
    'ไม่สำเร็จ': 'Failed',
    'ไม่พบข้อมูล ': 'No data on ',
    ' - ยกเลิกคำสั่ง': ' - command cancelled',
    'วิ่งวงกลมเสร็จสิ้น': 'Circle complete',
    'จำนวนรอบที่ล้อหมุนไป:': 'Wheel revolutions:',
    'ล้อซ้าย: ': 'Left: ', 'ล้อขวา: ': 'Right: ', ' รอบ': ' rev',
    'Monitor บนมือถือ': 'Phone monitor',
    'สแกนเพื่อดูแผนที่ + ควบคุมขับจากมือถือ': 'Scan to view the map + drive from your phone',
    'ต่อ WiFi วงเดียวกับ Jetson นี้ก่อนสแกน (ไม่ต้องใช้อินเทอร์เน็ต)':
        'Join the same WiFi as this Jetson before scanning (no internet needed)',
    'แผนที่ + ควบคุม (แนะนำ)': 'Map + control (recommended)',
    'Monitor + ควบคุม เบา (rosboard เท่านั้น)': 'Lightweight monitor + control (rosboard only)',
    'หน้ารวม (แผนที่+ควบคุม) ยังไม่พร้อมใช้งาน\n': 'Combined page (map+control) not available\n',
    '(ต้องมีทั้ง rosboard และ ': '(needs both rosboard and ',
    'รหัสเข้าใช้งาน rosboard (PIN)': 'rosboard access PIN',
    'รหัสเข้าใช้งาน (PIN)': 'Access PIN',
    'ตั้งจุดหมายจากมือถือ': 'Set goal from phone',
    'สแกนเพื่อเลือกจุดหมาย/วาดเส้นทางให้หุ่นยนต์วิ่งตาม': 'Scan to pick a goal / draw a path for the robot',
    'หุ่นยนต์จะเคลื่อนที่แบบอัตโนมัติทันทีที่ยืนยันบนหน้าจอนั้น - ':
        'The robot moves autonomously as soon as you confirm on that page - ',
    'ตรวจสอบพื้นที่ให้ปลอดภัยก่อนเสมอ (ISO 3691-4)': 'always check the area is safe first (ISO 3691-4)',
    'ตั้งจุดหมาย / ลากเส้นทาง (แนะนำ)': 'Set goal / draw path (recommended)',
    'แตะแผนที่เพื่อส่งหุ่นยนต์ไปจุดนั้น หรือลากเพื่อวาดเส้นทาง': 'Tap the map to send the robot there, or drag to draw a path',
    'หน้าตั้งจุดหมายยังไม่พร้อมใช้งาน (rosboard ไม่ทำงาน)': 'Goal page unavailable (rosboard not running)',
    'Monitor แผนที่ (rosboard)': 'Map monitor (rosboard)',
    'แผนที่แบบ Foxglove (Lichtblick) ด้านบน + จอย': 'Foxglove-style map (Lichtblick) on top + joy',
    'สติ๊กควบคุมขับ (rosboard 🕹) ด้านล่าง - ': 'stick drive (rosboard 🕹) below - ',
    'โหลดครั้งแรกอาจช้าสักครู่ (~170MB ผ่าน LAN)': 'first load may be slow (~170MB over LAN)',
}
_EN_FRAGMENTS = sorted(EN.items(), key=lambda kv: -len(kv[0]))
_THAI_RE = re.compile('[฀-๿]')


def T(text):
    if _LANG['cur'] != 'en' or not isinstance(text, str) or not _THAI_RE.search(text):
        return text
    if text in EN:
        return EN[text]
    for th, en in _EN_FRAGMENTS:
        if th in text:
            text = text.replace(th, en)
    return text


def load_lang():
    try:
        with open(LANG_FILE) as f:
            return 'en' if f.read().strip() == 'en' else 'th'
    except OSError:
        return 'th'


class _TranslatedMessagebox:
    """messagebox with T() applied to title/message."""
    def __getattr__(self, name):
        fn = getattr(_tk_messagebox, name)

        def wrapped(title=None, message=None, **kw):
            return fn(T(title), T(message), **kw)
        return wrapped


messagebox = _TranslatedMessagebox()


def git_describe():
    try:
        return subprocess.run(
            ['git', '-C', REPO_DIR, 'describe', '--always', '--dirty', '--tags'],
            capture_output=True, text=True, timeout=3).stdout.strip() or '-'
    except Exception:
        return '-'


# ======================================================================
class RosBridge:
    """Background rclpy node: latest-value cache for the UI + goal/cancel/
    zero-velocity publishers. Every callback only stores data; the Tk
    thread polls it. If rclpy can't start (no ROS env) the UI still works
    for launching, just without live data."""

    def __init__(self):
        self.ok = False
        self.odom = None        # (v, w, t)
        self.cmd = None         # (vx, wz, t)
        self.map_msg = None
        self.map_seq = 0
        self.watchdog = {}      # name -> (bool, t)
        self.hook_state = None  # latest /hook/state text (hook_bridge.py)
        self.rates = {'scan': collections.deque(maxlen=60),
                      'odom': collections.deque(maxlen=120),
                      'imu': collections.deque(maxlen=400)}
        self.log = collections.deque(maxlen=300)
        self.log_seq = 0
        try:
            self._start()
            self.ok = True
        except Exception as e:  # noqa: BLE001
            print(f'amr_control_panel: ROS bridge disabled: {e}')

    def _start(self):
        import rclpy
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.qos import (DurabilityPolicy, QoSProfile, ReliabilityPolicy,
                               qos_profile_sensor_data)
        from geometry_msgs.msg import PoseStamped, Twist
        from nav_msgs.msg import Odometry, OccupancyGrid
        from rcl_interfaces.msg import Log
        from sensor_msgs.msg import Imu, LaserScan
        from std_msgs.msg import Bool, Empty, String
        from std_srvs.srv import SetBool
        import tf2_ros

        self._PoseStamped, self._Twist, self._Empty = PoseStamped, Twist, Empty
        rclpy.init()
        self._rclpy = rclpy
        node = rclpy.create_node('amr_control_panel')
        self.node = node

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, node)

        def on_odom(m):
            now = time.monotonic()
            self.odom = (m.twist.twist.linear.x, m.twist.twist.angular.z, now)
            self.rates['odom'].append(now)

        def on_cmd(m):
            self.cmd = (m.linear.x, m.angular.z, time.monotonic())

        def on_map(m):
            self.map_msg = m
            self.map_seq += 1

        def on_scan(_m):
            self.rates['scan'].append(time.monotonic())

        def on_log(m):
            lvl = {10: 'D', 20: 'I', 30: 'W', 40: 'E', 50: 'F'}.get(m.level, '?')
            self.log.append((lvl, f'[{lvl}] {m.name}: {m.msg}'))
            self.log_seq += 1

        def wd(name):
            def cb(m):
                self.watchdog[name] = (bool(m.data), time.monotonic())
            return cb

        map_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        node.create_subscription(Odometry, '/odometry/filtered', on_odom, 10)
        node.create_subscription(Twist, '/cmd_vel', on_cmd, 10)
        node.create_subscription(OccupancyGrid, '/map', on_map, map_qos)
        node.create_subscription(LaserScan, '/scan', on_scan, qos_profile_sensor_data)
        # EKF takes yaw rate from the gyro only, so /imu/data matters in every
        # mode (imu_watchdog_node only runs with Navigation).
        node.create_subscription(
            Imu, '/imu/data', lambda _m: self.rates['imu'].append(time.monotonic()),
            qos_profile_sensor_data)
        node.create_subscription(Log, '/rosout', on_log, 50)
        node.create_subscription(Bool, '/depth_watchdog/ok', wd('depth'), 10)
        node.create_subscription(Bool, '/imu_watchdog/ok', wd('imu'), 10)

        self.goal_pub = node.create_publisher(PoseStamped, '/rosboard_nav_goal', 10)
        # station missions: nav_goal_executor parks inside the station box
        self.station_pub = node.create_publisher(PoseStamped, '/rosboard_nav_station', 10)
        self.cancel_pub = node.create_publisher(Empty, '/rosboard_nav_cancel', 10)
        self.cmd_pub = node.create_publisher(Twist, '/cmd_vel', 10)

        # Phone-side mapping controls (rosboard -> /rosboard_mapping_cmd,
        # validated JSON {action, name}); the panel owns SLAM pause and map
        # saving and reports back on /amr_mapping_state for the phone UI.
        self._String = String
        self.mapping_cmds = collections.deque(maxlen=10)
        node.create_subscription(
            String, '/rosboard_mapping_cmd',
            lambda m: self.mapping_cmds.append(m.data), 10)
        self.mapping_state_pub = node.create_publisher(String, '/amr_mapping_state', 10)
        # Stations of the map Navigation is running on, for the phone
        # goal_picker page (pins + "send to station"). JSON, sent at 1 Hz.
        self.stations_pub = node.create_publisher(String, '/amr_stations', 10)
        # amr_slam scan_gate_node: SLAM only receives scans while open.
        self._SetBool = SetBool
        self.gate_client = node.create_client(SetBool, '/scan_gate/set_enabled')
        self.gate_open = None  # latest /scan_gate/enabled, None = no gate seen

        def on_gate(m):
            self.gate_open = bool(m.data)
        gate_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                              reliability=ReliabilityPolicy.RELIABLE)
        node.create_subscription(Bool, '/scan_gate/enabled', on_gate, gate_qos)

        # hook_bridge.py publishes only on change (latched), so liveness is
        # checked with hook_alive() rather than message age.
        def on_hook(m):
            self.hook_state = m.data
        node.create_subscription(String, '/hook/state', on_hook, gate_qos)

        self._executor = SingleThreadedExecutor()
        self._executor.add_node(node)
        threading.Thread(target=self._spin, daemon=True).start()

    def _spin(self):
        try:
            self._executor.spin()
        except Exception:  # noqa: BLE001 - shutdown races
            pass

    # ----- queries -----
    def hook_alive(self):
        """True while hook_bridge is running (the latched value alone would
        keep showing the last state after the bridge died)."""
        return self.ok and self.node.count_publishers('/hook/state') > 0

    def pose(self):
        """(x, y, yaw) of base_footprint in map, or None."""
        if not self.ok:
            return None
        try:
            from rclpy.time import Time
            t = self.tf_buffer.lookup_transform('map', 'base_footprint', Time())
        except Exception:  # noqa: BLE001
            return None
        tr, q = t.transform.translation, t.transform.rotation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        return tr.x, tr.y, yaw

    def rate(self, name):
        now = time.monotonic()
        return sum(1 for t in self.rates[name] if now - t < 2.0) / 2.0

    def fresh(self, item, age=0.5):
        return item is not None and time.monotonic() - item[-1] < age

    # ----- commands -----
    def send_goal(self, x, y, theta, station=False):
        if not self.ok:
            return False
        m = self._PoseStamped()
        m.header.frame_id = 'map'
        m.header.stamp = self.node.get_clock().now().to_msg()
        m.pose.position.x, m.pose.position.y = float(x), float(y)
        m.pose.orientation.z = math.sin(theta / 2.0)
        m.pose.orientation.w = math.cos(theta / 2.0)
        (self.station_pub if station else self.goal_pub).publish(m)
        return True

    def cancel_goal(self):
        if self.ok:
            self.cancel_pub.publish(self._Empty())

    def software_stop(self):
        """Cancel Nav2 task + burst of zero /cmd_vel (non-safety-rated)."""
        if not self.ok:
            return

        def burst():
            self.cancel_pub.publish(self._Empty())
            z = self._Twist()
            for _ in range(10):
                self.cmd_pub.publish(z)
                time.sleep(0.03)
        threading.Thread(target=burst, daemon=True).start()

    def publish_stations(self, map_name, stations):
        if self.ok:
            self.stations_pub.publish(self._String(data=json.dumps(
                {'map': map_name or '',
                 'stations': [dict(st, color=station_color(i))
                              for i, st in enumerate(stations)]})))

    def publish_mapping_state(self, state):
        if self.ok:
            self.mapping_state_pub.publish(self._String(data=json.dumps(state)))

    def wait_scan_gate(self, timeout):
        return self.ok and self.gate_client.wait_for_service(timeout_sec=timeout)

    def set_scan_gate(self, enabled, timeout=5.0):
        """Open/close the SLAM scan gate (idempotent). Blocking - call from
        a worker thread. Returns True on success."""
        if not self.wait_scan_gate(timeout):
            return False
        fut = self.gate_client.call_async(self._SetBool.Request(data=bool(enabled)))
        t0 = time.monotonic()
        while not fut.done() and time.monotonic() - t0 < timeout:
            time.sleep(0.05)
        return fut.done() and fut.result() is not None and fut.result().success

    def shutdown(self):
        if self.ok:
            try:
                self._executor.shutdown(timeout_sec=1.0)
                self.node.destroy_node()
                self._rclpy.shutdown()
            except Exception:  # noqa: BLE001
                pass


# ======================================================================
class MapPlaceholder(tk.Frame):
    """Stands in for MapView: the panel no longer draws the map (operator
    asked to watch it on the phone goal_picker page instead - also frees
    the CPU the 4 Hz map/overlay redraw took while Nav2 was missing its
    control rate). Same set_map/set_overlay API as MapView, as no-ops, so
    the pages and _tick need no other change. Stations are still chosen
    with the buttons; tap-to-goal is on the phone."""

    def __init__(self, parent, on_tap=None, **kw):
        super().__init__(parent, bg='#1c222a', highlightthickness=1,
                         highlightbackground=BORDER, **kw)
        tk.Label(self, text=T('แผนที่ดูบนมือถือ\n(ปุ่ม QR มือถือ → หน้า Goal Picker)'),
                 bg='#1c222a', fg=TEXT_MUTED, font=('Sans', -px(16)),
                 justify='center').place(relx=0.5, rely=0.5, anchor='center')

    def set_map(self, *_a, **_k):
        pass

    def set_overlay(self, *_a, **_k):
        pass


# ======================================================================
class MapView(tk.Canvas):
    """Occupancy grid + robot + markers. Optional tap (and drag for
    heading) callback in map coordinates: on_tap(x, y, theta)."""

    def __init__(self, parent, on_tap=None, **kw):
        super().__init__(parent, bg='#1c222a',
                         highlightthickness=1, highlightbackground=BORDER, **kw)
        self._photo = None
        self._seq = -1
        self._info = None   # (res, ox, oy, w, h)
        self._xf = None     # (scale, off_x, off_y)
        self._on_tap = on_tap
        self._press = None
        self._markers = []
        self._goal = None
        self.create_text(px(10), px(10), anchor='nw', text=T('รอข้อมูลแผนที่ (/map)...'),
                         fill=TEXT_MUTED, tags='placeholder', font=('Sans', -px(16)))
        self.bind('<Configure>', lambda _e: self._rerender())
        if on_tap:
            self.bind('<ButtonPress-1>', self._on_press)
            self.bind('<B1-Motion>', self._on_drag)
            self.bind('<ButtonRelease-1>', self._on_release)

    def set_map(self, msg, seq):
        if msg is None or seq == self._seq:
            return
        self._seq = seq
        info = msg.info
        w, h = info.width, info.height
        if w == 0 or h == 0:
            return
        arr = np.asarray(msg.data, dtype=np.int16).reshape(h, w)
        rgb = np.empty((h, w, 3), dtype=np.uint8)
        rgb[:] = MAP_UNKNOWN
        rgb[(arr >= 0) & (arr < 50)] = MAP_FREE
        rgb[arr >= 50] = MAP_OCC
        self._img = Image.fromarray(np.flipud(rgb), 'RGB')
        self._info = (info.resolution, info.origin.position.x,
                      info.origin.position.y, w, h)
        self._rerender()

    def _rerender(self):
        if self._info is None:
            return
        cw, ch = max(self.winfo_width(), 50), max(self.winfo_height(), 50)
        _, _, _, w, h = self._info
        s = min(cw / w, ch / h)
        dw, dh = max(int(w * s), 1), max(int(h * s), 1)
        self._xf = (s, (cw - dw) // 2, (ch - dh) // 2)
        self._photo = ImageTk.PhotoImage(self._img.resize((dw, dh), Image.NEAREST))
        self.delete('map', 'placeholder')
        self.create_image(self._xf[1], self._xf[2], anchor='nw',
                          image=self._photo, tags='map')
        self.tag_lower('map')

    def w2c(self, x, y):
        res, ox, oy, _, h = self._info
        s, offx, offy = self._xf
        return offx + (x - ox) / res * s, offy + (h - (y - oy) / res) * s

    def c2w(self, cx, cy):
        res, ox, oy, _, h = self._info
        s, offx, offy = self._xf
        return ox + (cx - offx) / s * res, oy + (h - (cy - offy) / s) * res

    def _arrow(self, x, y, yaw, color, tag, r=9):
        r = px(r)
        cx, cy = self.w2c(x, y)
        self.create_oval(cx - r, cy - r, cx + r, cy + r, fill=color,
                         outline='white', width=px(2), tags=tag)
        self.create_line(cx, cy, cx + 2.2 * r * math.cos(yaw),
                         cy - 2.2 * r * math.sin(yaw), fill='white',
                         width=px(3), arrow='last', tags=tag)

    def set_overlay(self, pose, markers=(), goal=None):
        self.delete('overlay')
        if self._xf is None:
            return
        if goal is not None:
            self._arrow(goal[0], goal[1], goal[2], AMBER, 'overlay', r=7)
        if pose is not None:
            self._arrow(pose[0], pose[1], pose[2], GREEN, 'overlay')
        # Station pins last, so the robot arrow never hides one (a station
        # saved where the robot is parked used to vanish under it).
        for m in markers:
            self._pin(m, m.get('color', BLUE), m.get('selected'))
        if self._press:
            x0, y0, x1, y1 = self._press
            self.create_line(x0, y0, x1, y1, fill=AMBER, width=px(3),
                             arrow='last', tags='overlay')

    def _pin(self, m, col, selected=False):
        """Map pin: the tip sits exactly on the station (x, y), a short
        line from the tip shows the saved heading, name above the head.
        Selected: bigger head with an amber ring (colour stays the station's)."""
        cx, cy = self.w2c(m['x'], m['y'])
        r, h = px(8), px(22)  # head radius, tip-to-head-centre height
        if selected:
            r = px(11)
            self.create_oval(cx - r - px(3), cy - h - r - px(3), cx + r + px(3), cy - h + r + px(3),
                             fill='', outline=AMBER, width=px(3), tags='overlay')
        hx, hy = self.w2c(m['x'] + math.cos(m.get('theta', 0.0)),
                          m['y'] + math.sin(m.get('theta', 0.0)))
        d = math.hypot(hx - cx, hy - cy) or 1.0
        self.create_line(cx, cy, cx + (hx - cx) / d * px(18), cy + (hy - cy) / d * px(18),
                         fill=col, width=px(3), arrow='last', tags='overlay')
        self.create_polygon(cx, cy, cx - r * 0.8, cy - h + r * 0.4, cx + r * 0.8, cy - h + r * 0.4,
                            fill=col, outline='white', tags='overlay')
        self.create_oval(cx - r, cy - h - r, cx + r, cy - h + r, fill=col,
                         outline='white', width=px(2), tags='overlay')
        self.create_oval(cx - r * 0.35, cy - h - r * 0.35, cx + r * 0.35, cy - h + r * 0.35,
                         fill='white', outline='', tags='overlay')
        self.create_text(cx, cy - h - r - px(3), anchor='s', text=m['name'],
                         fill=TEXT_MAIN, font=('Sans', -px(13), 'bold'), tags='overlay')

    def _on_press(self, e):
        if self._xf is not None:
            self._press = [e.x, e.y, e.x, e.y]

    def _on_drag(self, e):
        if self._press:
            self._press[2:] = [e.x, e.y]

    def _on_release(self, e):
        if not self._press:
            return
        x0, y0, _, _ = self._press
        self._press = None
        dx, dy = e.x - x0, -(e.y - y0)
        theta = math.atan2(dy, dx) if math.hypot(dx, dy) > px(15) else 0.0
        x, y = self.c2w(x0, y0)
        self._on_tap(x, y, theta)


# ======================================================================
class AmrControlPanel(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title('AMR Control')
        self.configure(bg=BG)
        # Same icon as the desktop launcher (icons/ next to scripts/ in src;
        # this script is symlinked into install/, hence realpath).
        icon = os.path.join(os.path.dirname(os.path.realpath(__file__)),
                            '..', 'icons', 'amr_control_panel.png')
        try:
            self._icon = tk.PhotoImage(file=icon)
            self.iconphoto(True, self._icon)
        except tk.TclError:
            pass
        # Full-screen kiosk on the onboard 7" touchscreen (1920x1080); the
        # 1024x600 design is scaled by px() to whatever the screen is.
        # AMR_PANEL_WINDOWED=1 keeps a normal window (desk testing).
        sw, sh = self.winfo_screenwidth(), self.winfo_screenheight()
        try:
            _SCALE['v'] = float(os.environ['AMR_PANEL_SCALE'])
        except (KeyError, ValueError):
            _SCALE['v'] = min(sw / 1024, sh / 600)
        if os.environ.get('AMR_PANEL_WINDOWED') == '1':
            self.geometry(f'{px(1024)}x{px(600)}')
        else:
            self.geometry(f'{sw}x{sh}+0+0')
            self.attributes('-fullscreen', True)
        self.resizable(False, False)

        self._proc = None
        self._mode = None  # 'mapping' | 'navigation' | 'manual'
        self._nav_map_name = None
        self._status_poll_job = None
        self._rosboard_proc = None
        self._lichtblick_proc = None
        self._mobile_proc = None
        self._qr_photos = []  # keep references so Tk doesn't GC the images

        # "Move forward" runs as its own OS process (move_forward_node.py),
        # not a thread here, so it never shares the Tkinter mainloop's GIL -
        # see move_forward_node.py's module docstring for why that matters.
        self._move_proc = None
        self._move_poll_job = None
        self._move_kind = None  # 'forward' | 'square' | 'circle' - which button started _move_proc

        self._calib_panel = None  # open amr_calib_panel.CalibPanel Toplevel, if any

        # Auto-mode mission state
        self._stations = []
        self._selected_station = None
        self._stations_pub_t = 0.0
        self._mission = None  # {'station', 'state': active|paused|arrived, 'd0'}
        self._nav_goal = None  # last goal sent from the Nav-test map (x, y, th)

        # Mapping scan state. SLAM starts paused; the operator starts/pauses
        # scanning from the phone (or the panel) - see _mapping_*.
        self._slam_ready = False
        self._scanning = False
        self._map_saving = False
        self._mapping_msg = ''
        self._mapping_state_t = 0.0

        self._dev_unlocked = False
        self._pages = {}
        self._page = None
        self._start_buttons = []  # every "start bringup" button, all pages

        self.ros = RosBridge()
        _LANG['cur'] = load_lang()

        self._build_fonts()
        self._build_ui()
        self._set_status('idle', 'พร้อมใช้งาน')
        self._show_page('home')
        self._refresh_mapping_ui()
        self._i18n_walk()
        self.protocol('WM_DELETE_WINDOW', self._on_close)
        self.after(300, self._tick)

    def _build_fonts(self):
        self.f_title = tkfont.Font(family='Sans', size=-px(24), weight='bold')
        self.f_huge = tkfont.Font(family='Sans', size=-px(27), weight='bold')
        self.f_button = tkfont.Font(family='Sans', size=-px(19), weight='bold')
        self.f_button_sub = tkfont.Font(family='Sans', size=-px(16), weight='bold')
        self.f_status = tkfont.Font(family='Sans', size=-px(16))
        self.f_small = tkfont.Font(family='Sans', size=-px(13))
        self.f_mono = tkfont.Font(family='Monospace', size=-px(12))

    # ---------- widget helpers ----------
    def _btn(self, parent, text, command, kind='neutral', font=None,
             padx=14, pady=12, state='normal', **kw):
        # pady=px(12) + 12pt bold font keeps every touch target >= 48 px tall.
        bg, active, fg = BTN_KINDS[kind]
        if not isinstance(padx, tuple) and padx == 14:
            padx = px(14)
        if not isinstance(pady, tuple) and pady == 12:
            pady = px(12)
        b = tk.Button(parent, text=text, command=command, font=font or self.f_button_sub,
                      bg=bg, fg=fg, activebackground=active, activeforeground=fg,
                      disabledforeground=DISABLED_FG, relief='flat', bd=0,
                      highlightthickness=0, padx=padx, pady=pady,
                      cursor='hand2', state=state, **kw)
        b._kind = kind
        return b

    def _set_btn_state(self, b, enabled):
        bg, _, fg = BTN_KINDS[b._kind]
        b.config(state=('normal' if enabled else 'disabled'),
                 bg=(bg if enabled else DISABLED_BG))

    def _card(self, parent, **kw):
        return tk.Frame(parent, bg=SURFACE, highlightthickness=1,
                        highlightbackground=BORDER, **kw)

    def _label(self, parent, text='', font=None, fg=TEXT_MAIN, bg=SURFACE, **kw):
        return tk.Label(parent, text=text, font=font or self.f_status, fg=fg,
                        bg=bg, **kw)

    def _section_title(self, parent, text, bg=SURFACE):
        return tk.Label(parent, text=text, font=self.f_small, fg=TEXT_MUTED,
                        bg=bg, anchor='w')

    def _make_mode_card(self, parent, title_th, title_en, kind, command,
                        desc=''):
        b = self._btn(parent, '', command, kind=kind, font=self.f_huge,
                      padx=px(20), pady=px(40), justify='left', anchor='w')
        b._i18n_card = (title_th, title_en, desc)
        # Card width depends on how many share the row (2 on Home, 4 on
        # Developer): re-fit the text whenever the card is resized.
        b.bind('<Configure>', lambda _e, b=b: self._fit_card(b), add='+')
        self._card_text(b)
        return b

    def _card_text(self, b):
        th, en, desc = b._i18n_card
        head = en if _LANG['cur'] == 'en' else f'{th}\n{en}'
        b.config(text=head + (f'\n\n{T(desc)}' if desc else ''))
        self._fit_card(b)

    def _fit_card(self, b):
        """One font for every card in the same row: the largest whose
        title lines fit the narrowest card (never cut mid-word);
        wraplength follows each card's width."""
        cards = [w for w in b.master.winfo_children()
                 if getattr(w, '_i18n_card', None)]
        if any(c.winfo_width() <= 1 for c in cards):
            return
        avail = min(c.winfo_width() for c in cards) - 2 * px(20) - px(8)
        lines = [ln for c in cards for ln in c.cget('text').split('\n')[:2] if ln]
        for f in (self.f_huge, self.f_title, self.f_button):
            if all(f.measure(ln) <= avail for ln in lines):
                break
        for c in cards:
            wrap = c.winfo_width() - 2 * px(20) - px(8)
            if str(c.cget('font')) != str(f) or int(str(c.cget('wraplength'))) != wrap:
                c.config(font=f, wraplength=wrap)

    # ---------- language ----------
    def _i18n_walk(self, root=None):
        """Apply T() to every widget's text. A widget's Thai source text is
        remembered, so toggling back restores it; text changed by code
        since the last walk is taken as the new source."""
        stack = [root or self]
        while stack:
            w = stack.pop()
            stack.extend(w.winfo_children())
            if hasattr(w, '_i18n_card'):
                if w._i18n_card:
                    self._card_text(w)
                continue
            if isinstance(w, MapView):
                w.itemconfig('placeholder', text=T('รอข้อมูลแผนที่ (/map)...'))
                continue
            if isinstance(w, ttk.Notebook):
                store = w.__dict__.setdefault('_i18n_tabs', {})
                for tab in w.tabs():
                    cur = w.tab(tab, 'text')
                    src, shown = store.get(tab, (None, None))
                    if cur != shown:
                        src = cur
                    shown = T(src)
                    store[tab] = (src, shown)
                    if cur != shown:
                        w.tab(tab, text=shown)
                continue
            try:
                cur = w.cget('text')
            except tk.TclError:
                continue
            if not isinstance(cur, str) or not cur:
                continue
            src, shown = getattr(w, '_i18n', (None, None))
            if cur != shown:
                src = cur
            shown = T(src)
            w._i18n = (src, shown)
            if cur != shown:
                w.config(text=shown)

    def _toggle_lang(self):
        _LANG['cur'] = 'en' if _LANG['cur'] == 'th' else 'th'
        try:
            with open(LANG_FILE, 'w') as f:
                f.write(_LANG['cur'])
        except OSError:
            pass
        self._i18n_walk()

    def _set_card_enabled(self, card, enabled):
        self._set_btn_state(card, enabled)

    def _stepper(self, parent, label, value_text, on_minus, on_plus, width=8,
                 big=False):
        # big: >= 48 px (design units) touch targets, e.g. the Parameters tab
        btn_font, bpx, bpy = ((self.f_huge, px(26), px(10)) if big
                              else (self.f_button, px(16), px(8)))
        row = tk.Frame(parent, bg=SURFACE)
        if label:
            self._label(row, label, width=7, anchor='w').pack(side='left')
        minus = self._btn(row, '−', on_minus, font=btn_font, padx=bpx, pady=bpy)
        minus.pack(side='left')
        val = self._label(row, value_text, width=width, anchor='center',
                          font=self.f_button if big else None)
        val.pack(side='left', padx=px(4))
        plus = self._btn(row, '+', on_plus, font=btn_font, padx=bpx, pady=bpy)
        plus.pack(side='left')
        return row, minus, val, plus

    # ---------- layout ----------
    def _build_ui(self):
        self._build_header()
        self._body = tk.Frame(self, bg=BG)
        self._body.pack(fill='both', expand=True, padx=px(12), pady=(px(6), px(10)))
        for name, builder in (
                ('home', self._build_home), ('general', self._build_general),
                ('auto', self._build_auto), ('manual', self._build_manual),
                ('dev', self._build_dev), ('mapping', self._build_mapping),
                ('navtest', self._build_navtest), ('settings', self._build_settings)):
            f = tk.Frame(self._body, bg=BG)
            builder(f)
            self._pages[name] = f
        self._set_running_ui(False)

    PAGE_TITLES = {
        'home': 'AMR Control', 'general': 'General Operations',
        'auto': 'Auto Mode', 'manual': 'Manual Mode',
        'dev': 'Developer Mode', 'mapping': 'Mapping',
        'navtest': 'Navigation Test', 'settings': 'Settings',
    }
    PAGE_PARENT = {
        'general': 'home', 'dev': 'home', 'auto': 'general', 'manual': 'general',
        'mapping': 'dev', 'navtest': 'dev', 'settings': 'dev',
    }
    DEV_PAGES = ('dev', 'mapping', 'navtest', 'settings')

    def _build_header(self):
        h = tk.Frame(self, bg=SURFACE, height=px(64))
        h.pack(fill='x')
        h.pack_propagate(False)

        self.btn_back = self._btn(h, '‹  กลับ', self._go_back, font=self.f_button,
                                  padx=px(14), pady=px(8))
        self.btn_back.pack(side='left', padx=(px(10), px(6)), pady=px(8))
        self.page_title = tk.Label(h, text='AMR Control', font=self.f_title,
                                   bg=SURFACE, fg=TEXT_MAIN)
        self.page_title.pack(side='left', padx=px(6))

        # Software STOP - always visible, on every page.
        self.btn_estop = self._btn(h, '■  STOP', self._on_software_stop,
                                   kind='danger', font=self.f_button, padx=px(18), pady=px(8))
        self.btn_estop.pack(side='right', padx=(px(6), px(10)), pady=px(8))

        pills = tk.Frame(h, bg=SURFACE)
        pills.pack(side='right', padx=px(6))

        def pill(text):
            lbl = tk.Label(pills, text=text, font=self.f_small, bg=SURFACE_2,
                           fg=TEXT_MAIN, padx=px(10), pady=px(6))
            lbl.pack(side='left', padx=px(3))
            return lbl
        self.pill_safety = pill('Safety —')
        self.pill_net = pill('Net —')
        self.pill_batt = pill('Batt —')
        self.pill_hook = pill('Hook —')

        st = tk.Frame(h, bg=SURFACE)
        st.pack(side='right', padx=px(10))
        row = tk.Frame(st, bg=SURFACE)
        row.pack(anchor='e')
        self.status_dot = tk.Canvas(row, width=px(14), height=px(14), bg=SURFACE,
                                    highlightthickness=0)
        self.status_dot.pack(side='left', padx=(px(0), px(6)))
        self._dot_id = self.status_dot.create_oval(px(2), px(2), px(12), px(12), fill='#9e9e9e', outline='')
        self.status_label = tk.Label(row, text='', font=self.f_status,
                                     bg=SURFACE, fg=TEXT_MAIN)
        self.status_label.pack(side='left')
        # Packed only while it has text, so a lone status line sits
        # vertically centred in the header like the pills next to it.
        self.detail_label = tk.Label(st, text='', font=self.f_small, bg=SURFACE,
                                     fg=TEXT_MUTED, anchor='e', justify='right')

    def _show_page(self, name):
        if name in self.DEV_PAGES and not self._dev_unlocked:
            return
        if name == 'home':
            self._dev_unlocked = False
        if self._page:
            self._pages[self._page].pack_forget()
        self._page = name
        self._pages[name].pack(fill='both', expand=True)
        self.page_title.config(text=self.PAGE_TITLES[name])
        if name == 'home':
            self.btn_back.pack_forget()
        elif not self.btn_back.winfo_ismapped():
            self.btn_back.pack(side='left', padx=(px(10), px(6)), pady=px(8), before=self.page_title)
        if name == 'auto':
            self._refresh_stations()

    def _go_back(self):
        self._show_page(self.PAGE_PARENT.get(self._page, 'home'))

    # ---------- Home ----------
    def _build_home(self, p):
        top = tk.Frame(p, bg=BG)
        top.pack(fill='x', pady=(px(4), px(8)))
        tk.Label(top, text='เลือกโหมดการทำงาน', font=self.f_status, bg=BG,
                 fg=TEXT_MUTED).pack(side='left', padx=px(4))
        # Fixed bilingual label (not translated) so it is findable in either language.
        self.btn_lang = self._btn(top, 'ภาษา  ไทย / English', self._toggle_lang,
                                  font=self.f_button_sub, padx=px(18), pady=px(10))
        self.btn_lang._i18n_card = None
        self.btn_lang.pack(side='right', padx=px(4))
        cards = tk.Frame(p, bg=BG)
        cards.pack(fill='both', expand=True)
        cards.columnconfigure(0, weight=1, uniform='c')
        cards.columnconfigure(1, weight=1, uniform='c')
        cards.rowconfigure(0, weight=1)
        self._make_mode_card(
            cards, 'ใช้งานทั่วไป', 'General Operations', 'primary',
            lambda: self._show_page('general'),
        ).grid(row=0, column=0, sticky='nsew', padx=(px(4), px(8)), pady=(px(0), px(8)))
        self._make_mode_card(
            cards, 'โหมดนักพัฒนา', 'Developer Mode', 'info',
            self._on_click_dev,
        ).grid(row=0, column=1, sticky='nsew', padx=(px(8), px(4)), pady=(px(0), px(8)))

    # ---------- PIN modal ----------
    def _on_click_dev(self):
        if self._dev_unlocked or self._ask_pin():
            self._dev_unlocked = True
            self._show_page('dev')

    def _ask_pin(self):
        pin, is_default = get_dev_pin()
        return self._pin_keypad(
            'ใส่ PIN เพื่อเข้าโหมดนักพัฒนา',
            (f'ยังไม่ได้ตั้ง PIN - ใช้ค่าเริ่มต้น (ตั้งได้ที่ {DEV_PIN_FILE})'
             if is_default else '4-6 หลัก'),
            AMBER if is_default else TEXT_MUTED,
            lambda code: None if code == pin else 'PIN ไม่ถูกต้อง') is not None

    def _on_change_pin(self):
        pin, is_default = get_dev_pin()
        if self._pin_keypad(
                'ใส่ PIN ปัจจุบัน',
                'ค่าเริ่มต้น 1234' if is_default else '4-6 หลัก', TEXT_MUTED,
                lambda code: None if code == pin else 'PIN ไม่ถูกต้อง') is None:
            return
        new = self._pin_keypad(
            'ใส่ PIN ใหม่', '4-6 หลัก', TEXT_MUTED,
            lambda code: None if re.fullmatch(r'\d{4,6}', code) else 'ต้องเป็นตัวเลข 4-6 หลัก')
        if new is None:
            return
        if self._pin_keypad(
                'ยืนยัน PIN ใหม่อีกครั้ง', '4-6 หลัก', TEXT_MUTED,
                lambda code: None if code == new else 'PIN ไม่ตรงกัน') is None:
            return
        try:
            tmp = DEV_PIN_FILE + '.tmp'
            fd = os.open(tmp, os.O_WRONLY | os.O_CREAT | os.O_TRUNC, 0o600)
            with os.fdopen(fd, 'w') as f:
                f.write(new + '\n')
            os.replace(tmp, DEV_PIN_FILE)
        except OSError as e:
            messagebox.showerror('เปลี่ยน PIN ไม่สำเร็จ', str(e))
            return
        messagebox.showinfo('เปลี่ยน PIN แล้ว', 'ใช้ PIN ใหม่ตั้งแต่ครั้งถัดไป')

    def _pin_keypad(self, title, hint_text, hint_color, check):
        """Modal touch keypad. check(code) returns None if accepted, else
        an error message (keypad stays open). Returns the code or None."""
        result = {'code': None}
        entered = []
        win = tk.Toplevel(self)
        win.title('PIN')
        win.configure(bg=SURFACE)
        win.transient(self)
        win.resizable(False, False)

        self._label(win, title, font=self.f_button).pack(pady=(px(16), px(4)))
        hint = self._label(win, hint_text, font=self.f_small, fg=hint_color)
        hint.pack()
        disp = self._label(win, '', font=self.f_huge, width=10, bg=SURFACE_2)
        disp.pack(pady=px(10), ipady=px(6))

        def refresh():
            disp.config(text='●' * len(entered) or ' ')

        def press(d):
            if len(entered) < 6:
                entered.append(d)
                refresh()

        def back():
            if entered:
                entered.pop()
                refresh()

        def verify(_e=None):
            code = ''.join(entered)
            err = check(code)
            if err is None:
                result['code'] = code
                win.destroy()
            else:
                entered.clear()
                refresh()
                hint.config(text=T(err), fg=RED)

        pad = tk.Frame(win, bg=SURFACE)
        pad.pack(padx=px(20))
        keys = ['1', '2', '3', '4', '5', '6', '7', '8', '9', '⌫', '0', 'OK']
        for i, k in enumerate(keys):
            if k == '⌫':
                b = self._btn(pad, k, back, font=self.f_button, width=4)
            elif k == 'OK':
                b = self._btn(pad, k, verify, kind='primary', font=self.f_button, width=4)
            else:
                b = self._btn(pad, k, lambda d=k: press(d), font=self.f_button, width=4)
            b.grid(row=i // 3, column=i % 3, padx=px(4), pady=px(4))
        self._btn(win, 'ยกเลิก', win.destroy).pack(pady=(px(8), px(16)))

        for d in '0123456789':
            win.bind(d, lambda _e, d=d: press(d))
        win.bind('<BackSpace>', lambda _e: back())
        win.bind('<Return>', verify)
        win.bind('<Escape>', lambda _e: win.destroy())
        win.update_idletasks()
        self._center_toplevel(win, win.winfo_reqwidth() + 10, win.winfo_reqheight() + 10)
        win.grab_set()
        win.focus_set()
        win.wait_window()
        return result['code']

    # ---------- General ----------
    def _build_general(self, p):
        cards = tk.Frame(p, bg=BG)
        cards.pack(fill='both', expand=True, pady=(px(8), px(0)))
        cards.columnconfigure(0, weight=1, uniform='c')
        cards.columnconfigure(1, weight=1, uniform='c')
        cards.rowconfigure(0, weight=1)
        self._make_mode_card(
            cards, 'อัตโนมัติ', 'Auto Mode', 'primary',
            lambda: self._show_page('auto'),
        ).grid(row=0, column=0, sticky='nsew', padx=(px(4), px(8)), pady=(px(0), px(8)))
        self._make_mode_card(
            cards, 'ควบคุมเอง', 'Manual Mode', 'warn',
            lambda: self._show_page('manual'),
        ).grid(row=0, column=1, sticky='nsew', padx=(px(8), px(4)), pady=(px(0), px(8)))

    # ---------- Auto ----------
    def _build_auto(self, p):
        p.columnconfigure(0, weight=3)
        p.columnconfigure(1, weight=2)
        p.rowconfigure(0, weight=1)

        self.auto_map = MapPlaceholder(p)
        self.auto_map.grid(row=0, column=0, sticky='nsew', padx=(px(0), px(8)))

        right = tk.Frame(p, bg=BG)
        right.grid(row=0, column=1, sticky='nsew')

        sys_card = self._card(right)
        sys_card.pack(fill='x')
        self.btn_auto_start = self._btn(
            sys_card, 'เริ่มระบบนำทาง (เลือกแผนที่)', self._on_click_navigation,
            kind='info')
        self.btn_auto_start.pack(fill='x', padx=px(10), pady=px(10))
        self._start_buttons.append(self.btn_auto_start)

        st_card = self._card(right)
        st_card.pack(fill='both', expand=True, pady=px(8))
        self._section_title(st_card, 'จุดหมาย (Destination)').pack(fill='x', padx=px(10), pady=(px(8), px(2)))
        self.station_grid = tk.Frame(st_card, bg=SURFACE)
        self.station_grid.pack(fill='both', expand=True, padx=px(8), pady=(px(0), px(8)))

        status = self._card(right)
        status.pack(fill='x')
        self.mission_label = self._label(status, 'ยังไม่มีภารกิจ', anchor='w',
                                         justify='left')
        self.mission_label.pack(fill='x', padx=px(10), pady=(px(8), px(2)))
        self.mission_bar = ttk.Progressbar(status, maximum=100)
        self.mission_bar.pack(fill='x', padx=px(10), pady=(px(0), px(8)))

        acts = tk.Frame(right, bg=BG)
        acts.pack(fill='x', pady=(px(8), px(0)))
        acts.columnconfigure((0, 1, 2), weight=1, uniform='a')
        self.btn_mission_start = self._btn(acts, 'START\nMISSION',
                                           self._on_mission_start, kind='primary')
        self.btn_mission_start.grid(row=0, column=0, sticky='nsew', padx=(px(0), px(4)))
        self.btn_mission_pause = self._btn(acts, 'PAUSE', self._on_mission_pause,
                                           kind='warn')
        self.btn_mission_pause.grid(row=0, column=1, sticky='nsew', padx=px(4))
        self.btn_mission_cancel = self._btn(acts, 'CANCEL /\nกลับ Base',
                                            self._on_mission_cancel, kind='danger')
        self.btn_mission_cancel.grid(row=0, column=2, sticky='nsew', padx=(px(4), px(0)))

    def _refresh_stations(self):
        for w in self.station_grid.winfo_children():
            w.destroy()
        self._stations = load_stations(self._nav_map_name) if self._nav_map_name else []
        if not self._stations:
            msg = ('เริ่มระบบนำทางก่อน' if not self._nav_map_name else
                   f'แผนที่ {self._nav_map_name} ยังไม่มี Station\n'
                   '(เพิ่มได้ที่ Developer › Navigation Test)')
            self._label(self.station_grid, T(msg), font=self.f_small, fg=TEXT_MUTED,
                        justify='left').pack(anchor='w', pady=px(6))
            return
        for i, s in enumerate(self._stations[:8]):
            sel = self._selected_station == s['name']
            # colour stripe = the station's pin colour on the map
            cell = tk.Frame(self.station_grid, bg=station_color(i))
            cell.grid(row=i // 2, column=i % 2, sticky='ew', padx=px(3), pady=px(3))
            b = self._btn(cell, s['name'],
                          lambda n=s['name']: self._select_station(n),
                          kind=('warn' if sel else 'neutral'), pady=px(10))
            b.pack(fill='x', padx=(px(8), 0))
        self.station_grid.columnconfigure((0, 1), weight=1, uniform='s')

    def _select_station(self, name):
        self._selected_station = name
        self._refresh_stations()
        self._update_mission_ui()

    def _station(self, name):
        return next((s for s in self._stations if s['name'] == name), None)

    def _base_station(self):
        return next((s for s in self._stations
                     if s['name'].lower() in ('base', 'home', 'dock', 'charging dock')), None)

    def _on_mission_start(self):
        st = self._station(self._selected_station)
        if st is None or self._mode != 'navigation':
            return
        if not messagebox.askyesno(
                'ยืนยันภารกิจ',
                f'ส่งหุ่นยนต์ไปที่ "{st["name"]}" แบบอัตโนมัติ?\n\n'
                'ตรวจสอบเส้นทางว่าไม่มีคน/สิ่งกีดขวางก่อนยืนยัน (ISO 3691-4)'):
            return
        self._start_mission(st)

    def _start_mission(self, st):
        pose = self.ros.pose()
        d0 = math.hypot(st['x'] - pose[0], st['y'] - pose[1]) if pose else None
        if self.ros.send_goal(st['x'], st['y'], st['theta'], station=True):
            self._mission = {'station': st, 'state': 'active', 'd0': d0}
        self._update_mission_ui()

    def _on_mission_pause(self):
        m = self._mission
        if not m:
            return
        if m['state'] == 'active':
            self.ros.cancel_goal()
            m['state'] = 'paused'
        elif m['state'] == 'paused':
            self.ros.send_goal(m['station']['x'], m['station']['y'], m['station']['theta'],
                               station=True)
            m['state'] = 'active'
        self._update_mission_ui()

    def _on_mission_cancel(self):
        base = self._base_station()
        if self._mission and self._mission['state'] in ('active', 'paused'):
            self.ros.cancel_goal()
            self._mission = None
        elif base is not None and self._mode == 'navigation':
            if messagebox.askyesno('กลับ Base', f'ส่งหุ่นยนต์กลับ "{base["name"]}"?'):
                self._start_mission(base)
                return
        self._update_mission_ui()

    def _update_mission_ui(self):
        nav = self._mode == 'navigation'
        m = self._mission
        active = m is not None and m['state'] in ('active', 'paused')
        self._set_btn_state(self.btn_mission_start,
                            nav and not active and self._station(self._selected_station) is not None)
        self._set_btn_state(self.btn_mission_pause, nav and active)
        self.btn_mission_pause.config(
            text=('RESUME' if m and m['state'] == 'paused' else 'PAUSE'))
        self._set_btn_state(self.btn_mission_cancel,
                            nav and (active or self._base_station() is not None))
        self.btn_mission_cancel.config(text=T('CANCEL' if active else 'กลับ Base'))

        if not m:
            sel = self._selected_station
            self.mission_label.config(text=T(f'เลือก: {sel}' if sel else 'ยังไม่มีภารกิจ'))
            self.mission_bar['value'] = 0
            return
        st = m['station']
        pose = self.ros.pose()
        txt = f'เป้าหมาย: {st["name"]}  ·  '
        if pose is not None:
            d = math.hypot(st['x'] - pose[0], st['y'] - pose[1])
            v = abs(self.ros.odom[0]) if self.ros.fresh(self.ros.odom, 1.0) else 0.0
            if m['d0'] is None:
                m['d0'] = max(d, 0.01)
            self.mission_bar['value'] = max(0, min(100, 100 * (1 - d / max(m['d0'], 0.01))))
            if m['state'] == 'active' and d < ARRIVE_DIST_M and v < ARRIVE_SPEED:
                m['state'] = 'arrived'
            eta = d / max(v, 0.1)
            txt += {'active': f'เหลือ {d:.1f} ม. · ETA ~{eta:.0f} s',
                    'paused': f'หยุดชั่วคราว · เหลือ {d:.1f} ม.',
                    'arrived': 'ถึงแล้ว ✓'}[m['state']]
        else:
            txt += {'active': 'กำลังไป...', 'paused': 'หยุดชั่วคราว',
                    'arrived': 'ถึงแล้ว ✓'}[m['state']]
        self.mission_label.config(text=T(txt))
        if m['state'] == 'arrived':
            self.mission_bar['value'] = 100

    # ---------- Manual ----------
    def _build_manual(self, p):
        p.columnconfigure(0, weight=1, uniform='m')
        p.columnconfigure(1, weight=1, uniform='m')
        p.rowconfigure(1, weight=1)

        self.manual_banner = tk.Label(
            p, text='Manual Override - ควบคุมด้วยรีโมท/จอยภายนอก ขับช้าและมองเส้นทางตลอดเวลา',
            font=self.f_button_sub, bg=AMBER, fg='#1a1a1a', pady=px(8))
        self.manual_banner.grid(row=0, column=0, columnspan=2, sticky='ew', pady=(px(0), px(8)))

        left = self._card(p)
        left.grid(row=1, column=0, sticky='nsew', padx=(px(0), px(6)))
        self._section_title(left, 'Remote input (cmd_vel)').pack(fill='x', padx=px(10), pady=(px(8), px(0)))
        self.remote_status = self._label(left, 'Controller: —', font=self.f_button)
        self.remote_status.pack(anchor='w', padx=px(10))
        self.vector_canvas = tk.Canvas(left, width=px(260), height=px(260), bg=SURFACE,
                                       highlightthickness=0)
        self.vector_canvas.pack(pady=px(6))

        right = self._card(p)
        right.grid(row=1, column=1, sticky='nsew', padx=(px(6), px(0)))
        self._section_title(right, 'Telemetry (/odometry/filtered)').pack(fill='x', padx=px(10), pady=(px(8), px(0)))
        self.gauge_canvas = tk.Canvas(right, width=px(440), height=px(190), bg=SURFACE,
                                      highlightthickness=0)
        self.gauge_canvas.pack(pady=px(4))
        row = tk.Frame(right, bg=SURFACE)
        row.pack(fill='x', padx=px(10), pady=(px(4), px(10)), side='bottom')
        self.btn_manual_start = self._btn(row, 'เริ่มระบบ Manual', self._on_click_manual,
                                          kind='warn')
        self.btn_manual_start.pack(side='left', fill='x', expand=True, padx=(px(0), px(4)))
        self._start_buttons.append(self.btn_manual_start)
        self._btn(row, 'QR มือถือ', lambda: threading.Thread(
            target=self._ensure_monitors_then_show_qr, daemon=True).start()
        ).pack(side='left', padx=px(4))
        self.btn_manual_stop = self._btn(row, 'ปิดระบบ', self._on_stop, kind='danger')
        self.btn_manual_stop.pack(side='left', padx=(px(4), px(0)))

    def _on_click_manual(self):
        if self._proc is not None:
            return
        self._launch([
            'ros2', 'launch', 'amr_bringup', 'bringup.launch.py',
            'rviz:=false', 'foxglove:=true', 'slam:=false', 'navigation:=false',
        ], mode='manual', label='โหมดควบคุมเอง (Manual)')
        threading.Thread(target=self._ensure_monitors_then_show_qr,
                         daemon=True).start()

    def _draw_manual(self):
        c = self.vector_canvas
        c.delete('all')
        cx = cy = px(130)
        r = px(110)
        c.create_oval(cx - r, cy - r, cx + r, cy + r, outline=BORDER, width=px(2))
        c.create_line(cx - r, cy, cx + r, cy, fill=BORDER)
        c.create_line(cx, cy - r, cx, cy + r, fill=BORDER)
        active = self.ros.fresh(self.ros.cmd, 0.5)
        if active:
            vx, wz = self.ros.cmd[0], self.ros.cmd[1]
            tx = cx - max(-1, min(1, wz / 1.0)) * r
            ty = cy - max(-1, min(1, vx / MOVE_FORWARD_SPEED_MAX)) * r
            d = px(8)
            c.create_line(cx, cy, tx, ty, fill=AMBER, width=px(5), arrow='last')
            c.create_oval(tx - d, ty - d, tx + d, ty + d, fill=AMBER, outline='')
        self.remote_status.config(
            text=T('Controller: ACTIVE (มีคำสั่ง)' if active else 'Controller: idle / disconnected'),
            fg=(AMBER if active else TEXT_MUTED))

        g = self.gauge_canvas
        g.delete('all')
        odom = self.ros.odom if self.ros.fresh(self.ros.odom, 1.0) else None
        for i, (name, val, vmax, unit) in enumerate((
                ('v', odom[0] if odom else None, MOVE_FORWARD_SPEED_MAX, 'm/s'),
                ('ω', odom[1] if odom else None, 1.0, 'rad/s'))):
            ox, oy, rr = px(110 + i * 220), px(120), px(85)
            g.create_arc(ox - rr, oy - rr, ox + rr, oy + rr, start=0, extent=180,
                         style='arc', outline=BORDER, width=px(14))
            if val is not None:
                frac = max(-1, min(1, val / vmax))
                col = RED if abs(frac) > 0.9 else GREEN
                g.create_arc(ox - rr, oy - rr, ox + rr, oy + rr, start=90,
                             extent=-90 * frac, style='arc', outline=col, width=px(14))
            g.create_text(ox, oy - px(20), text=(f'{val:+.2f}' if val is not None else '—'),
                          fill=TEXT_MAIN, font=self.f_huge)
            g.create_text(ox, oy + px(12), text=f'{name} [{unit}]', fill=TEXT_MUTED,
                          font=self.f_small)

    # ---------- Developer menu ----------
    def _build_dev(self, p):
        cards = tk.Frame(p, bg=BG)
        cards.pack(fill='both', expand=True, pady=(px(8), px(0)))
        cards.rowconfigure(0, weight=1)
        for i, (th, en, kind, page) in enumerate((
                ('สร้างแผนที่', 'Mapping', 'primary', 'mapping'),
                ('ทดสอบนำทาง', 'Navigation Test', 'info', 'navtest'),
                ('แก้ไขแผนที่', 'Map Edit', 'neutral', None),
                ('ตั้งค่า', 'Settings', 'neutral', 'settings'))):
            cards.columnconfigure(i, weight=1, uniform='d')
            cmd = (self._on_map_edit if page is None
                   else lambda pg=page: self._show_page(pg))
            self._make_mode_card(cards, th, en, kind, cmd
                                 ).grid(row=0, column=i, sticky='nsew', padx=px(4), pady=(px(0), px(8)))

    # ---------- Map Edit (popup) ----------
    # Hand-editing goes PGM -> PNG -> Pinta -> PNG -> new PGM through
    # amr_navigation/scripts/map_edit.py (GIMP's file plug-ins crash on the
    # Jetson; Pinta cannot write PGM). map_edit.py never overwrites a map:
    # the result is <map>_eN with a copy of the stations.
    def _run_map_edit(self, *args):
        try:
            r = subprocess.run([sys.executable, MAP_EDIT, *args], stdout=subprocess.PIPE,
                               stderr=subprocess.STDOUT, text=True, timeout=60)
            return r.returncode, r.stdout.strip()
        except (OSError, subprocess.TimeoutExpired) as e:
            return 1, str(e)

    def _on_map_edit(self):
        yaml_path = self._show_map_picker(title='เลือกแผนที่ที่จะแก้ไข',
                                          ok_text='แก้ไขแผนที่นี้')
        if not yaml_path:
            return
        name = os.path.splitext(os.path.basename(yaml_path))[0]
        png = os.path.join(MAPS_DIR, name + '_edit.png')

        win = tk.Toplevel(self)
        win.title('Map Edit')
        win.configure(bg=SURFACE)
        win.transient(self)
        self._label(win, f'แก้ไขแผนที่: {name}', font=self.f_button
                    ).pack(padx=px(20), pady=(px(16), px(4)), anchor='w')
        self._label(win, T('ใน Pinta: ใช้ Pencil + ปิด Antialiasing · ดำ 000000 = สิ่งกีดขวาง · '
                           'ขาว FEFEFE = ที่ว่าง · เทา CDCDCD = unknown\n'
                           'ห้ามครอป/ย่อ/หมุนภาพ · กด Save (PNG) ใน Pinta แล้วกลับมากดขั้นที่ 2'),
                    font=self.f_small, fg=TEXT_MUTED, justify='left'
                    ).pack(padx=px(20), anchor='w')
        out = tk.Text(win, height=14, width=78, bg=SURFACE_2, fg=TEXT_MAIN,
                      font=('Monospace', -px(13)), relief='flat', wrap='word')
        out.pack(padx=px(20), pady=px(10), fill='both', expand=True)

        def show(text, color=TEXT_MAIN):
            out.config(state='normal', fg=color)
            out.delete('1.0', 'end')
            out.insert('end', text)
            out.config(state='disabled')

        def open_pinta():
            if os.path.exists(png):
                if not messagebox.askyesno(
                        'Map Edit', f'มีไฟล์ที่แก้ค้างไว้ ({os.path.basename(png)})\n'
                        'Yes = แก้ต่อจากไฟล์เดิม\nNo = เริ่มใหม่จากแผนที่ต้นฉบับ', parent=win):
                    rc, txt = self._run_map_edit('to-png', yaml_path, '-o', png, '--force')
                    if rc:
                        return show(txt, RED)
            else:
                rc, txt = self._run_map_edit('to-png', yaml_path, '-o', png)
                if rc:
                    return show(txt, RED)
            try:
                subprocess.Popen(['pinta', png], start_new_session=True,
                                 stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                show(f'เปิด Pinta: {png}\nแก้เสร็จแล้วกด Save ใน Pinta (PNG) แล้วกดขั้นที่ 2')
            except OSError as e:
                show(f'เปิด Pinta ไม่ได้: {e}\nแก้ไฟล์นี้ด้วยโปรแกรมอื่นได้: {png}', RED)

        def check():
            if not os.path.exists(png):
                return show('ยังไม่มีไฟล์ที่แก้ - กดขั้นที่ 1 ก่อน', AMBER), None
            rc, txt = self._run_map_edit('to-pgm', yaml_path, png, '--dry-run')
            warn = rc or 'obstacle -> free' in txt or 'WARNING' in txt
            show(txt, RED if rc else (AMBER if warn else TEXT_MAIN))
            return txt, rc

        def save():
            txt, rc = check()
            if txt is None or rc:
                return
            msg = 'บันทึกเป็นแผนที่ใหม่ (ไม่ทับแผนที่เดิม)?'
            if 'obstacle -> free' in txt:
                msg += ('\n\n⚠ มีสิ่งกีดขวางถูกลบออก - ตรวจหน้างานจริงทุกจุดใน list '
                        'ว่าไม่มีของอยู่แล้ว')
            if not messagebox.askyesno('Map Edit', msg, parent=win):
                return
            rc, txt = self._run_map_edit('to-pgm', yaml_path, png)
            if rc:
                return show(txt, RED)
            try:
                os.remove(png)  # done - next edit starts from the new map
            except OSError:
                pass
            show(txt + '\n\nเลือกแผนที่ใหม่ได้ตอนเริ่ม Navigation · '
                 'ทดสอบวิ่งทุกเส้นทางก่อนใช้ในไลน์ · commit แผนที่ใหม่เข้า git', GREEN)

        row = tk.Frame(win, bg=SURFACE)
        row.pack(fill='x', padx=px(20), pady=(0, px(16)))
        self._btn(row, '① เปิดแก้ใน Pinta', open_pinta, kind='info').pack(side='left')
        self._btn(row, '② ตรวจการเปลี่ยนแปลง', check).pack(side='left', padx=px(8))
        self._btn(row, '③ บันทึกเป็นแผนที่ใหม่', save, kind='primary').pack(side='left')
        self._btn(row, 'ปิด', win.destroy).pack(side='right')
        show('ขั้นตอน: ① เปิดแก้ใน Pinta → Save → ② ตรวจ → ③ บันทึก')
        win.update_idletasks()
        self._center_toplevel(win, win.winfo_reqwidth() + 10, win.winfo_reqheight() + 10)

    # ---------- Mapping ----------
    def _build_mapping(self, p):
        p.columnconfigure(0, weight=3)
        p.columnconfigure(1, weight=2)
        p.rowconfigure(1, weight=1)
        self.mapping_banner = tk.Label(
            p, text='ใช้ Joystick (ฮาร์ดแวร์/มือถือ) ขับหุ่นยนต์ช้าๆ ให้ทั่วพื้นที่',
            font=self.f_button_sub, bg=AMBER, fg='#1a1a1a', pady=px(8))
        self.mapping_banner.grid(row=0, column=0, columnspan=2, sticky='ew', pady=(px(0), px(8)))

        self.mapping_map = MapPlaceholder(p)
        self.mapping_map.grid(row=1, column=0, sticky='nsew', padx=(px(0), px(8)))

        right = tk.Frame(p, bg=BG)
        right.grid(row=1, column=1, sticky='nsew')

        ctl = self._card(right)
        ctl.pack(fill='x')
        self.btn_mapping = self._btn(ctl, 'START MAPPING', self._on_click_mapping,
                                     kind='primary', font=self.f_button)
        self.btn_mapping.pack(fill='x', padx=px(10), pady=(px(10), px(6)))
        self._start_buttons.append(self.btn_mapping)
        self.btn_scan = self._btn(ctl, 'เริ่มสแกน', self._on_click_scan, kind='warn')
        self.btn_scan.pack(fill='x', padx=px(10), pady=px(6))
        self.btn_save_map = self._btn(ctl, 'SAVE MAP (บันทึกแผนที่)', self._on_save_map,
                                      kind='info')
        self.btn_save_map.pack(fill='x', padx=px(10), pady=px(6))
        self.btn_discard = self._btn(ctl, 'DISCARD / RESET', self._on_discard_map,
                                     kind='danger')
        self.btn_discard.pack(fill='x', padx=px(10), pady=px(6))
        self._btn(ctl, 'QR มือถือ (แผนที่ + Joystick)', lambda: threading.Thread(
            target=self._ensure_monitors_then_show_qr, daemon=True).start()
        ).pack(fill='x', padx=px(10), pady=(px(6), px(10)))

        diag = self._card(right)
        diag.pack(fill='both', expand=True, pady=(px(8), px(0)))
        self._section_title(diag, 'Sensor Diagnostics').pack(fill='x', padx=px(10), pady=(px(8), px(2)))
        self.diag_labels = {}
        for key, text in (('scan', 'LiDAR /scan'), ('odom', 'Odometry'),
                          ('imu_hz', 'IMU /imu/data'), ('depth', 'Depth watchdog')):
            row = tk.Frame(diag, bg=SURFACE)
            row.pack(fill='x', padx=px(10), pady=px(2))
            self._label(row, text, anchor='w').pack(side='left')
            v = self._label(row, '—', fg=TEXT_MUTED)
            v.pack(side='right')
            self.diag_labels[key] = v

    def _on_discard_map(self):
        if self._mode != 'mapping' or self._proc is None:
            return
        if messagebox.askyesno('Discard / Reset',
                               'หยุด SLAM และทิ้งแผนที่ที่ยังไม่ได้บันทึก?'):
            self._on_stop()

    def _update_diag(self):
        hz_s, hz_o = self.ros.rate('scan'), self.ros.rate('odom')
        self.diag_labels['scan'].config(text=f'{hz_s:.1f} Hz',
                                        fg=GREEN if hz_s > 3 else RED)
        self.diag_labels['odom'].config(text=f'{hz_o:.1f} Hz',
                                        fg=GREEN if hz_o > 5 else RED)
        hz_i = self.ros.rate('imu')
        self.diag_labels['imu_hz'].config(text=f'{hz_i:.1f} Hz',
                                          fg=GREEN if hz_i > 5 else RED)
        self._show_watchdogs(self.diag_labels, ('depth',))

    def _show_watchdogs(self, labels, keys):
        for k in keys:
            wd = self.ros.watchdog.get(k)
            if wd is None or time.monotonic() - wd[1] > 3:
                labels[k].config(text=T('ไม่มีข้อมูล'), fg=TEXT_MUTED)
            else:
                labels[k].config(text='OK' if wd[0] else 'FAULT',
                                 fg=GREEN if wd[0] else RED)

    # ---------- Navigation test ----------
    def _build_navtest(self, p):
        p.columnconfigure(0, weight=1)
        p.columnconfigure(1, weight=0, minsize=px(450))
        p.rowconfigure(0, weight=1)

        self.nav_map = MapPlaceholder(p, on_tap=self._on_map_tap)
        self.nav_map.grid(row=0, column=0, sticky='nsew', padx=(px(0), px(8)))

        right = tk.Frame(p, bg=BG, width=px(450))
        right.grid(row=0, column=1, sticky='nsew')
        right.pack_propagate(False)

        top = self._card(right)
        top.pack(fill='x')
        row = tk.Frame(top, bg=SURFACE)
        row.pack(fill='x', padx=px(8), pady=px(8))
        self.btn_navigation = self._btn(row, 'เริ่ม Navigation', self._on_click_navigation,
                                        kind='info')
        self.btn_navigation.pack(side='left', fill='x', expand=True, padx=(px(0), px(4)))
        self._start_buttons.append(self.btn_navigation)
        self.btn_stop = self._btn(row, 'ปิดระบบ (Stop)', self._on_stop, kind='danger')
        self.btn_stop.pack(side='left', padx=(px(4), px(0)))

        style = ttk.Style(self)
        style.theme_use('default')
        style.configure('Dark.TNotebook', background=BG, borderwidth=0)
        style.configure('Dark.TNotebook.Tab', font=self.f_status, padding=(px(14), px(10)),
                        background=SURFACE_2, foreground=TEXT_MUTED)
        style.map('Dark.TNotebook.Tab', background=[('selected', SURFACE)],
                  foreground=[('selected', TEXT_MAIN)])
        style.configure('TProgressbar', troughcolor=SURFACE_2, background=GREEN)

        nb = ttk.Notebook(right, style='Dark.TNotebook')
        nb.pack(fill='both', expand=True, pady=(px(8), px(0)))
        t_goal = tk.Frame(nb, bg=SURFACE)
        t_motion = tk.Frame(nb, bg=SURFACE)
        t_log = tk.Frame(nb, bg=SURFACE)
        nb.add(t_goal, text='2D Goal')
        nb.add(t_motion, text='Motion Test')
        nb.add(t_log, text='Log')

        # --- 2D goal tab ---
        self._label(t_goal, 'แตะบนแผนที่ = ตั้งจุดหมาย, ลาก = กำหนดทิศ\n'
                            'ยืนยันก่อนหุ่นยนต์ออกตัวทุกครั้ง',
                    font=self.f_small, fg=TEXT_MUTED, justify='left'
                    ).pack(anchor='w', padx=px(10), pady=(px(10), px(6)))
        self.btn_goal_cancel = self._btn(t_goal, 'ยกเลิก Goal', self._on_nav_cancel,
                                         kind='danger')
        self.btn_goal_cancel.pack(fill='x', padx=px(10), pady=px(4))
        self.btn_add_station = self._btn(t_goal, '＋ บันทึกตำแหน่งนี้เป็น Station',
                                         self._on_add_station)
        self.btn_add_station.pack(fill='x', padx=px(10), pady=px(4))
        self.btn_del_station = self._btn(t_goal, 'ลบ Station...', self._on_del_station)
        self.btn_del_station.pack(fill='x', padx=px(10), pady=px(4))
        self._btn(t_goal, 'QR ตั้งจุดหมายจากมือถือ', lambda: threading.Thread(
            target=self._ensure_nav_monitors_then_show_qr, daemon=True).start()
        ).pack(fill='x', padx=px(10), pady=px(4))
        self._section_title(t_goal, 'Planner (อ่านอย่างเดียว - แก้ใน nav2_params.yaml)'
                            ).pack(fill='x', padx=px(10), pady=(px(10), px(0)))
        self._label(t_goal, 'Global: NavFn   ·   Local: MPPI', font=self.f_small
                    ).pack(anchor='w', padx=px(10))
        # imu_watchdog_node runs only with Navigation, so it is shown here
        # rather than on the Mapping page.
        self._section_title(t_goal, 'Watchdogs').pack(fill='x', padx=px(10), pady=(px(10), 0))
        self.nav_wd_labels = {}
        for key, text in (('imu', 'IMU watchdog'), ('depth', 'Depth watchdog')):
            row = tk.Frame(t_goal, bg=SURFACE)
            row.pack(fill='x', padx=px(10))
            self._label(row, text, font=self.f_small, anchor='w').pack(side='left')
            v = self._label(row, '—', font=self.f_small, fg=TEXT_MUTED)
            v.pack(side='right')
            self.nav_wd_labels[key] = v

        # --- motion test tab (presets kept from the previous layout) ---
        sub = ttk.Notebook(t_motion, style='Dark.TNotebook')
        sub.pack(fill='both', expand=True, padx=px(4), pady=px(4))
        tab_forward = tk.Frame(sub, bg=SURFACE)
        sub.add(tab_forward, text='ตรง')
        tab_square = tk.Frame(sub, bg=SURFACE)
        sub.add(tab_square, text='สี่เหลี่ยม')
        tab_circle = tk.Frame(sub, bg=SURFACE)
        sub.add(tab_circle, text='วงกลม')

        self._move_distance = 1.0
        row, self.btn_move_minus, self.move_distance_label, self.btn_move_plus = self._stepper(
            tab_forward, 'ระยะ:', '1.0 ม.',
            lambda: self._adjust_move_distance(-MOVE_FORWARD_STEP_M),
            lambda: self._adjust_move_distance(MOVE_FORWARD_STEP_M))
        row.pack(anchor='w', padx=px(8), pady=(px(8), px(4)))
        self._move_speed = MOVE_FORWARD_SPEED_DEFAULT
        row, self.btn_speed_minus, self.move_speed_label, self.btn_speed_plus = self._stepper(
            tab_forward, 'ความเร็ว:', f'{MOVE_FORWARD_SPEED_DEFAULT:.2f} m/s',
            lambda: self._adjust_move_speed(-MOVE_FORWARD_SPEED_STEP),
            lambda: self._adjust_move_speed(MOVE_FORWARD_SPEED_STEP))
        row.pack(anchor='w', padx=px(8), pady=px(4))
        self.btn_move_forward = self._btn(tab_forward, 'เดินหน้า (Move Forward)',
                                          self._on_click_move_forward, kind='info')
        self.btn_move_forward.pack(fill='x', padx=px(8), pady=px(6))
        self._label(tab_forward,
                    f'{MOVE_FORWARD_SPEED_MIN:.2f}-{MOVE_FORWARD_SPEED_MAX:.2f} m/s, '
                    f'{MOVE_FORWARD_MIN_M:.1f}-{MOVE_FORWARD_MAX_M:.1f} ม. - '
                    'ตรวจพื้นที่ด้านหน้าก่อนกด (ISO 3691-4)',
                    font=self.f_small, fg=TEXT_MUTED, wraplength=px(400), justify='left'
                    ).pack(anchor='w', padx=px(8))

        # Square path (4x straight leg + 90 deg turn, closed-loop, returns
        # to the starting point/heading - see square_path_node.py). Shares
        # tab_forward's speed control (self._move_speed).
        self._square_side = SQUARE_SIDE_DEFAULT
        row, self.btn_square_minus, self.square_side_label, self.btn_square_plus = self._stepper(
            tab_square, 'ด้าน:', f'{SQUARE_SIDE_DEFAULT:.1f} ม./ด้าน',
            lambda: self._adjust_square_side(-SQUARE_SIDE_STEP),
            lambda: self._adjust_square_side(SQUARE_SIDE_STEP), width=10)
        row.pack(anchor='w', padx=px(8), pady=(px(8), px(4)))
        self.btn_square_path = self._btn(tab_square, 'วิ่งเป็นสี่เหลี่ยม (Square Path)',
                                         self._on_click_square_path, kind='info')
        self.btn_square_path.pack(fill='x', padx=px(8), pady=px(6))
        self._label(tab_square,
                    f'ด้านละ {SQUARE_SIDE_MIN:.1f}-{SQUARE_SIDE_MAX:.1f} ม. ใช้ความเร็วจากแท็บ "ตรง" '
                    'หมุนทวนเข็ม 90° ต่อมุม - ตรวจพื้นที่โดยรอบก่อนกด (ISO 3691-4)',
                    font=self.f_small, fg=TEXT_MUTED, wraplength=px(400), justify='left'
                    ).pack(anchor='w', padx=px(8))

        # Circle path (constant-curvature arc, closed-loop - see
        # circle_path_node.py).
        self._circle_radius = CIRCLE_RADIUS_DEFAULT
        row, self.btn_circle_radius_minus, self.circle_radius_label, self.btn_circle_radius_plus = \
            self._stepper(tab_circle, 'รัศมี:', f'{CIRCLE_RADIUS_DEFAULT:.1f} ม.',
                          lambda: self._adjust_circle_radius(-CIRCLE_RADIUS_STEP),
                          lambda: self._adjust_circle_radius(CIRCLE_RADIUS_STEP))
        row.pack(anchor='w', padx=px(8), pady=(px(8), px(4)))
        self._circle_angle = CIRCLE_ANGLE_DEFAULT
        row, self.btn_circle_angle_minus, self.circle_angle_label, self.btn_circle_angle_plus = \
            self._stepper(tab_circle, 'มุม:', f'{CIRCLE_ANGLE_DEFAULT:.0f}°',
                          lambda: self._adjust_circle_angle(-CIRCLE_ANGLE_STEP),
                          lambda: self._adjust_circle_angle(CIRCLE_ANGLE_STEP))
        row.pack(anchor='w', padx=px(8), pady=px(4))
        self.btn_circle_path = self._btn(tab_circle, 'วิ่งวงกลม (Circle Path)',
                                         self._on_click_circle_path, kind='info')
        self.btn_circle_path.pack(fill='x', padx=px(8), pady=px(6))
        self._label(tab_circle,
                    f'ω คงที่ {CIRCLE_ANGULAR_SPEED:.1f} rad/s - ตรวจพื้นที่โดยรอบก่อนกด (ISO 3691-4)',
                    font=self.f_small, fg=TEXT_MUTED, wraplength=px(400), justify='left'
                    ).pack(anchor='w', padx=px(8))

        # --- log tab ---
        lrow = tk.Frame(t_log, bg=SURFACE)
        lrow.pack(fill='x', padx=px(6), pady=(px(6), px(0)))
        self._log_warn_only = tk.BooleanVar(value=True)
        tk.Checkbutton(lrow, text='เฉพาะ WARN/ERROR', variable=self._log_warn_only,
                       font=self.f_small, bg=SURFACE, fg=TEXT_MAIN, selectcolor=SURFACE_2,
                       activebackground=SURFACE, command=self._render_log
                       ).pack(side='left')
        self.log_text = tk.Text(t_log, bg='#0d1117', fg=TEXT_MAIN, font=self.f_mono,
                                relief='flat', wrap='word', state='disabled')
        self.log_text.pack(fill='both', expand=True, padx=px(6), pady=px(6))
        self.log_text.tag_config('W', foreground=AMBER)
        self.log_text.tag_config('E', foreground=RED)
        self.log_text.tag_config('F', foreground=RED)
        self._log_seq_shown = -1

    def _on_map_tap(self, x, y, theta):
        if self._mode != 'navigation':
            messagebox.showinfo('ยังไม่พร้อม', 'เริ่ม Navigation ก่อนตั้งจุดหมาย')
            return
        if not messagebox.askyesno(
                'ยืนยัน 2D Goal',
                f'ส่งหุ่นยนต์ไป x={x:.2f}, y={y:.2f}, θ={math.degrees(theta):.0f}°?\n\n'
                'ตรวจสอบเส้นทางว่าปลอดภัยก่อนยืนยัน (ISO 3691-4)'):
            return
        if self.ros.send_goal(x, y, theta):
            self._nav_goal = (x, y, theta)

    def _on_nav_cancel(self):
        self.ros.cancel_goal()
        self._nav_goal = None
        if self._mission:
            self._mission = None

    def _on_add_station(self):
        if self._mode != 'navigation' or not self._nav_map_name:
            messagebox.showinfo('ยังไม่พร้อม', 'เริ่ม Navigation (เลือกแผนที่) ก่อน')
            return
        pose = self.ros.pose()
        if pose is None:
            messagebox.showwarning('ไม่พบตำแหน่ง', 'ยังไม่มี TF map → base_footprint (AMCL)')
            return
        existing = load_stations(self._nav_map_name)
        names = {s['name'] for s in existing}
        # Default to the first unused Station_A, Station_B, ... - a fixed
        # 'Station_A' default silently replaced that station with the
        # current pose every time it was accepted (looked like the station
        # "followed the robot").
        default = next((f'Station_{c}' for c in 'ABCDEFGHIJKLMNOPQRSTUVWXYZ'
                        if f'Station_{c}' not in names), f'Station_{len(names) + 1}')
        name = self._show_save_name_dialog(default, title='ตั้งชื่อ Station')
        if not name:
            return
        if name in names and not messagebox.askyesno(
                'ชื่อซ้ำ', f'มี Station "{name}" อยู่แล้ว - ย้ายไปตำแหน่งปัจจุบัน (ทับของเดิม)?'):
            return
        stations = [s for s in existing if s['name'] != name]
        stations.append({'name': name, 'x': round(pose[0], 3), 'y': round(pose[1], 3),
                         'theta': round(pose[2], 3)})
        save_stations(self._nav_map_name, stations)
        self._refresh_stations()  # Auto page buttons + pins pick it up now
        messagebox.showinfo('บันทึกแล้ว', f'Station "{name}" ({pose[0]:.2f}, {pose[1]:.2f})')

    def _on_del_station(self):
        if not self._nav_map_name:
            return
        stations = load_stations(self._nav_map_name)
        if not stations:
            messagebox.showinfo('ไม่มี Station', 'แผนที่นี้ยังไม่มี Station')
            return
        names = [s['name'] for s in stations]
        pick = self._show_list_picker('ลบ Station', names)
        if pick is None:
            return
        if messagebox.askyesno('ลบ Station', f'ลบ "{names[pick]}"?'):
            del stations[pick]
            save_stations(self._nav_map_name, stations)
            self._refresh_stations()

    def _render_log(self):
        warn = self._log_warn_only.get()
        lines = [(l, t) for l, t in list(self.ros.log) if not warn or l in 'WEF']
        self.log_text.config(state='normal')
        self.log_text.delete('1.0', 'end')
        for lvl, text in lines[-150:]:
            self.log_text.insert('end', text + '\n', lvl)
        self.log_text.see('end')
        self.log_text.config(state='disabled')
        self._log_seq_shown = self.ros.log_seq

    # ---------- Settings ----------
    def _build_settings(self, p):
        nb = ttk.Notebook(p, style='Dark.TNotebook')
        nb.pack(fill='both', expand=True)
        gen = tk.Frame(nb, bg=SURFACE)
        par = tk.Frame(nb, bg=SURFACE)
        nb.add(gen, text='General')
        nb.add(par, text='Parameters')

        def kv(parent, k, v=''):
            row = tk.Frame(parent, bg=SURFACE)
            row.pack(fill='x', padx=px(16), pady=px(3))
            self._label(row, k, width=26, anchor='w', fg=TEXT_MUTED).pack(side='left')
            lbl = self._label(row, v, anchor='w')
            lbl.pack(side='left', fill='x')
            return lbl

        self._section_title(gen, 'System').pack(fill='x', padx=px(16), pady=(px(12), px(2)))
        self.set_time = kv(gen, 'System Time')
        self.set_ip = kv(gen, 'IP Address')
        kv(gen, 'ROS_DOMAIN_ID', os.environ.get('ROS_DOMAIN_ID', '0 (default)'))
        kv(gen, 'RMW', os.environ.get('RMW_IMPLEMENTATION', 'default'))
        self.set_git = kv(gen, 'Software version (git)', git_describe())
        kv(gen, 'Language / Brightness', 'เปลี่ยนภาษาที่หน้า Home · ความสว่างปรับที่ OS')
        self._section_title(gen, 'Calibration').pack(fill='x', padx=px(16), pady=(px(14), px(2)))
        row = tk.Frame(gen, bg=SURFACE)
        row.pack(fill='x', padx=px(16))
        self.btn_calibrate = self._btn(row, 'ปรับเทียบ Localization / Wheel (Calibrate)',
                                       self._on_click_calibrate, kind='info')
        self.btn_calibrate.pack(side='left')

        self._section_title(gen, 'Security').pack(fill='x', padx=px(16), pady=(px(14), px(2)))
        row = tk.Frame(gen, bg=SURFACE)
        row.pack(fill='x', padx=px(16))
        self._btn(row, 'เปลี่ยน PIN โหมดนักพัฒนา', self._on_change_pin).pack(side='left')

        # Editable, bounded copies of the safety-related Nav2 values (see
        # NAV_PARAM_SPECS). Save only rewrites nav2_params.yaml - the git
        # tree goes "-dirty" and the change still needs commit + test + tag
        # before line use (CLAUDE.md).
        self._build_param_editor(par)

    def _build_param_editor(self, outer):
        # the stepper rows outgrow the 7" screen: scroll the whole tab
        # (wide scrollbar for touch, wheel/touchpad for mouse)
        sb = tk.Scrollbar(outer, width=px(28))
        sb.pack(side='right', fill='y')
        cv = tk.Canvas(outer, bg=SURFACE, highlightthickness=0, yscrollcommand=sb.set)
        cv.pack(side='left', fill='both', expand=True)
        sb.config(command=cv.yview)
        par = tk.Frame(cv, bg=SURFACE)
        win = cv.create_window(0, 0, window=par, anchor='nw')
        par.bind('<Configure>', lambda _e: cv.configure(scrollregion=cv.bbox('all')))
        cv.bind('<Configure>', lambda e: cv.itemconfigure(win, width=e.width))

        # The last visible row is usually cut by the screen edge, which read
        # as "missing letters". Cover it with a scroll hint while more rows
        # lie below; hidden once scrolled to the end.
        more = tk.Label(outer, text='▼  เลื่อนลงเพื่อดูค่าเพิ่มเติม', font=self.f_small,
                        bg=SURFACE_2, fg=TEXT_MAIN, pady=px(6))

        def on_scroll(first, last):
            sb.set(first, last)
            if float(last) < 0.999:
                more.place(relx=0, rely=1, anchor='sw', relwidth=1, width=-px(28))
                more.lift()
            else:
                more.place_forget()
        cv.configure(yscrollcommand=on_scroll)

        # bound on 'all' (plus the class bindings, which still run) and
        # filtered to widgets inside this tab, so buttons/labels scroll too
        inside = str(outer) + '.'

        def mine(e):
            return isinstance(e.widget, tk.Misc) and str(e.widget).startswith(inside) \
                and e.widget is not sb

        def wheel(e):
            if mine(e):
                cv.yview_scroll(-1 if (e.num == 4 or getattr(e, 'delta', 0) > 0) else 1, 'units')
        for seq in ('<Button-4>', '<Button-5>', '<MouseWheel>'):
            cv.bind_all(seq, wheel, add='+')

        # finger drag scrolls like a phone. Past DRAG_PX it is a drag, not a
        # tap: the pressed tk.Button's command is unhooked until after its
        # release (class binding runs before 'all'), so no accidental +/-
        # or Save while scrolling.
        DRAG_PX = px(12)
        drag = {'y': None, 'top': 0.0, 'on': False}

        def press(e):
            drag.update(y=e.y_root, top=cv.yview()[0], on=False, w=e.widget) if mine(e) \
                else drag.update(y=None)

        def motion(e):
            if drag['y'] is None:
                return
            dy = e.y_root - drag['y']
            if not drag['on'] and abs(dy) < DRAG_PX:
                return
            if not drag['on']:
                drag['on'] = True
                w = drag['w']
                if isinstance(w, tk.Button):
                    drag['cmd'] = (w, w.cget('command'))
                    w.config(command='')
            total = cv.bbox('all')[3] if cv.bbox('all') else 1
            cv.yview_moveto(drag['top'] - dy / max(total, 1))

        def release(_e):
            drag['y'] = None
            held = drag.pop('cmd', None)
            if held:
                held[0].after_idle(lambda: held[0].config(command=held[1]))
        cv.bind_all('<ButtonPress-1>', press, add='+')
        cv.bind_all('<B1-Motion>', motion, add='+')
        cv.bind_all('<ButtonRelease-1>', release, add='+')

        par.columnconfigure(0, weight=3)
        par.columnconfigure(1, weight=2)
        left = tk.Frame(par, bg=SURFACE)
        left.grid(row=0, column=0, sticky='nsew', padx=px(16), pady=px(10))
        right = tk.Frame(par, bg=SURFACE)
        right.grid(row=0, column=1, sticky='nsew', padx=(0, px(16)), pady=px(10))

        self._param_file = {}
        self._param_edit = {}
        self._param_widgets = {}
        for r, (key, label, unit, lo, hi, step, fmt) in enumerate(NAV_PARAM_SPECS):
            self._label(left, label, anchor='w', width=20).grid(row=r, column=0, sticky='w')
            row, minus, val, plus = self._stepper(
                left, '', '', lambda k=key, st=step: self._adjust_param(k, -st),
                lambda k=key, st=step: self._adjust_param(k, st), width=6, big=True)
            row.grid(row=r, column=1, sticky='w', pady=px(4))
            rng = self._label(left, f'{unit}  ({fmt.format(lo)}–{fmt.format(hi)})',
                              font=self.f_small, fg=TEXT_MUTED, anchor='w')
            rng.grid(row=r, column=2, sticky='w', padx=px(8))
            self._param_widgets[key] = (minus, val, plus)

        self.param_note = self._label(right, '', font=self.f_small, fg=TEXT_MUTED,
                                      justify='left', anchor='nw', wraplength=px(330))
        self.param_note.pack(fill='both', expand=True)
        right.bind('<Configure>', lambda e: self.param_note.config(
            wraplength=max(px(200), e.width - px(8))), add='+')
        btns = tk.Frame(right, bg=SURFACE)
        btns.pack(fill='x', side='bottom')
        self.btn_param_save = self._btn(btns, 'บันทึกลงไฟล์', self._on_param_save, kind='warn')
        self.btn_param_save.pack(side='left', fill='x', expand=True, padx=(0, px(4)))
        self._btn(btns, 'คืนค่าจากไฟล์', self._load_params).pack(side='left')
        self._btn(btns, 'ค่า Default', self._default_params).pack(side='left', padx=(px(4), 0))
        self._load_params()

    def _load_params(self):
        try:
            with open(NAV2_PARAMS) as f:
                self._param_file = read_nav_params(f.read())
            self._param_error = None
        except (OSError, ValueError) as e:
            self._param_file = {}
            self._param_error = str(e)
        self._param_edit = dict(self._param_file)
        self._refresh_params()

    def _default_params(self):
        if self._param_error:
            return
        self._param_edit = dict(NAV_PARAM_DEFAULTS)
        self._refresh_params()

    def _adjust_param(self, key, delta):
        spec = next(sp for sp in NAV_PARAM_SPECS if sp[0] == key)
        _, _, _, lo, hi, _, _ = spec
        if key in self._param_edit:
            self._param_edit[key] = round(max(lo, min(hi, self._param_edit[key] + delta)), 3)
        self._refresh_params()

    def _refresh_params(self):
        ok = not self._param_error
        changed = []
        for key, label, unit, lo, hi, step, fmt in NAV_PARAM_SPECS:
            minus, val, plus = self._param_widgets[key]
            v = self._param_edit.get(key)
            if v is None:
                val.config(text='—', fg=TEXT_MUTED)
                self._set_btn_state(minus, False)
                self._set_btn_state(plus, False)
                continue
            diff = abs(v - self._param_file[key]) > 1e-6
            if diff:
                changed.append(f'{T(label)}: {fmt.format(self._param_file[key])} → {fmt.format(v)} {unit}')
            val.config(text=fmt.format(v), fg=AMBER if diff else TEXT_MAIN)
            self._set_btn_state(minus, v > lo + 1e-9)
            self._set_btn_state(plus, v < hi - 1e-9)
        self._set_btn_state(self.btn_param_save, ok and bool(changed))
        self._param_changes = changed

        lines = []
        if not ok:
            lines.append(T('อ่านไฟล์ไม่ได้ - แก้ผ่านไฟล์เท่านั้น') + f'\n{self._param_error}')
        e = self._param_edit
        if ok and e:
            corner = max(math.hypot(x, y) for x in (e['fp_front'], e['fp_rear'])
                         for y in (e['fp_left'], e['fp_right']))
            lines.append(T('รัศมีมุมหุ่น') + f' {corner:.2f} m')
            if e['inflation'] < corner:
                lines.append(T('⚠ inflation น้อยกว่ารัศมีมุมหุ่น - หุ่นอาจวิ่งเฉียดสิ่งกีดขวางตอนหมุน'))
            lines.append(T('FrontStop เริ่มที่หน้ากันชน') + f' {e["fp_front"]:.2f} m '
                         + T('ลึก') + f' {FRONT_STOP_DEPTH:.2f} m')
            lines.append(T('Footprint = ระยะจากกลางรถ (กลางล้อขับ) ถึงขอบนอกสุด '
                           'ของรถหรือ payload ที่ยื่นมากกว่า - ไม่ใช่ระยะที่ยื่นออกจากตัวรถ'))
            lines.append(T('Footprint รวม') + f' {e["fp_front"] + e["fp_rear"]:.2f} x '
                         f'{e["fp_left"] + e["fp_right"]:.2f} m')
            b = laser_box_for(e)
            lines.append(T('กล่องกรอง LiDAR (self-hit)') + f' x {b["min_x"]:.2f}..{b["max_x"]:.2f}'
                         + f' y {b["min_y"]:.2f}..{b["max_y"]:.2f} m')
            d_stop = e['v_max'] ** 2 / (2 * e['dec_lin']) + e['v_max'] * STOP_LATENCY_S
            lines.append(T('ระยะหยุดโดยประมาณที่ v_max') + f' {d_stop:.2f} m '
                         + T('(รวม latency') + f' {STOP_LATENCY_S:.1f} s)')
            if d_stop > FRONT_STOP_DEPTH * 0.8:
                lines.append(T('⚠ ระยะหยุดเกิน 80% ของความลึก FrontStop - เพิ่ม decel หรือลด v_max'))
            if e['acc_lin'] > e['dec_lin']:
                lines.append(T('⚠ accel มากกว่า decel - ควรให้ decel ≥ accel'))
            lines.append(T('accel/decel ใช้กับ Navigation (velocity_smoother) เท่านั้น - '
                           'teleop/joystick ส่ง /cmd_vel ตรง ใช้ ramp ของ STM32 (0.5 / 1.2)'))
        if changed:
            lines.append(T('ค่าที่จะเปลี่ยน:') + '\n' + '\n'.join(changed))
        lines.append(T('มีผลเมื่อเริ่ม Navigation/Mapping ครั้งถัดไป · '
                       'ต้อง commit + ทดสอบ + tag ก่อนใช้บนไลน์'))
        self.param_note.config(text='\n\n'.join(lines))

    def _on_param_save(self):
        if not self._param_changes:
            return
        if not messagebox.askyesno(
                'บันทึก Parameter',
                'ค่าที่จะเปลี่ยน:\n' + '\n'.join(self._param_changes) + '\n\n'
                'ค่าเหล่านี้เกี่ยวกับ Safety (ISO 3691-4) - หลังบันทึกต้องทดสอบระยะหยุด/'
                'การหลบสิ่งกีดขวางบนหุ่นจริง แล้ว commit + tag ก่อนใช้บนไลน์\n\nบันทึก?'):
            return
        try:
            with open(NAV2_PARAMS) as f:
                text = f.read()
            new = write_nav_params(text, self._param_edit)
            if read_nav_params(new) != {k: round(v, 2) for k, v in self._param_edit.items()}:
                raise ValueError('verify after write failed')
            with open(LASER_FILTER) as f:
                box_new = write_laser_box(f.read(), self._param_edit)
            if read_laser_box(box_new) != laser_box_for(self._param_edit):
                raise ValueError('laser_filter.yaml verify after write failed')
            # both verified before either is replaced, so the footprint and
            # the self-hit box can't end up out of sync
            for path, body in ((NAV2_PARAMS, new), (LASER_FILTER, box_new)):
                with open(path + '.tmp', 'w') as f:
                    f.write(body)
                os.replace(path + '.tmp', path)
        except (OSError, ValueError) as e:
            messagebox.showerror('บันทึกไม่สำเร็จ', str(e))
            return
        self._load_params()
        self.set_git.config(text=git_describe())
        messagebox.showinfo(
            'บันทึกแล้ว',
            'เขียน nav2_params.yaml + laser_filter.yaml แล้ว (git tree เป็น -dirty)\n'
            + ('ระบบกำลังทำงาน - ต้องปิดแล้วเริ่มใหม่เพื่อใช้ค่าใหม่'
               if self._proc is not None else 'ค่าใหม่มีผลเมื่อเริ่มระบบครั้งถัดไป'))

    # ---------- periodic UI refresh ----------
    def _update_hook_pill(self):
        st = self.ros.hook_state if self.ros.hook_alive() else None
        if not st:
            self.pill_hook.config(text='Hook —', bg=SURFACE_2, fg=TEXT_MUTED)
        elif st.startswith('FAULT') or st == 'UNKNOWN':
            self.pill_hook.config(text=f'Hook {st}', bg=RED, fg='white')
        elif st.startswith('MOVING'):
            arrow = '▲' if st == 'MOVING_UP' else '▼'
            self.pill_hook.config(text=f'Hook {arrow} moving', bg=AMBER_DARK, fg='white')
        elif st in ('UP', 'DOWN'):
            arrow = '▲' if st == 'UP' else '▼'
            self.pill_hook.config(text=f'Hook {arrow} {st}', bg=BLUE_DARK, fg='white')
        else:  # IDLE: between the limits
            self.pill_hook.config(text='Hook mid', bg=AMBER_DARK, fg='white')

    def _tick(self):
        try:
            self._tick_inner()
            if _LANG['cur'] == 'en':
                self._i18n_walk()
        finally:
            self.after(250, self._tick)

    def _tick_inner(self):
        self._handle_mapping_cmds()
        if self._mode == 'mapping' and time.monotonic() - self._mapping_state_t > 1.0:
            self.ros.publish_mapping_state(self._mapping_state())
            self._mapping_state_t = time.monotonic()
        if time.monotonic() - self._stations_pub_t > 1.0:
            nav = self._mode == 'navigation' and self._nav_map_name
            # read the file, not self._stations: that is only refreshed
            # when the Auto page is opened
            self.ros.publish_stations(self._nav_map_name if nav else '',
                                      load_stations(self._nav_map_name) if nav else [])
            self._stations_pub_t = time.monotonic()
        ip = get_local_ip() if int(time.time()) % 5 == 0 or self.pill_net.cget('text') == 'Net —' else None
        if ip:
            online = ip != '127.0.0.1'
            self.pill_net.config(text=f'Net {ip}' if online else 'Net offline',
                                 fg=TEXT_MAIN if online else RED)
            self._ip = ip
        wds = [self.ros.watchdog.get(k) for k in ('imu', 'depth')]
        live = [w for w in wds if w and time.monotonic() - w[1] < 3]
        if not live:
            self.pill_safety.config(text='Safety —', bg=SURFACE_2, fg=TEXT_MUTED)
        elif all(w[0] for w in live):
            self.pill_safety.config(text='Safety OK', bg=GREEN_DARK, fg='white')
        else:
            self.pill_safety.config(text='SENSOR FAULT', bg=RED, fg='white')
        self._update_hook_pill()

        page = self._page
        pose = self.ros.pose()
        if page == 'auto':
            self.auto_map.set_map(self.ros.map_msg, self.ros.map_seq)
            markers = [dict(s, color=station_color(i),
                            selected=(s['name'] == self._selected_station))
                       for i, s in enumerate(self._stations)]
            self.auto_map.set_overlay(pose, markers)
            self._update_mission_ui()
        elif page == 'mapping':
            self.mapping_map.set_map(self.ros.map_msg, self.ros.map_seq)
            self.mapping_map.set_overlay(pose)
            self._update_diag()
        elif page == 'navtest':
            self._show_watchdogs(self.nav_wd_labels, ('imu', 'depth'))
            self.nav_map.set_map(self.ros.map_msg, self.ros.map_seq)
            self.nav_map.set_overlay(pose, load_stations(self._nav_map_name)
                                     if self._nav_map_name else (), self._nav_goal)
            if self.ros.log_seq != self._log_seq_shown:
                self._render_log()
        elif page == 'manual':
            self._draw_manual()
        elif page == 'settings':
            self.set_time.config(text=time.strftime('%Y-%m-%d %H:%M:%S'))
            self.set_ip.config(text=getattr(self, '_ip', '-'))

    def _on_software_stop(self):
        self._cancel_move_forward()
        self.ros.software_stop()
        if self._mission and self._mission['state'] == 'active':
            self._mission['state'] = 'paused'
        self._nav_goal = None
        self._set_status('error', 'STOP (software)',
                         'ยกเลิก goal + ส่ง cmd_vel=0 แล้ว · ไม่ใช่ E-stop ระดับ safety')

    # ---------- status helpers ----------
    def _set_status(self, kind, text, detail=''):
        colors = {
            'idle': '#9e9e9e',
            'starting': AMBER,
            'running': GREEN,
            'error': RED,
        }
        self.status_dot.itemconfig(self._dot_id, fill=colors.get(kind, '#9e9e9e'))
        self.status_label.config(text=T(text))
        self.detail_label.config(text=T(detail.replace('\n', ' · ')))
        if detail:
            self.detail_label.pack(anchor='e')
        else:
            self.detail_label.pack_forget()

    def _set_running_ui(self, running, mode=None):
        for b in self._start_buttons:
            self._set_card_enabled(b, not running)
        self._set_btn_state(self.btn_stop, running)
        self._set_btn_state(self.btn_manual_stop, running)
        self._set_btn_state(self.btn_save_map, running and mode == 'mapping')
        self._set_btn_state(self.btn_discard, running and mode == 'mapping')
        is_moving = self._move_proc is not None and self._move_proc.poll() is None
        if not is_moving:
            for b, text in ((self.btn_move_forward, 'เดินหน้า (Move Forward)'),
                            (self.btn_square_path, 'วิ่งเป็นสี่เหลี่ยม (Square Path)'),
                            (self.btn_circle_path, 'วิ่งวงกลม (Circle Path)')):
                self._set_btn_state(b, running)
                b.config(text=T(text))
            for b in (self.btn_move_minus, self.btn_move_plus, self.btn_speed_minus,
                      self.btn_speed_plus, self.btn_square_minus, self.btn_square_plus,
                      self.btn_circle_radius_minus, self.btn_circle_radius_plus,
                      self.btn_circle_angle_minus, self.btn_circle_angle_plus):
                self._set_btn_state(b, running)
            if running:
                self._adjust_move_distance(0)  # re-apply min/max bound greying
                self._adjust_move_speed(0)
                self._adjust_square_side(0)
                self._adjust_circle_radius(0)
                self._adjust_circle_angle(0)

    # ---------- mode launches ----------
    def _on_click_mapping(self):
        if self._proc is not None:
            return
        self._launch([
            'ros2', 'launch', 'amr_bringup', 'bringup.launch.py',
            'rviz:=false', 'foxglove:=true', 'slam:=true', 'navigation:=false',
            'slam_start_scanning:=false',
        ], mode='mapping', label='โหมดสร้างแผนที่ (Mapping)')
        self._slam_ready = False
        self._scanning = False
        self._mapping_msg = 'กำลังเริ่ม SLAM...'
        self.ros.gate_open = None  # ignore the previous run's latched state
        threading.Thread(target=self._ensure_monitors_then_show_qr,
                         daemon=True).start()
        threading.Thread(target=self._wait_scan_gate, daemon=True).start()

    def _wait_scan_gate(self):
        """Mapping is launched with slam_start_scanning:=false, so SLAM gets
        no scans until the operator presses Start. Close the gate again
        explicitly once it is up (idempotent) and mark the UI ready."""
        proc = self._proc
        t0 = time.monotonic()
        while self._proc is proc and proc is not None and time.monotonic() - t0 < 90:
            if self.ros.set_scan_gate(False, timeout=2.0):
                def done():
                    if self._proc is proc:
                        self._slam_ready = True
                        self._scanning = False
                        self._mapping_msg = 'พร้อม - กดเริ่มสแกน'
                        self._refresh_mapping_ui()
                self.after(0, done)
                return
        def fail():
            if self._proc is proc:
                self._mapping_msg = 'ไม่พบ scan gate - เริ่มสแกนไม่ได้'
                self._refresh_mapping_ui()
        self.after(0, fail)

    # ---------- scan start / pause / save / finish (panel + phone) ----------
    def _set_scanning(self, want):
        if self._mode != 'mapping' or not self._slam_ready or want == self._scanning:
            return
        self._slam_ready = False  # block re-entry until the call returns
        self._refresh_mapping_ui()

        def work():
            ok = self.ros.set_scan_gate(want)

            def done():
                self._slam_ready = self._mode == 'mapping'
                if ok:
                    self._scanning = want
                    self._mapping_msg = 'กำลังสแกน' if want else 'หยุดสแกนชั่วคราว'
                else:
                    self._mapping_msg = 'สั่ง scan gate ไม่สำเร็จ'
                self._refresh_mapping_ui()
            self.after(0, done)
        threading.Thread(target=work, daemon=True).start()

    def _on_click_scan(self):
        self._set_scanning(not self._scanning)

    def _handle_mapping_cmds(self):
        while self.ros.ok and self.ros.mapping_cmds:
            try:
                cmd = json.loads(self.ros.mapping_cmds.popleft())
            except ValueError:
                continue
            action = cmd.get('action')
            if self._mode != 'mapping':
                continue
            if action == 'start':
                self._set_scanning(True)
            elif action == 'pause':
                self._set_scanning(False)
            elif action == 'save':
                name = sanitize_map_name(str(cmd.get('name') or '')) or \
                    time.strftime('map_%Y%m%d_%H%M%S')
                if os.path.exists(os.path.join(MAPS_DIR, name + '.yaml')):
                    self._mapping_msg = f'มีแผนที่ชื่อ "{name}" อยู่แล้ว - ใช้ชื่ออื่น'
                else:
                    self._start_map_save(name, from_phone=True)
            elif action == 'finish':
                self._mapping_msg = 'ปิด Mapping จากมือถือ'
                self._on_stop()
            self._refresh_mapping_ui()

    def _mapping_state(self):
        if self._mode == 'mapping' and self._slam_ready and self.ros.gate_open is not None:
            self._scanning = self.ros.gate_open
        return {'mode': self._mode or 'idle', 'ready': self._slam_ready,
                'scanning': self._scanning, 'saving': self._map_saving,
                'msg': self._mapping_msg}

    def _refresh_mapping_ui(self):
        mapping = self._mode == 'mapping'
        self._set_btn_state(self.btn_scan, mapping and self._slam_ready)
        self.btn_scan.config(text=T('หยุดสแกนชั่วคราว' if self._scanning else 'เริ่มสแกน'))
        if mapping:
            self.mapping_banner.config(
                text=T(('● กำลังสแกน · ' if self._scanning else '❚❚ ไม่ได้สแกน · ')
                       + self._mapping_msg),
                bg=GREEN if self._scanning else AMBER,
                fg='white' if self._scanning else '#1a1a1a')
        else:
            self.mapping_banner.config(
                text=T('ใช้ Joystick (ฮาร์ดแวร์/มือถือ) ขับหุ่นยนต์ช้าๆ ให้ทั่วพื้นที่'),
                bg=AMBER, fg='#1a1a1a')
        self.ros.publish_mapping_state(self._mapping_state())
        self._mapping_state_t = time.monotonic()

    # ---------- rosboard / Lichtblick / mobile monitor QR ----------
    def _ensure_monitors_then_show_qr(self):
        ip = get_local_ip()

        if not is_port_open(ip, ROSBOARD_PORT):
            self._start_rosboard()
            for _ in range(20):
                time.sleep(0.5)
                if is_port_open(ip, ROSBOARD_PORT):
                    break
        rosboard_ok = is_port_open(ip, ROSBOARD_PORT)
        rosboard_url = build_rosboard_url(ip, MAPPING_MONITOR_TOPICS)

        lichtblick_ok = False
        if os.path.isdir(LICHTBLICK_DIST_DIR):
            if not is_port_open(ip, LICHTBLICK_PORT):
                self._start_static_server(
                    '_lichtblick_proc', LICHTBLICK_PORT, LICHTBLICK_DIST_DIR)
                for _ in range(20):
                    time.sleep(0.5)
                    if is_port_open(ip, LICHTBLICK_PORT):
                        break
            lichtblick_ok = is_port_open(ip, LICHTBLICK_PORT)

        mobile_url = None
        if lichtblick_ok and rosboard_ok and os.path.isdir(MOBILE_DIST_DIR):
            if not is_port_open(ip, MOBILE_PORT):
                self._start_static_server(
                    '_mobile_proc', MOBILE_PORT, MOBILE_DIST_DIR)
                for _ in range(20):
                    time.sleep(0.5)
                    if is_port_open(ip, MOBILE_PORT):
                        break
            if is_port_open(ip, MOBILE_PORT):
                mobile_url = build_mobile_url(ip)

        self.after(0, lambda: self._show_monitor_qr(mobile_url, rosboard_url))

    def _start_rosboard(self):
        if self._rosboard_proc is not None and self._rosboard_proc.poll() is None:
            return
        env = os.environ.copy()
        try:
            self._rosboard_proc = subprocess.Popen(
                ['ros2', 'run', 'rosboard', 'rosboard_node'],
                preexec_fn=os.setsid, env=env,
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        except Exception:
            self._rosboard_proc = None

    def _start_static_server(self, proc_attr, port, directory):
        proc = getattr(self, proc_attr)
        if proc is not None and proc.poll() is None:
            return
        env = os.environ.copy()
        try:
            proc = subprocess.Popen(
                ['python3', '-m', 'http.server', str(port),
                 '--directory', directory, '--bind', '0.0.0.0'],
                preexec_fn=os.setsid, env=env,
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        except Exception:
            proc = None
        setattr(self, proc_attr, proc)

    def _add_qr_block(self, parent, title, url, extra_note=None):
        img = qrcode.make(url).resize((px(220), px(220)))
        photo = ImageTk.PhotoImage(img)
        self._qr_photos.append(photo)  # keep a reference so Tk doesn't GC it

        block = tk.Frame(parent, bg=SURFACE)
        self._label(block, title, wraplength=px(260), justify='center').pack(pady=(px(0), px(8)))
        tk.Label(block, image=photo, bg='white').pack()
        self._label(block, url, font=self.f_small, fg=TEXT_MUTED, wraplength=px(260),
                    justify='center').pack(pady=(px(8), px(4)))
        if extra_note:
            self._label(block, extra_note, font=self.f_small, fg=TEXT_MUTED,
                        wraplength=px(260), justify='center').pack(pady=(px(0), px(4)))
        return block

    def _qr_window(self, title, heading, subheading):
        self._qr_photos = []
        win = tk.Toplevel(self)
        win.title(title)
        win.configure(bg=SURFACE)
        win.transient(self)
        win.resizable(False, False)
        self._label(win, heading, wraplength=px(560), justify='center').pack(padx=px(20), pady=(px(18), px(4)))
        self._label(win, subheading, font=self.f_small, fg=TEXT_MUTED, wraplength=px(560),
                    justify='center').pack(padx=px(20), pady=(px(0), px(12)))
        cols = tk.Frame(win, bg=SURFACE)
        cols.pack(padx=px(20), fill='x')
        return win, cols

    def _finish_qr_window(self, win, pin_title):
        pin = get_rosboard_pin()
        if pin:
            pin_frame = tk.Frame(win, bg=SURFACE_2, highlightthickness=1,
                                 highlightbackground=AMBER)
            pin_frame.pack(padx=px(20), pady=(px(14), px(4)), fill='x')
            tk.Label(pin_frame, text=pin_title, font=self.f_small,
                     bg=SURFACE_2, fg=TEXT_MUTED).pack(pady=(px(8), px(0)))
            tk.Label(pin_frame, text=pin, font=self.f_huge,
                     bg=SURFACE_2, fg=AMBER).pack(pady=(px(0), px(8)))
        self._btn(win, 'ปิด', win.destroy, padx=px(30)).pack(pady=(px(10), px(18)))
        win.update_idletasks()
        self._center_toplevel(win, win.winfo_reqwidth() + 10,
                              win.winfo_reqheight() + 10)

    def _show_monitor_qr(self, mobile_url, rosboard_url):
        win, cols = self._qr_window(
            'Monitor บนมือถือ', 'สแกนเพื่อดูแผนที่ + ควบคุมขับจากมือถือ',
            'ต่อ WiFi วงเดียวกับ Jetson นี้ก่อนสแกน (ไม่ต้องใช้อินเทอร์เน็ต)')
        if mobile_url:
            mobile_block = self._add_qr_block(
                cols, 'แผนที่ + ควบคุม (แนะนำ)', mobile_url,
                extra_note='แผนที่แบบ Foxglove (Lichtblick) ด้านบน + จอย'
                           'สติ๊กควบคุมขับ (rosboard 🕹) ด้านล่าง - '
                           'โหลดครั้งแรกอาจช้าสักครู่ (~170MB ผ่าน LAN)')
            mobile_block.grid(row=0, column=0, padx=(px(0), px(16)), sticky='n')
        else:
            self._label(cols, 'หน้ารวม (แผนที่+ควบคุม) ยังไม่พร้อมใช้งาน\n'
                              f'(ต้องมีทั้ง rosboard และ {LICHTBLICK_DIST_DIR})',
                        font=self.f_small, fg=RED, wraplength=px(220), justify='center'
                        ).grid(row=0, column=0, padx=(px(0), px(16)), sticky='n')
        rosboard_block = self._add_qr_block(
            cols, 'Monitor + ควบคุม เบา (rosboard เท่านั้น)', rosboard_url)
        rosboard_block.grid(row=0, column=1, padx=(px(16), px(0)), sticky='n')
        self._finish_qr_window(win, 'รหัสเข้าใช้งาน rosboard (PIN)')

    def _on_click_navigation(self):
        if self._proc is not None:
            return
        if not glob.glob(os.path.join(MAPS_DIR, '*.yaml')):
            messagebox.showwarning(
                'ไม่พบแผนที่',
                'ยังไม่มีไฟล์แผนที่ที่บันทึกไว้ กรุณาสร้างแผนที่ก่อน (โหมด Mapping)')
            return
        map_path = self._show_map_picker()
        if map_path is None:
            return
        self._nav_map_name = os.path.splitext(os.path.basename(map_path))[0]
        self._selected_station = None
        self._mission = None
        self._launch([
            'ros2', 'launch', 'amr_bringup', 'bringup.launch.py',
            'rviz:=false', 'foxglove:=true', 'slam:=false', 'navigation:=true',
            f'map:={map_path}',
        ], mode='navigation',
            label=f'โหมดนำทาง (Navigation)\nแผนที่: {os.path.basename(map_path)}')
        if self._page == 'auto':
            self._refresh_stations()
        else:
            threading.Thread(target=self._ensure_nav_monitors_then_show_qr,
                             daemon=True).start()

    # ---------- goal_picker QR (Navigation mode: tap/draw a destination) ----------
    def _ensure_nav_monitors_then_show_qr(self):
        ip = get_local_ip()

        if not is_port_open(ip, ROSBOARD_PORT):
            self._start_rosboard()
            for _ in range(20):
                time.sleep(0.5)
                if is_port_open(ip, ROSBOARD_PORT):
                    break
        rosboard_ok = is_port_open(ip, ROSBOARD_PORT)

        goal_picker_url = build_goal_picker_url(ip) if rosboard_ok else None
        rosboard_url = build_rosboard_url(ip, MAPPING_MONITOR_TOPICS) if rosboard_ok else None

        self.after(0, lambda: self._show_navigation_qr(goal_picker_url, rosboard_url))

    def _show_navigation_qr(self, goal_picker_url, rosboard_url):
        win, cols = self._qr_window(
            'ตั้งจุดหมายจากมือถือ', 'สแกนเพื่อเลือกจุดหมาย/วาดเส้นทางให้หุ่นยนต์วิ่งตาม',
            'ต่อ WiFi วงเดียวกับ Jetson นี้ก่อนสแกน (ไม่ต้องใช้อินเทอร์เน็ต)\n'
            'หุ่นยนต์จะเคลื่อนที่แบบอัตโนมัติทันทีที่ยืนยันบนหน้าจอนั้น - '
            'ตรวจสอบพื้นที่ให้ปลอดภัยก่อนเสมอ (ISO 3691-4)')
        if goal_picker_url:
            goal_block = self._add_qr_block(
                cols, 'ตั้งจุดหมาย / ลากเส้นทาง (แนะนำ)', goal_picker_url,
                extra_note='แตะแผนที่เพื่อส่งหุ่นยนต์ไปจุดนั้น หรือลากเพื่อวาดเส้นทาง')
            goal_block.grid(row=0, column=0, padx=(px(0), px(16)), sticky='n')
        else:
            self._label(cols, 'หน้าตั้งจุดหมายยังไม่พร้อมใช้งาน (rosboard ไม่ทำงาน)',
                        font=self.f_small, fg=RED, wraplength=px(220), justify='center'
                        ).grid(row=0, column=0, padx=(px(0), px(16)), sticky='n')
        if rosboard_url:
            rosboard_block = self._add_qr_block(
                cols, 'Monitor แผนที่ (rosboard)', rosboard_url)
            rosboard_block.grid(row=0, column=1, padx=(px(16), px(0)), sticky='n')
        self._finish_qr_window(win, 'รหัสเข้าใช้งาน (PIN)')

    def _center_toplevel(self, win, w, h):
        self._i18n_walk(win)
        self.update_idletasks()
        x = self.winfo_x() + (self.winfo_width() - w) // 2
        y = self.winfo_y() + (self.winfo_height() - h) // 2
        win.geometry(f'{w}x{h}+{max(x, 0)}+{max(y, 0)}')

    def _show_list_picker(self, title, names, latest_tag=False):
        """Modal touch-friendly list; returns the picked index or None."""
        result = {'idx': None}
        win = tk.Toplevel(self)
        win.title(title)
        win.configure(bg=SURFACE)
        win.transient(self)
        win.resizable(False, False)

        self._label(win, title, font=self.f_button).pack(padx=px(20), pady=(px(18), px(10)), anchor='w')

        list_frame = tk.Frame(win, bg=SURFACE)
        list_frame.pack(padx=px(20), fill='both', expand=True)
        scrollbar = tk.Scrollbar(list_frame, width=px(28))
        scrollbar.pack(side='right', fill='y')
        lb = tk.Listbox(list_frame, font=self.f_button_sub, height=6,
                        activestyle='none', bg=SURFACE_2, fg=TEXT_MAIN,
                        selectbackground=BLUE, selectforeground='white',
                        relief='flat', highlightthickness=0,
                        yscrollcommand=scrollbar.set, exportselection=False)
        lb.pack(side='left', fill='both', expand=True)
        scrollbar.config(command=lb.yview)
        for i, n in enumerate(names):
            lb.insert('end', ('  ' + n + T('   (ล่าสุด)')) if (latest_tag and i == 0) else '  ' + n)
        lb.selection_set(0)
        lb.activate(0)

        btn_row = tk.Frame(win, bg=SURFACE)
        btn_row.pack(fill='x', padx=px(20), pady=px(16))

        def confirm(_e=None):
            sel = lb.curselection()
            if sel:
                result['idx'] = sel[0]
            win.destroy()

        def cancel(_e=None):
            win.destroy()

        self._btn(btn_row, 'ยกเลิก', cancel).pack(side='right')
        self._btn(btn_row, 'เลือก', confirm, kind='info', padx=px(24)
                  ).pack(side='right', padx=(px(0), px(8)))

        lb.bind('<Double-Button-1>', confirm)
        win.bind('<Return>', confirm)
        win.bind('<Escape>', cancel)
        win.protocol('WM_DELETE_WINDOW', cancel)

        self._center_toplevel(win, px(480), px(380))
        win.grab_set()
        lb.focus_set()
        win.wait_window()
        return result['idx']

    def _show_map_picker(self, title='เลือกแผนที่สำหรับนำทาง', ok_text='เลือกแผนที่นี้'):
        yamls = sorted(glob.glob(os.path.join(MAPS_DIR, '*.yaml')),
                       key=os.path.getmtime, reverse=True)
        yamls = [p for p in yamls if not p.endswith('_stations.yaml')]
        names = [os.path.splitext(os.path.basename(p))[0] for p in yamls]

        result = {'path': None}
        win = tk.Toplevel(self)
        win.title('Map')
        win.configure(bg=SURFACE)
        win.transient(self)
        win.resizable(False, False)
        self._label(win, title, font=self.f_button
                    ).pack(padx=px(20), pady=(px(16), px(8)), anchor='w')

        body = tk.Frame(win, bg=SURFACE)
        body.pack(padx=px(20), fill='both', expand=True)
        list_frame = tk.Frame(body, bg=SURFACE)
        list_frame.pack(side='left', fill='y')
        scrollbar = tk.Scrollbar(list_frame, width=px(28))
        scrollbar.pack(side='right', fill='y')
        lb = tk.Listbox(list_frame, font=self.f_button, height=8, width=28,
                        activestyle='none', bg=SURFACE_2, fg=TEXT_MAIN,
                        selectbackground=BLUE, selectforeground='white',
                        relief='flat', highlightthickness=0,
                        yscrollcommand=scrollbar.set, exportselection=False)
        lb.pack(side='left', fill='y')
        scrollbar.config(command=lb.yview)
        for i, n in enumerate(names):
            lb.insert('end', '  ' + n + (T('   (ล่าสุด)') if i == 0 else ''))

        right = tk.Frame(body, bg=SURFACE)
        right.pack(side='left', fill='both', expand=True, padx=(px(12), 0))
        pw, ph = px(420), px(300)
        canvas = tk.Canvas(right, width=pw, height=ph, bg='#1c222a',
                           highlightthickness=1, highlightbackground=BORDER)
        canvas.pack()
        info = self._label(right, '', font=self.f_small, fg=TEXT_MUTED,
                           justify='left', anchor='w')
        info.pack(fill='x', pady=(px(6), 0))
        photo = {}

        def show(_e=None):
            sel = lb.curselection()
            if not sel:
                return
            path = yamls[sel[0]]
            canvas.delete('all')
            try:
                meta, img = load_map_image(path)
            except Exception as e:  # noqa: BLE001 - corrupt/missing pgm
                canvas.create_text(pw // 2, ph // 2, text=f'preview error: {e}',
                                   fill=RED, width=pw - 20)
                info.config(text='')
                return
            w, h = img.size
            sc = min(pw / w, ph / h)
            dw, dh = max(int(w * sc), 1), max(int(h * sc), 1)
            ox, oy = (pw - dw) // 2, (ph - dh) // 2
            photo['p'] = ImageTk.PhotoImage(img.resize((dw, dh), Image.NEAREST))
            canvas.create_image(ox, oy, anchor='nw', image=photo['p'])
            res, (mx, my) = meta['resolution'], meta['origin'][:2]
            name = os.path.splitext(os.path.basename(path))[0]
            stations = load_stations(name)
            r = px(5)
            for st in stations:
                cx = ox + (st['x'] - mx) / res * sc
                cy = oy + (h - (st['y'] - my) / res) * sc
                canvas.create_rectangle(cx - r, cy - r, cx + r, cy + r,
                                        fill=BLUE, outline='white')
                canvas.create_text(cx + r + 3, cy, anchor='w', text=st['name'],
                                   fill=TEXT_MAIN, font=self.f_small)
            info.config(text=T(
                f'ขนาด {w * res:.1f} x {h * res:.1f} ม. · {res:.3f} ม./px · '
                f'Station {len(stations)} จุด\n'
                f'แก้ไขล่าสุด {time.strftime("%Y-%m-%d %H:%M", time.localtime(os.path.getmtime(path)))}'))

        btn_row = tk.Frame(win, bg=SURFACE)
        btn_row.pack(fill='x', padx=px(20), pady=px(16))

        def confirm(_e=None):
            sel = lb.curselection()
            if sel:
                result['path'] = yamls[sel[0]]
            win.destroy()

        self._btn(btn_row, 'ยกเลิก', win.destroy).pack(side='right')
        self._btn(btn_row, ok_text, confirm, kind='info', padx=px(24)
                  ).pack(side='right', padx=(0, px(8)))

        lb.bind('<<ListboxSelect>>', show)
        lb.bind('<Double-Button-1>', confirm)
        win.bind('<Return>', confirm)
        win.bind('<Escape>', lambda _e: win.destroy())
        lb.selection_set(0)
        lb.activate(0)
        show()
        win.update_idletasks()
        self._center_toplevel(win, win.winfo_reqwidth() + 10, win.winfo_reqheight() + 10)
        win.grab_set()
        lb.focus_set()
        win.wait_window()
        return result['path']

    def _launch(self, cmd, mode, label):
        self._mode = mode
        self._set_status('starting', 'กำลังเริ่มระบบ...', label)
        self._set_running_ui(True, mode)
        env = os.environ.copy()
        self._proc = subprocess.Popen(
            cmd, preexec_fn=os.setsid, env=env,
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        self._launch_label = label

        def watch():
            time.sleep(4)
            if self._proc is not None and self._proc.poll() is None:
                self.after(0, lambda: self._set_status(
                    'running', 'กำลังทำงาน', self._launch_label))
        threading.Thread(target=watch, daemon=True).start()
        self._poll_process()

    def _poll_process(self):
        if self._proc is not None:
            ret = self._proc.poll()
            if ret is not None:
                self._on_process_exited(ret)
                return
        self._status_poll_job = self.after(1000, self._poll_process)

    def _on_process_exited(self, returncode):
        was_running = self._proc is not None
        self._proc = None
        self._mode = None
        self._mission = None
        self._nav_goal = None
        self._slam_ready = False
        self._scanning = False
        self._map_saving = False
        self._set_running_ui(False)
        self._refresh_mapping_ui()  # also tells the phone mode=idle
        if was_running and returncode not in (0, -2, -15):
            self._set_status('error', 'ระบบหยุดทำงานผิดปกติ',
                             f'exit code: {returncode}')
        else:
            self._set_status('idle', 'พร้อมใช้งาน')

    def _on_stop(self):
        if self._proc is None:
            return
        self._cancel_move_forward()  # abort any in-progress "move forward" first
        self._set_status('starting', 'กำลังปิดระบบ...')
        proc = self._proc

        def do_stop():
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGINT)
                proc.wait(timeout=8)
            except Exception:
                try:
                    os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
                except Exception:
                    pass
            self.after(0, lambda: self._on_process_exited(0))
        threading.Thread(target=do_stop, daemon=True).start()

    # ---------- move-forward-by-distance ----------
    def _adjust_move_distance(self, delta):
        new_value = round(self._move_distance + delta, 2)
        new_value = max(MOVE_FORWARD_MIN_M, min(MOVE_FORWARD_MAX_M, new_value))
        self._move_distance = new_value
        self.move_distance_label.config(text=T(f'{new_value:.1f} ม.'))
        self._set_btn_state(self.btn_move_minus, new_value > MOVE_FORWARD_MIN_M)
        self._set_btn_state(self.btn_move_plus, new_value < MOVE_FORWARD_MAX_M)

    def _adjust_move_speed(self, delta):
        new_value = round(self._move_speed + delta, 2)
        new_value = max(MOVE_FORWARD_SPEED_MIN, min(MOVE_FORWARD_SPEED_MAX, new_value))
        self._move_speed = new_value
        self.move_speed_label.config(text=f'{new_value:.2f} m/s')
        self._set_btn_state(self.btn_speed_minus, new_value > MOVE_FORWARD_SPEED_MIN)
        self._set_btn_state(self.btn_speed_plus, new_value < MOVE_FORWARD_SPEED_MAX)

    def _adjust_square_side(self, delta):
        new_value = round(self._square_side + delta, 2)
        new_value = max(SQUARE_SIDE_MIN, min(SQUARE_SIDE_MAX, new_value))
        self._square_side = new_value
        self.square_side_label.config(text=T(f'{new_value:.1f} ม./ด้าน'))
        self._set_btn_state(self.btn_square_minus, new_value > SQUARE_SIDE_MIN)
        self._set_btn_state(self.btn_square_plus, new_value < SQUARE_SIDE_MAX)

    def _adjust_circle_radius(self, delta):
        new_value = round(self._circle_radius + delta, 2)
        new_value = max(CIRCLE_RADIUS_MIN, min(CIRCLE_RADIUS_MAX, new_value))
        self._circle_radius = new_value
        self.circle_radius_label.config(text=T(f'{new_value:.1f} ม.'))
        self._set_btn_state(self.btn_circle_radius_minus, new_value > CIRCLE_RADIUS_MIN)
        self._set_btn_state(self.btn_circle_radius_plus, new_value < CIRCLE_RADIUS_MAX)

    def _adjust_circle_angle(self, delta):
        new_value = round(self._circle_angle + delta)
        new_value = max(CIRCLE_ANGLE_MIN, min(CIRCLE_ANGLE_MAX, new_value))
        self._circle_angle = new_value
        self.circle_angle_label.config(text=f'{new_value:.0f}°')
        self._set_btn_state(self.btn_circle_angle_minus, new_value > CIRCLE_ANGLE_MIN)
        self._set_btn_state(self.btn_circle_angle_plus, new_value < CIRCLE_ANGLE_MAX)

    def _cancel_move_forward(self):
        if self._move_proc is not None and self._move_proc.poll() is None:
            try:
                os.killpg(os.getpgid(self._move_proc.pid), signal.SIGINT)
            except Exception:
                pass

    def _begin_move(self, kind, active_btn, cmd, capture=False):
        self._move_kind = kind
        active_btn.config(text=T('ยกเลิก (Cancel)'))
        for b in (self.btn_move_forward, self.btn_square_path, self.btn_circle_path):
            if b is not active_btn:
                self._set_btn_state(b, False)
        for b in self._start_buttons:
            self._set_card_enabled(b, False)
        env = os.environ.copy()
        self._move_proc = subprocess.Popen(
            cmd, preexec_fn=os.setsid, env=env, text=capture,
            stdout=(subprocess.PIPE if capture else subprocess.DEVNULL),
            stderr=subprocess.DEVNULL)
        self._poll_move_proc()

    def _on_click_move_forward(self):
        if self._move_proc is not None and self._move_proc.poll() is None:
            # Already moving - this click means Cancel.
            self._cancel_move_forward()
            return

        if self._proc is None:
            return

        distance = self._move_distance
        speed = self._move_speed

        if not messagebox.askyesno(
                'ยืนยันการเคลื่อนที่',
                f'ให้หุ่นยนต์เดินหน้าตรง {distance:.2f} เมตร '
                f'ด้วยความเร็ว {speed:.2f} m/s ใช่หรือไม่?\n\n'
                'ตรวจสอบว่าด้านหน้าหุ่นยนต์ไม่มีคน/สิ่งกีดขวางก่อนยืนยัน'):
            return

        self._begin_move('forward', self.btn_move_forward,
                         ['ros2', 'run', 'amr_bringup', 'move_forward_node.py',
                          f'{distance:.3f}', f'{speed:.3f}'])

    def _on_click_calibrate(self):
        if self._calib_panel is not None and self._calib_panel.winfo_exists():
            self._calib_panel.lift()
            self._calib_panel.focus_force()
            return
        if self._proc is None:
            # Calibration nodes need /cmd_vel + /wheel/odom, which only
            # exist once bringup.launch.py (started by Mapping/Navigation)
            # is running the STM32 micro_ros_agent link - without it the
            # rotate/straight tests silently see no odom data.
            messagebox.showwarning(
                'ยังไม่ได้เริ่มระบบ',
                'กรุณาเริ่มระบบ (Mapping / Navigation / Manual) '
                'เพื่อเชื่อมต่อ STM32/odometry ก่อน แล้วค่อยเปิดหน้าปรับเทียบ')
            return
        self._calib_panel = amr_calib_panel.CalibPanel(
            self, on_close=self._on_calib_panel_closed)

    def _on_calib_panel_closed(self):
        self._calib_panel = None

    def _on_click_square_path(self):
        if self._move_proc is not None and self._move_proc.poll() is None:
            # Already moving - this click means Cancel.
            self._cancel_move_forward()
            return

        if self._proc is None:
            return

        side = self._square_side
        speed = self._move_speed

        if not messagebox.askyesno(
                'ยืนยันการเคลื่อนที่',
                f'ให้หุ่นยนต์วิ่งเป็นสี่เหลี่ยมด้านละ {side:.2f} เมตร '
                f'ด้วยความเร็ว {speed:.2f} m/s แล้วกลับมาจุดเดิมใช่หรือไม่?\n\n'
                'ตรวจสอบว่าโดยรอบหุ่นยนต์ไม่มีคน/สิ่งกีดขวางตลอดเส้นทางก่อนยืนยัน'):
            return

        self._begin_move('square', self.btn_square_path,
                         ['ros2', 'run', 'amr_bringup', 'square_path_node.py',
                          f'{side:.3f}', f'{speed:.3f}', f'{SQUARE_TURN_SPEED:.3f}'])

    def _on_click_circle_path(self):
        if self._move_proc is not None and self._move_proc.poll() is None:
            # Already moving - this click means Cancel.
            self._cancel_move_forward()
            return

        if self._proc is None:
            return

        radius = self._circle_radius
        angle = self._circle_angle

        if not messagebox.askyesno(
                'ยืนยันการเคลื่อนที่',
                f'ให้หุ่นยนต์วิ่งเป็นวงกลมรัศมี {radius:.2f} เมตร '
                f'มุม {angle:.0f} องศา ด้วยความเร็วเชิงมุม '
                f'{CIRCLE_ANGULAR_SPEED:.1f} rad/s ใช่หรือไม่?\n\n'
                'ตรวจสอบว่าโดยรอบหุ่นยนต์ไม่มีคน/สิ่งกีดขวางตลอดเส้นทางก่อนยืนยัน'):
            return

        self._begin_move('circle', self.btn_circle_path,
                         ['ros2', 'run', 'amr_bringup', 'circle_path_node.py',
                          f'{radius:.3f}', f'{angle:.1f}', f'{CIRCLE_ANGULAR_SPEED:.3f}'],
                         capture=True)

    def _poll_move_proc(self):
        proc = self._move_proc
        if proc is None:
            return
        ret = proc.poll()
        if ret is None:
            self._move_poll_job = self.after(200, self._poll_move_proc)
            return

        self._move_poll_job = None
        self._move_proc = None
        kind = self._move_kind
        self._move_kind = None
        if kind == 'square':
            if ret == 1:
                messagebox.showwarning(
                    'หยุดกลางทาง',
                    'ใช้เวลานานเกินคาดในช่วงเดินหน้า (odom นิ่ง/หุ่นติดขัด) - หยุดคำสั่งแล้ว')
            elif ret == 2:
                messagebox.showerror(
                    'ไม่สำเร็จ', 'ไม่พบข้อมูล /wheel/raw_distance - ยกเลิกคำสั่ง')
            elif ret == 3:
                messagebox.showwarning(
                    'หยุดกลางทาง',
                    'ใช้เวลานานเกินคาดในช่วงหมุน (หุ่นติดขัด) - หยุดคำสั่งแล้ว')
            elif ret == 4:
                messagebox.showerror(
                    'ไม่สำเร็จ', 'ไม่พบข้อมูล /odometry/filtered - ยกเลิกคำสั่ง')
        else:
            if ret == 1:
                messagebox.showwarning(
                    'หยุดกลางทาง',
                    'ใช้เวลานานเกินคาด (odom นิ่ง/หุ่นติดขัด) - หยุดคำสั่งแล้ว')
            elif ret == 2:
                topic = '/wheel/odom' if kind == 'circle' else '/odometry/filtered'
                messagebox.showerror(
                    'ไม่สำเร็จ', f'ไม่พบข้อมูล {topic} - ยกเลิกคำสั่ง')
            elif kind == 'circle' and ret == 0:
                revs_l, revs_r = self._parse_wheel_revs(proc)
                if revs_l is not None:
                    messagebox.showinfo(
                        'วิ่งวงกลมเสร็จสิ้น',
                        f'จำนวนรอบที่ล้อหมุนไป:\n'
                        f'  ล้อซ้าย: {revs_l:.2f} รอบ\n'
                        f'  ล้อขวา: {revs_r:.2f} รอบ')
        self._reset_move_ui()

    @staticmethod
    def _parse_wheel_revs(proc):
        """Reads circle_path_node.py's 'WHEEL_REVS left=.. right=..' stdout
        line. Returns (revs_l, revs_r) as floats, or (None, None) if the
        process produced no parseable output (e.g. no pulse_count data)."""
        try:
            output = proc.stdout.read() if proc.stdout else ''
        except Exception:
            output = ''
        for line in output.splitlines():
            if line.startswith('WHEEL_REVS left='):
                try:
                    parts = line.split()
                    revs_l = float(parts[1].split('=')[1])
                    revs_r = float(parts[2].split('=')[1])
                    return revs_l, revs_r
                except (IndexError, ValueError):
                    return None, None
        return None, None

    def _reset_move_ui(self):
        running = self._proc is not None
        for b, text in ((self.btn_move_forward, 'เดินหน้า (Move Forward)'),
                        (self.btn_square_path, 'วิ่งเป็นสี่เหลี่ยม (Square Path)'),
                        (self.btn_circle_path, 'วิ่งวงกลม (Circle Path)')):
            self._set_btn_state(b, running)
            b.config(text=T(text))
        for b in self._start_buttons:
            self._set_card_enabled(b, not running)

    def _show_save_name_dialog(self, default_name, title='ตั้งชื่อแผนที่'):
        result = {'name': None}
        win = tk.Toplevel(self)
        win.title(title)
        win.configure(bg=SURFACE)
        win.transient(self)
        win.resizable(False, False)

        self._label(win, title, font=self.f_button).pack(padx=px(20), pady=(px(18), px(8)), anchor='w')

        entry_var = tk.StringVar(value=default_name)
        entry = tk.Entry(win, textvariable=entry_var, font=self.f_button,
                         bg=SURFACE_2, fg=TEXT_MAIN, insertbackground=TEXT_MAIN,
                         relief='flat')
        entry.pack(padx=px(20), fill='x', ipady=px(10))

        hint = self._label(win, 'ใช้ตัวอักษร ตัวเลข - และ _ เท่านั้น',
                           font=self.f_small, fg=TEXT_MUTED, anchor='w')
        hint.pack(padx=px(20), pady=(px(6), px(0)), fill='x')

        btn_row = tk.Frame(win, bg=SURFACE)
        btn_row.pack(fill='x', padx=px(20), pady=px(16))

        def confirm(_e=None):
            name = sanitize_map_name(entry_var.get())
            if not name:
                hint.config(text=T('กรุณาใส่ชื่อที่ถูกต้อง'), fg=RED)
                return
            result['name'] = name
            win.destroy()

        def cancel(_e=None):
            win.destroy()

        self._btn(btn_row, 'ยกเลิก', cancel).pack(side='right')
        self._btn(btn_row, 'บันทึก', confirm, kind='primary', padx=px(24)
                  ).pack(side='right', padx=(px(0), px(8)))

        win.bind('<Return>', confirm)
        win.bind('<Escape>', cancel)
        win.protocol('WM_DELETE_WINDOW', cancel)

        self._center_toplevel(win, px(440), px(230))
        win.grab_set()
        entry.focus_set()
        entry.select_range(0, 'end')
        win.wait_window()
        return result['name']

    def _on_save_map(self):
        if self._mode != 'mapping' or self._proc is None:
            return
        default_name = time.strftime('map_%Y%m%d_%H%M%S')
        name = self._show_save_name_dialog(default_name)
        if not name:
            return
        os.makedirs(MAPS_DIR, exist_ok=True)
        map_path = os.path.join(MAPS_DIR, name)
        if os.path.exists(map_path + '.yaml'):
            if not messagebox.askyesno(
                    'ชื่อซ้ำ', f'มีแผนที่ชื่อ "{name}" อยู่แล้ว ต้องการเขียนทับหรือไม่?'):
                return
        self._start_map_save(name)

    def _start_map_save(self, name, from_phone=False):
        if self._map_saving:
            return
        os.makedirs(MAPS_DIR, exist_ok=True)
        map_path = os.path.join(MAPS_DIR, name)
        self._map_saving = True
        self._mapping_msg = f'กำลังบันทึก {name}...'
        self._refresh_mapping_ui()
        self._set_btn_state(self.btn_save_map, False)
        self.btn_save_map.config(text=T('กำลังบันทึก...'))

        def do_save():
            try:
                result = subprocess.run(
                    ['ros2', 'run', 'nav2_map_server', 'map_saver_cli',
                     '-f', map_path, '--ros-args', '-p', 'save_map_timeout:=10.0'],
                    capture_output=True, text=True, timeout=20)
                ok = result.returncode == 0 and os.path.exists(map_path + '.yaml')
            except subprocess.TimeoutExpired:
                ok = False
            self.after(0, lambda: self._after_save(ok, name, from_phone))
        threading.Thread(target=do_save, daemon=True).start()

    def _after_save(self, ok, name, from_phone=False):
        self._map_saving = False
        self._mapping_msg = (f'บันทึกแล้ว: {name}' if ok else 'บันทึกแผนที่ไม่สำเร็จ')
        self._refresh_mapping_ui()
        self._set_btn_state(self.btn_save_map, self._mode == 'mapping')
        self.btn_save_map.config(text=T('SAVE MAP (บันทึกแผนที่)'))
        if from_phone:
            # no modal on the panel - the operator is at the phone
            self._set_status('running', T(self._mapping_msg))
            return
        if ok:
            messagebox.showinfo('บันทึกสำเร็จ', f'บันทึกแผนที่แล้ว: {name}.yaml')
        else:
            messagebox.showerror('บันทึกไม่สำเร็จ',
                                 'ไม่สามารถบันทึกแผนที่ได้ ลองใหม่อีกครั้ง')

    def _on_close(self):
        if self._proc is not None:
            if not messagebox.askyesno(
                    'ปิดโปรแกรม', 'ระบบกำลังทำงานอยู่ ต้องการหยุดและปิดโปรแกรมหรือไม่?'):
                return
            try:
                os.killpg(os.getpgid(self._proc.pid), signal.SIGINT)
            except Exception:
                pass
        self._cancel_move_forward()
        if self._move_proc is not None:
            try:
                self._move_proc.wait(timeout=2)
            except Exception:
                pass
        if self._move_poll_job is not None:
            self.after_cancel(self._move_poll_job)
        if self._calib_panel is not None and self._calib_panel.winfo_exists():
            self._calib_panel._on_close()
        self.ros.shutdown()
        self.destroy()


def main():
    app = AmrControlPanel()
    app.mainloop()


if __name__ == '__main__':
    main()
