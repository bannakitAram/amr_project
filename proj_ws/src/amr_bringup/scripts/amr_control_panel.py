#!/usr/bin/env python3
"""Minimal desktop launcher for the AMR: pick Mapping or Navigation mode,
start/stop the corresponding bringup, and (in Mapping mode) save the map -
without needing to type any ros2 launch commands by hand.
"""
import glob
import os
import re
import signal
import socket
import subprocess
import threading
import time
import tkinter as tk
from tkinter import font as tkfont
from tkinter import messagebox, ttk

from urllib.parse import quote

import qrcode
from PIL import ImageTk

from ament_index_python.packages import get_package_share_directory

import amr_calib_panel

MAPS_DIR = os.path.join(
    get_package_share_directory('amr_navigation'), '..', '..', '..', '..',
    'src', 'amr_navigation', 'maps')
MAPS_DIR = os.path.normpath(MAPS_DIR)

ROSBOARD_PORT = 8888

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

BG = '#f4f6f8'
CARD_BG = '#ffffff'
ACCENT = '#2e7d32'
ACCENT_DARK = '#1b5e20'
NAV_ACCENT = '#1565c0'
NAV_ACCENT_DARK = '#0d47a1'
DANGER = '#c62828'
TEXT_MAIN = '#1a1a1a'
TEXT_MUTED = '#6b7280'


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


def is_port_open(host, port, timeout=0.5):
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.settimeout(timeout)
    try:
        return s.connect_ex((host, port)) == 0
    finally:
        s.close()


class AmrControlPanel(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title('AMR Control')
        self.configure(bg=BG)
        # Fixed to the AMR's onboard touchscreen resolution (1024x600) - not
        # resizable, since this runs full-screen/kiosk-style on that panel
        # with no window manager to drag/resize it against.
        self.geometry('1024x600')
        self.minsize(1024, 600)
        self.resizable(False, False)

        self._proc = None
        self._mode = None  # 'mapping' | 'navigation'
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

        self._build_fonts()
        self._build_ui()
        self._set_status('idle', 'พร้อมใช้งาน')
        self.protocol('WM_DELETE_WINDOW', self._on_close)

    def _build_fonts(self):
        self.f_title = tkfont.Font(family='Sans', size=22, weight='bold')
        self.f_button = tkfont.Font(family='Sans', size=15, weight='bold')
        self.f_button_sub = tkfont.Font(family='Sans', size=10)
        self.f_status = tkfont.Font(family='Sans', size=12)
        self.f_small = tkfont.Font(family='Sans', size=10)

    def _build_ui(self):
        header = tk.Frame(self, bg=BG)
        header.pack(fill='x', padx=28, pady=(16, 6))
        tk.Label(header, text='AMR Control', font=self.f_title,
                 bg=BG, fg=TEXT_MAIN).pack(anchor='w')
        tk.Label(header, text='เลือกโหมดการทำงาน', font=self.f_small,
                 bg=BG, fg=TEXT_MUTED).pack(anchor='w')

        cards = tk.Frame(self, bg=BG)
        cards.pack(fill='x', padx=28, pady=6)
        cards.columnconfigure(0, weight=1)
        cards.columnconfigure(1, weight=1)

        self.btn_mapping = self._make_mode_card(
            cards, 'สร้างแผนที่', 'Mapping', ACCENT, ACCENT_DARK,
            self._on_click_mapping)
        self.btn_mapping.grid(row=0, column=0, sticky='nsew', padx=(0, 8))

        self.btn_navigation = self._make_mode_card(
            cards, 'นำทาง', 'Navigation', NAV_ACCENT, NAV_ACCENT_DARK,
            self._on_click_navigation)
        self.btn_navigation.grid(row=0, column=1, sticky='nsew', padx=(8, 0))

        # Status card
        status_card = tk.Frame(self, bg=CARD_BG, highlightthickness=1,
                                highlightbackground='#e0e0e0')
        status_card.pack(fill='x', padx=28, pady=10)
        inner = tk.Frame(status_card, bg=CARD_BG)
        inner.pack(fill='x', padx=18, pady=10)

        dot_row = tk.Frame(inner, bg=CARD_BG)
        dot_row.pack(fill='x')
        self.status_dot = tk.Canvas(dot_row, width=14, height=14,
                                     bg=CARD_BG, highlightthickness=0)
        self.status_dot.pack(side='left', padx=(0, 8))
        self._dot_id = self.status_dot.create_oval(2, 2, 12, 12, fill='#9e9e9e', outline='')
        self.status_label = tk.Label(dot_row, text='', font=self.f_status,
                                      bg=CARD_BG, fg=TEXT_MAIN, anchor='w')
        self.status_label.pack(side='left', fill='x', expand=True)

        self.detail_label = tk.Label(inner, text='', font=self.f_small,
                                      bg=CARD_BG, fg=TEXT_MUTED, anchor='w',
                                      justify='left', wraplength=930)
        self.detail_label.pack(fill='x', pady=(6, 0))

        # Action row (Save map / Stop)
        actions = tk.Frame(self, bg=BG)
        actions.pack(fill='x', padx=28, pady=(0, 14))

        self.btn_save_map = tk.Button(
            actions, text='บันทึกแผนที่ (Save Map)', font=self.f_button_sub,
            bg='#ffffff', fg=ACCENT_DARK, activebackground='#e8f5e9',
            relief='solid', bd=1, padx=14, pady=10, cursor='hand2',
            command=self._on_save_map, state='disabled')
        self.btn_save_map.pack(side='left')

        self.btn_stop = tk.Button(
            actions, text='หยุด (Stop)', font=self.f_button_sub,
            bg='#ffffff', fg=DANGER, activebackground='#ffebee',
            relief='solid', bd=1, padx=20, pady=10, cursor='hand2',
            command=self._on_stop, state='disabled')
        self.btn_stop.pack(side='right')

        # Motion-preset card: "Move forward" / "Square path" / "Circle path",
        # each closed-loop on odom (see their respective *_node.py module
        # docstrings). Shown as ttk.Notebook tabs, one preset's controls
        # visible at a time - the control panel's window is fixed to the
        # onboard touchscreen's 1024x600 resolution and does not fit all
        # three presets' rows stacked at once (measured ~751px required vs.
        # 600px available before this was tabbed - the third preset's row
        # was silently clipped below the visible window).
        move_card = tk.Frame(self, bg=CARD_BG, highlightthickness=1,
                              highlightbackground='#e0e0e0')
        move_card.pack(fill='x', padx=28, pady=(0, 14))
        move_inner = tk.Frame(move_card, bg=CARD_BG)
        move_inner.pack(fill='x', padx=18, pady=10)

        style = ttk.Style(self)
        style.configure('Motion.TNotebook.Tab', font=self.f_status, padding=(12, 6))

        motion_tabs = ttk.Notebook(move_inner, style='Motion.TNotebook')
        motion_tabs.pack(fill='x')

        tab_forward = tk.Frame(motion_tabs, bg=CARD_BG)
        motion_tabs.add(tab_forward, text='เดินหน้าตรง')
        tab_square = tk.Frame(motion_tabs, bg=CARD_BG)
        motion_tabs.add(tab_square, text='สี่เหลี่ยม')
        tab_circle = tk.Frame(motion_tabs, bg=CARD_BG)
        motion_tabs.add(tab_circle, text='วงกลม')

        move_row = tk.Frame(tab_forward, bg=CARD_BG)
        move_row.pack(fill='x', pady=(8, 0))

        self._move_distance = 1.0
        self.btn_move_minus = tk.Button(
            move_row, text='-', font=self.f_button_sub,
            bg='#ffffff', fg=TEXT_MAIN, relief='solid', bd=1,
            width=2, cursor='hand2', state='disabled',
            command=lambda: self._adjust_move_distance(-MOVE_FORWARD_STEP_M))
        self.btn_move_minus.pack(side='left')

        self.move_distance_label = tk.Label(
            move_row, text='1.0 ม.', font=self.f_status,
            bg=CARD_BG, fg=TEXT_MAIN, width=8, anchor='center')
        self.move_distance_label.pack(side='left', padx=6)

        self.btn_move_plus = tk.Button(
            move_row, text='+', font=self.f_button_sub,
            bg='#ffffff', fg=TEXT_MAIN, relief='solid', bd=1,
            width=2, cursor='hand2', state='disabled',
            command=lambda: self._adjust_move_distance(MOVE_FORWARD_STEP_M))
        self.btn_move_plus.pack(side='left', padx=(0, 16))

        self.btn_move_forward = tk.Button(
            move_row, text='เดินหน้า (Move Forward)', font=self.f_button_sub,
            bg='#ffffff', fg=NAV_ACCENT_DARK, activebackground='#e3f2fd',
            relief='solid', bd=1, padx=14, pady=8, cursor='hand2',
            command=self._on_click_move_forward, state='disabled')
        self.btn_move_forward.pack(side='left')

        speed_row = tk.Frame(tab_forward, bg=CARD_BG)
        speed_row.pack(fill='x', pady=(8, 0))

        tk.Label(speed_row, text='ความเร็ว:', font=self.f_status,
                 bg=CARD_BG, fg=TEXT_MAIN).pack(side='left', padx=(0, 6))

        self._move_speed = MOVE_FORWARD_SPEED_DEFAULT
        self.btn_speed_minus = tk.Button(
            speed_row, text='-', font=self.f_button_sub,
            bg='#ffffff', fg=TEXT_MAIN, relief='solid', bd=1,
            width=2, cursor='hand2', state='disabled',
            command=lambda: self._adjust_move_speed(-MOVE_FORWARD_SPEED_STEP))
        self.btn_speed_minus.pack(side='left')

        self.move_speed_label = tk.Label(
            speed_row, text=f'{MOVE_FORWARD_SPEED_DEFAULT:.2f} m/s',
            font=self.f_status, bg=CARD_BG, fg=TEXT_MAIN, width=8, anchor='center')
        self.move_speed_label.pack(side='left', padx=6)

        self.btn_speed_plus = tk.Button(
            speed_row, text='+', font=self.f_button_sub,
            bg='#ffffff', fg=TEXT_MAIN, relief='solid', bd=1,
            width=2, cursor='hand2', state='disabled',
            command=lambda: self._adjust_move_speed(MOVE_FORWARD_SPEED_STEP))
        self.btn_speed_plus.pack(side='left')

        tk.Label(tab_forward,
                 text=f'ปรับความเร็วได้ {MOVE_FORWARD_SPEED_MIN:.2f}-'
                      f'{MOVE_FORWARD_SPEED_MAX:.2f} m/s, ระยะจำกัด '
                      f'{MOVE_FORWARD_MIN_M:.2f}-{MOVE_FORWARD_MAX_M:.1f} ม. '
                      '- ตรวจสอบพื้นที่ด้านหน้าว่าปลอดภัยก่อนกด (ISO 3691-4)',
                 font=self.f_small, bg=CARD_BG, fg=TEXT_MUTED, anchor='w',
                 justify='left', wraplength=930
                 ).pack(anchor='w', pady=(6, 4))

        # Square path (4x straight leg + 90 deg turn, closed-loop, returns
        # to the starting point/heading - see square_path_node.py). Shares
        # tab_forward's speed control (self._move_speed).
        square_row = tk.Frame(tab_square, bg=CARD_BG)
        square_row.pack(fill='x', pady=(8, 0))

        self._square_side = SQUARE_SIDE_DEFAULT
        self.btn_square_minus = tk.Button(
            square_row, text='-', font=self.f_button_sub,
            bg='#ffffff', fg=TEXT_MAIN, relief='solid', bd=1,
            width=2, cursor='hand2', state='disabled',
            command=lambda: self._adjust_square_side(-SQUARE_SIDE_STEP))
        self.btn_square_minus.pack(side='left')

        self.square_side_label = tk.Label(
            square_row, text=f'{SQUARE_SIDE_DEFAULT:.1f} ม./ด้าน',
            font=self.f_status, bg=CARD_BG, fg=TEXT_MAIN, width=10,
            anchor='center')
        self.square_side_label.pack(side='left', padx=6)

        self.btn_square_plus = tk.Button(
            square_row, text='+', font=self.f_button_sub,
            bg='#ffffff', fg=TEXT_MAIN, relief='solid', bd=1,
            width=2, cursor='hand2', state='disabled',
            command=lambda: self._adjust_square_side(SQUARE_SIDE_STEP))
        self.btn_square_plus.pack(side='left', padx=(0, 16))

        self.btn_square_path = tk.Button(
            square_row, text='วิ่งเป็นสี่เหลี่ยม (Square Path)',
            font=self.f_button_sub, bg='#ffffff', fg=NAV_ACCENT_DARK,
            activebackground='#e3f2fd', relief='solid', bd=1, padx=14,
            pady=8, cursor='hand2', command=self._on_click_square_path,
            state='disabled')
        self.btn_square_path.pack(side='left')

        tk.Label(tab_square,
                 text=f'ด้านละ {SQUARE_SIDE_MIN:.1f}-{SQUARE_SIDE_MAX:.1f} ม. '
                      '(รวม 4 ด้าน) ใช้ความเร็วเดินหน้าเดียวกับแท็บเดินหน้าตรง '
                      'หมุนกลับทวนเข็มนาฬิกา 90 องศาต่อมุม - ตรวจสอบพื้นที่โดยรอบ'
                      'หุ่นยนต์ให้ปลอดภัยตลอดเส้นทางก่อนกด (ISO 3691-4)',
                 font=self.f_small, bg=CARD_BG, fg=TEXT_MUTED, anchor='w',
                 justify='left', wraplength=930
                 ).pack(anchor='w', pady=(6, 4))

        # Circle path (constant-curvature arc, closed-loop - see
        # circle_path_node.py).
        circle_row = tk.Frame(tab_circle, bg=CARD_BG)
        circle_row.pack(fill='x', pady=(8, 0))

        tk.Label(circle_row, text='รัศมี:', font=self.f_status,
                 bg=CARD_BG, fg=TEXT_MAIN).pack(side='left', padx=(0, 4))

        self._circle_radius = CIRCLE_RADIUS_DEFAULT
        self.btn_circle_radius_minus = tk.Button(
            circle_row, text='-', font=self.f_button_sub,
            bg='#ffffff', fg=TEXT_MAIN, relief='solid', bd=1,
            width=2, cursor='hand2', state='disabled',
            command=lambda: self._adjust_circle_radius(-CIRCLE_RADIUS_STEP))
        self.btn_circle_radius_minus.pack(side='left')

        self.circle_radius_label = tk.Label(
            circle_row, text=f'{CIRCLE_RADIUS_DEFAULT:.1f} ม.',
            font=self.f_status, bg=CARD_BG, fg=TEXT_MAIN, width=8,
            anchor='center')
        self.circle_radius_label.pack(side='left', padx=6)

        self.btn_circle_radius_plus = tk.Button(
            circle_row, text='+', font=self.f_button_sub,
            bg='#ffffff', fg=TEXT_MAIN, relief='solid', bd=1,
            width=2, cursor='hand2', state='disabled',
            command=lambda: self._adjust_circle_radius(CIRCLE_RADIUS_STEP))
        self.btn_circle_radius_plus.pack(side='left', padx=(0, 16))

        tk.Label(circle_row, text='มุม:', font=self.f_status,
                 bg=CARD_BG, fg=TEXT_MAIN).pack(side='left', padx=(0, 4))

        self._circle_angle = CIRCLE_ANGLE_DEFAULT
        self.btn_circle_angle_minus = tk.Button(
            circle_row, text='-', font=self.f_button_sub,
            bg='#ffffff', fg=TEXT_MAIN, relief='solid', bd=1,
            width=2, cursor='hand2', state='disabled',
            command=lambda: self._adjust_circle_angle(-CIRCLE_ANGLE_STEP))
        self.btn_circle_angle_minus.pack(side='left')

        self.circle_angle_label = tk.Label(
            circle_row, text=f'{CIRCLE_ANGLE_DEFAULT:.0f}°',
            font=self.f_status, bg=CARD_BG, fg=TEXT_MAIN, width=6,
            anchor='center')
        self.circle_angle_label.pack(side='left', padx=6)

        self.btn_circle_angle_plus = tk.Button(
            circle_row, text='+', font=self.f_button_sub,
            bg='#ffffff', fg=TEXT_MAIN, relief='solid', bd=1,
            width=2, cursor='hand2', state='disabled',
            command=lambda: self._adjust_circle_angle(CIRCLE_ANGLE_STEP))
        self.btn_circle_angle_plus.pack(side='left', padx=(0, 16))

        self.btn_circle_path = tk.Button(
            circle_row, text='วิ่งวงกลม (Circle Path)',
            font=self.f_button_sub, bg='#ffffff', fg=NAV_ACCENT_DARK,
            activebackground='#e3f2fd', relief='solid', bd=1, padx=14,
            pady=8, cursor='hand2', command=self._on_click_circle_path,
            state='disabled')
        self.btn_circle_path.pack(side='left')

        tk.Label(tab_circle,
                 text=f'รัศมี {CIRCLE_RADIUS_MIN:.1f}-{CIRCLE_RADIUS_MAX:.1f} ม., '
                      f'มุม {CIRCLE_ANGLE_MIN}-{CIRCLE_ANGLE_MAX} องศา, '
                      f'ความเร็วเชิงมุมคงที่ {CIRCLE_ANGULAR_SPEED:.1f} rad/s - '
                      'ตรวจสอบพื้นที่โดยรอบหุ่นยนต์ตลอดเส้นทางก่อนกด (ISO 3691-4)',
                 font=self.f_small, bg=CARD_BG, fg=TEXT_MUTED, anchor='w',
                 justify='left', wraplength=930
                 ).pack(anchor='w', pady=(6, 4))

        # Calibration launcher: not tied to any one motion preset, so it
        # lives below the Notebook (always visible regardless of which tab
        # is selected) rather than inside a tab.
        calib_row = tk.Frame(move_inner, bg=CARD_BG)
        calib_row.pack(fill='x', pady=(8, 0))

        self.btn_calibrate = tk.Button(
            calib_row, text='ปรับเทียบ Localization (Calibrate)',
            font=self.f_button_sub, bg='#ffffff', fg=NAV_ACCENT_DARK,
            activebackground='#e3f2fd', relief='solid', bd=1, padx=14,
            pady=8, cursor='hand2', command=self._on_click_calibrate)
        self.btn_calibrate.pack(side='left')

    def _make_mode_card(self, parent, title_th, title_en, color, color_dark, command):
        # A real tk.Button cannot reliably contain other packed widgets - the
        # geometry manager fights itself on every hover/redraw, causing the
        # flicker/resize/collapsing-text bug. Use a plain Frame styled and
        # bound to behave like a button instead.
        sub_color = '#e8f5e9' if color == ACCENT else '#e3f2fd'
        card = tk.Frame(parent, bg=color, cursor='hand2')
        card.pack_propagate(True)
        content = tk.Frame(card, bg=color)
        content.pack(expand=True, fill='both', padx=18, pady=28)
        title_lbl = tk.Label(content, text=title_th, font=self.f_button,
                              bg=color, fg='white', cursor='hand2')
        title_lbl.pack(anchor='w')
        sub_lbl = tk.Label(content, text=title_en, font=self.f_button_sub,
                            bg=color, fg=sub_color, cursor='hand2')
        sub_lbl.pack(anchor='w')

        widgets = (card, content, title_lbl, sub_lbl)
        card.enabled = True

        def on_click(_e=None):
            if card.enabled:
                command()

        def on_enter(_e=None):
            if card.enabled:
                for w in (card, content):
                    w.config(bg=color_dark)
                title_lbl.config(bg=color_dark)
                sub_lbl.config(bg=color_dark)

        def on_leave(_e=None):
            if card.enabled:
                for w in (card, content):
                    w.config(bg=color)
                title_lbl.config(bg=color)
                sub_lbl.config(bg=color)

        for w in widgets:
            w.bind('<Button-1>', on_click)
            w.bind('<Enter>', on_enter)
            w.bind('<Leave>', on_leave)

        card._title_lbl = title_lbl
        card._sub_lbl = sub_lbl
        card._content = content
        card._color = color
        card._sub_color = sub_color
        return card

    def _set_card_enabled(self, card, enabled):
        card.enabled = enabled
        color = card._color if enabled else '#c8c8c8'
        sub_color = card._sub_color if enabled else '#e0e0e0'
        card.config(bg=color, cursor=('hand2' if enabled else 'arrow'))
        card._content.config(bg=color)
        card._title_lbl.config(bg=color, cursor=('hand2' if enabled else 'arrow'))
        card._sub_lbl.config(bg=color, fg=sub_color,
                              cursor=('hand2' if enabled else 'arrow'))

    # ---------- status helpers ----------
    def _set_status(self, kind, text, detail=''):
        colors = {
            'idle': '#9e9e9e',
            'starting': '#f9a825',
            'running': '#2e7d32',
            'error': '#c62828',
        }
        self.status_dot.itemconfig(self._dot_id, fill=colors.get(kind, '#9e9e9e'))
        self.status_label.config(text=text)
        self.detail_label.config(text=detail)

    def _set_running_ui(self, running, mode=None):
        self._set_card_enabled(self.btn_mapping, not running)
        self._set_card_enabled(self.btn_navigation, not running)
        self.btn_stop.config(state=('normal' if running else 'disabled'))
        self.btn_save_map.config(
            state=('normal' if (running and mode == 'mapping') else 'disabled'))
        is_moving = self._move_proc is not None and self._move_proc.poll() is None
        if not is_moving:
            self.btn_move_forward.config(
                state=('normal' if running else 'disabled'),
                text='เดินหน้า (Move Forward)')
            self.btn_square_path.config(
                state=('normal' if running else 'disabled'),
                text='วิ่งเป็นสี่เหลี่ยม (Square Path)')
            self.btn_circle_path.config(
                state=('normal' if running else 'disabled'),
                text='วิ่งวงกลม (Circle Path)')
            step_state = 'normal' if running else 'disabled'
            self.btn_move_minus.config(state=step_state)
            self.btn_move_plus.config(state=step_state)
            self.btn_speed_minus.config(state=step_state)
            self.btn_speed_plus.config(state=step_state)
            self.btn_square_minus.config(state=step_state)
            self.btn_square_plus.config(state=step_state)
            self.btn_circle_radius_minus.config(state=step_state)
            self.btn_circle_radius_plus.config(state=step_state)
            self.btn_circle_angle_minus.config(state=step_state)
            self.btn_circle_angle_plus.config(state=step_state)
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
        ], mode='mapping', label='โหมดสร้างแผนที่ (Mapping)')
        threading.Thread(target=self._ensure_monitors_then_show_qr,
                          daemon=True).start()

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
        img = qrcode.make(url).resize((220, 220))
        photo = ImageTk.PhotoImage(img)
        self._qr_photos.append(photo)  # keep a reference so Tk doesn't GC it

        block = tk.Frame(parent, bg=CARD_BG)
        tk.Label(block, text=title, font=self.f_status, bg=CARD_BG,
                 fg=TEXT_MAIN, wraplength=260, justify='center'
                 ).pack(pady=(0, 8))
        tk.Label(block, image=photo, bg=CARD_BG).pack()
        tk.Label(block, text=url, font=self.f_small, bg=CARD_BG,
                 fg=TEXT_MUTED, wraplength=260, justify='center'
                 ).pack(pady=(8, 4))
        if extra_note:
            tk.Label(block, text=extra_note, font=self.f_small, bg=CARD_BG,
                     fg=TEXT_MUTED, wraplength=260, justify='center'
                     ).pack(pady=(0, 4))
        return block

    def _show_monitor_qr(self, mobile_url, rosboard_url):
        self._qr_photos = []

        win = tk.Toplevel(self)
        win.title('Monitor บนมือถือ')
        win.configure(bg=CARD_BG)
        win.transient(self)
        win.resizable(False, False)

        tk.Label(win, text='สแกนเพื่อดูแผนที่ + ควบคุมขับจากมือถือ',
                 font=self.f_status, bg=CARD_BG, fg=TEXT_MAIN,
                 wraplength=560, justify='center'
                 ).pack(padx=20, pady=(18, 4))
        tk.Label(win, text='ต่อ WiFi วงเดียวกับ Jetson นี้ก่อนสแกน (ไม่ต้องใช้อินเทอร์เน็ต)',
                 font=self.f_small, bg=CARD_BG, fg=TEXT_MUTED,
                 wraplength=560, justify='center'
                 ).pack(padx=20, pady=(0, 12))

        cols = tk.Frame(win, bg=CARD_BG)
        cols.pack(padx=20, fill='x')

        if mobile_url:
            mobile_block = self._add_qr_block(
                cols, 'แผนที่ + ควบคุม (แนะนำ)', mobile_url,
                extra_note='แผนที่แบบ Foxglove (Lichtblick) ด้านบน + จอย'
                            'สติ๊กควบคุมขับ (rosboard 🕹) ด้านล่าง - '
                            'โหลดครั้งแรกอาจช้าสักครู่ (~170MB ผ่าน LAN)')
            mobile_block.grid(row=0, column=0, padx=(0, 16), sticky='n')
        else:
            tk.Label(cols, text='หน้ารวม (แผนที่+ควบคุม) ยังไม่พร้อมใช้งาน\n'
                                 f'(ต้องมีทั้ง rosboard และ {LICHTBLICK_DIST_DIR})',
                     font=self.f_small, bg=CARD_BG, fg=DANGER,
                     wraplength=220, justify='center'
                     ).grid(row=0, column=0, padx=(0, 16), sticky='n')

        rosboard_block = self._add_qr_block(
            cols, 'Monitor + ควบคุม เบา (rosboard เท่านั้น)', rosboard_url)
        rosboard_block.grid(row=0, column=1, padx=(16, 0), sticky='n')

        pin = get_rosboard_pin()
        if pin:
            pin_frame = tk.Frame(win, bg='#fff8e1', highlightthickness=1,
                                  highlightbackground='#f9a825')
            pin_frame.pack(padx=20, pady=(14, 4), fill='x')
            tk.Label(pin_frame, text='รหัสเข้าใช้งาน rosboard (PIN)', font=self.f_small,
                     bg='#fff8e1', fg=TEXT_MUTED).pack(pady=(8, 0))
            tk.Label(pin_frame, text=pin, font=tkfont.Font(
                family='Sans', size=24, weight='bold'),
                bg='#fff8e1', fg='#e65100').pack(pady=(0, 8))

        tk.Button(win, text='ปิด', font=self.f_button_sub, bg='#ffffff',
                  fg=TEXT_MUTED, relief='solid', bd=1, padx=14, pady=8,
                  cursor='hand2', command=win.destroy
                  ).pack(pady=(10, 18))

        win.update_idletasks()
        self._center_toplevel(win, win.winfo_reqwidth() + 10,
                               win.winfo_reqheight() + 10)

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
        self._launch([
            'ros2', 'launch', 'amr_bringup', 'bringup.launch.py',
            'rviz:=false', 'foxglove:=true', 'slam:=false', 'navigation:=true',
            f'map:={map_path}',
        ], mode='navigation',
            label=f'โหมดนำทาง (Navigation)\nแผนที่: {os.path.basename(map_path)}')
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
        self._qr_photos = []

        win = tk.Toplevel(self)
        win.title('ตั้งจุดหมายจากมือถือ')
        win.configure(bg=CARD_BG)
        win.transient(self)
        win.resizable(False, False)

        tk.Label(win, text='สแกนเพื่อเลือกจุดหมาย/วาดเส้นทางให้หุ่นยนต์วิ่งตาม',
                 font=self.f_status, bg=CARD_BG, fg=TEXT_MAIN,
                 wraplength=560, justify='center'
                 ).pack(padx=20, pady=(18, 4))
        tk.Label(win, text='ต่อ WiFi วงเดียวกับ Jetson นี้ก่อนสแกน (ไม่ต้องใช้อินเทอร์เน็ต)\n'
                            'หุ่นยนต์จะเคลื่อนที่แบบอัตโนมัติทันทีที่ยืนยันบนหน้าจอนั้น - '
                            'ตรวจสอบพื้นที่ให้ปลอดภัยก่อนเสมอ (ISO 3691-4)',
                 font=self.f_small, bg=CARD_BG, fg=TEXT_MUTED,
                 wraplength=560, justify='center'
                 ).pack(padx=20, pady=(0, 12))

        cols = tk.Frame(win, bg=CARD_BG)
        cols.pack(padx=20, fill='x')

        if goal_picker_url:
            goal_block = self._add_qr_block(
                cols, 'ตั้งจุดหมาย / ลากเส้นทาง (แนะนำ)', goal_picker_url,
                extra_note='แตะแผนที่เพื่อส่งหุ่นยนต์ไปจุดนั้น หรือลากเพื่อวาดเส้นทาง')
            goal_block.grid(row=0, column=0, padx=(0, 16), sticky='n')
        else:
            tk.Label(cols, text='หน้าตั้งจุดหมายยังไม่พร้อมใช้งาน (rosboard ไม่ทำงาน)',
                     font=self.f_small, bg=CARD_BG, fg=DANGER,
                     wraplength=220, justify='center'
                     ).grid(row=0, column=0, padx=(0, 16), sticky='n')

        if rosboard_url:
            rosboard_block = self._add_qr_block(
                cols, 'Monitor แผนที่ (rosboard)', rosboard_url)
            rosboard_block.grid(row=0, column=1, padx=(16, 0), sticky='n')

        pin = get_rosboard_pin()
        if pin:
            pin_frame = tk.Frame(win, bg='#fff8e1', highlightthickness=1,
                                  highlightbackground='#f9a825')
            pin_frame.pack(padx=20, pady=(14, 4), fill='x')
            tk.Label(pin_frame, text='รหัสเข้าใช้งาน (PIN)', font=self.f_small,
                     bg='#fff8e1', fg=TEXT_MUTED).pack(pady=(8, 0))
            tk.Label(pin_frame, text=pin, font=tkfont.Font(
                family='Sans', size=24, weight='bold'),
                bg='#fff8e1', fg='#e65100').pack(pady=(0, 8))

        tk.Button(win, text='ปิด', font=self.f_button_sub, bg='#ffffff',
                  fg=TEXT_MUTED, relief='solid', bd=1, padx=14, pady=8,
                  cursor='hand2', command=win.destroy
                  ).pack(pady=(10, 18))

        win.update_idletasks()
        self._center_toplevel(win, win.winfo_reqwidth() + 10,
                               win.winfo_reqheight() + 10)

    def _center_toplevel(self, win, w, h):
        self.update_idletasks()
        x = self.winfo_x() + (self.winfo_width() - w) // 2
        y = self.winfo_y() + (self.winfo_height() - h) // 2
        win.geometry(f'{w}x{h}+{max(x, 0)}+{max(y, 0)}')

    def _show_map_picker(self):
        yamls = sorted(glob.glob(os.path.join(MAPS_DIR, '*.yaml')),
                        key=os.path.getmtime, reverse=True)
        names = [os.path.splitext(os.path.basename(p))[0] for p in yamls]

        result = {'path': None}
        win = tk.Toplevel(self)
        win.title('เลือกแผนที่')
        win.configure(bg=CARD_BG)
        win.transient(self)
        win.resizable(False, False)

        tk.Label(win, text='เลือกแผนที่สำหรับนำทาง', font=self.f_status,
                 bg=CARD_BG, fg=TEXT_MAIN).pack(padx=20, pady=(18, 10), anchor='w')

        list_frame = tk.Frame(win, bg=CARD_BG)
        list_frame.pack(padx=20, fill='both', expand=True)
        scrollbar = tk.Scrollbar(list_frame)
        scrollbar.pack(side='right', fill='y')
        lb = tk.Listbox(list_frame, font=self.f_button_sub, height=7,
                         activestyle='dotbox', selectbackground=NAV_ACCENT,
                         selectforeground='white', yscrollcommand=scrollbar.set,
                         exportselection=False)
        lb.pack(side='left', fill='both', expand=True)
        scrollbar.config(command=lb.yview)
        for i, n in enumerate(names):
            lb.insert('end', (n + '   (ล่าสุด)') if i == 0 else n)
        lb.selection_set(0)
        lb.activate(0)

        btn_row = tk.Frame(win, bg=CARD_BG)
        btn_row.pack(fill='x', padx=20, pady=16)

        def confirm(_e=None):
            sel = lb.curselection()
            if sel:
                result['path'] = yamls[sel[0]]
            win.destroy()

        def cancel(_e=None):
            win.destroy()

        tk.Button(btn_row, text='ยกเลิก', font=self.f_button_sub,
                  bg='#ffffff', fg=TEXT_MUTED, relief='solid', bd=1,
                  padx=14, pady=8, cursor='hand2', command=cancel
                  ).pack(side='right')
        tk.Button(btn_row, text='เลือกแผนที่นี้', font=self.f_button_sub,
                  bg=NAV_ACCENT, fg='white', activebackground=NAV_ACCENT_DARK,
                  relief='flat', padx=16, pady=8, cursor='hand2',
                  command=confirm).pack(side='right', padx=(0, 8))

        lb.bind('<Double-Button-1>', confirm)
        win.bind('<Return>', confirm)
        win.bind('<Escape>', cancel)
        win.protocol('WM_DELETE_WINDOW', cancel)

        self._center_toplevel(win, 400, 340)
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
        self._set_running_ui(False)
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
        self.move_distance_label.config(text=f'{new_value:.1f} ม.')
        self.btn_move_minus.config(
            state=('disabled' if new_value <= MOVE_FORWARD_MIN_M else 'normal'))
        self.btn_move_plus.config(
            state=('disabled' if new_value >= MOVE_FORWARD_MAX_M else 'normal'))

    def _adjust_move_speed(self, delta):
        new_value = round(self._move_speed + delta, 2)
        new_value = max(MOVE_FORWARD_SPEED_MIN, min(MOVE_FORWARD_SPEED_MAX, new_value))
        self._move_speed = new_value
        self.move_speed_label.config(text=f'{new_value:.2f} m/s')
        self.btn_speed_minus.config(
            state=('disabled' if new_value <= MOVE_FORWARD_SPEED_MIN else 'normal'))
        self.btn_speed_plus.config(
            state=('disabled' if new_value >= MOVE_FORWARD_SPEED_MAX else 'normal'))

    def _adjust_square_side(self, delta):
        new_value = round(self._square_side + delta, 2)
        new_value = max(SQUARE_SIDE_MIN, min(SQUARE_SIDE_MAX, new_value))
        self._square_side = new_value
        self.square_side_label.config(text=f'{new_value:.1f} ม./ด้าน')
        self.btn_square_minus.config(
            state=('disabled' if new_value <= SQUARE_SIDE_MIN else 'normal'))
        self.btn_square_plus.config(
            state=('disabled' if new_value >= SQUARE_SIDE_MAX else 'normal'))

    def _adjust_circle_radius(self, delta):
        new_value = round(self._circle_radius + delta, 2)
        new_value = max(CIRCLE_RADIUS_MIN, min(CIRCLE_RADIUS_MAX, new_value))
        self._circle_radius = new_value
        self.circle_radius_label.config(text=f'{new_value:.1f} ม.')
        self.btn_circle_radius_minus.config(
            state=('disabled' if new_value <= CIRCLE_RADIUS_MIN else 'normal'))
        self.btn_circle_radius_plus.config(
            state=('disabled' if new_value >= CIRCLE_RADIUS_MAX else 'normal'))

    def _adjust_circle_angle(self, delta):
        new_value = round(self._circle_angle + delta)
        new_value = max(CIRCLE_ANGLE_MIN, min(CIRCLE_ANGLE_MAX, new_value))
        self._circle_angle = new_value
        self.circle_angle_label.config(text=f'{new_value:.0f}°')
        self.btn_circle_angle_minus.config(
            state=('disabled' if new_value <= CIRCLE_ANGLE_MIN else 'normal'))
        self.btn_circle_angle_plus.config(
            state=('disabled' if new_value >= CIRCLE_ANGLE_MAX else 'normal'))

    def _cancel_move_forward(self):
        if self._move_proc is not None and self._move_proc.poll() is None:
            try:
                os.killpg(os.getpgid(self._move_proc.pid), signal.SIGINT)
            except Exception:
                pass

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

        self._move_kind = 'forward'
        self.btn_move_forward.config(text='ยกเลิก (Cancel)')
        self.btn_square_path.config(state='disabled')
        self.btn_circle_path.config(state='disabled')
        self._set_card_enabled(self.btn_mapping, False)
        self._set_card_enabled(self.btn_navigation, False)

        env = os.environ.copy()
        self._move_proc = subprocess.Popen(
            ['ros2', 'run', 'amr_bringup', 'move_forward_node.py',
             f'{distance:.3f}', f'{speed:.3f}'],
            preexec_fn=os.setsid, env=env,
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        self._poll_move_proc()

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
                'กรุณากด "สร้างแผนที่ (Mapping)" หรือ "นำทาง (Navigation)" '
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

        self._move_kind = 'square'
        self.btn_square_path.config(text='ยกเลิก (Cancel)')
        self.btn_move_forward.config(state='disabled')
        self.btn_circle_path.config(state='disabled')
        self._set_card_enabled(self.btn_mapping, False)
        self._set_card_enabled(self.btn_navigation, False)

        env = os.environ.copy()
        self._move_proc = subprocess.Popen(
            ['ros2', 'run', 'amr_bringup', 'square_path_node.py',
             f'{side:.3f}', f'{speed:.3f}', f'{SQUARE_TURN_SPEED:.3f}'],
            preexec_fn=os.setsid, env=env,
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        self._poll_move_proc()

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

        self._move_kind = 'circle'
        self.btn_circle_path.config(text='ยกเลิก (Cancel)')
        self.btn_move_forward.config(state='disabled')
        self.btn_square_path.config(state='disabled')
        self._set_card_enabled(self.btn_mapping, False)
        self._set_card_enabled(self.btn_navigation, False)

        env = os.environ.copy()
        self._move_proc = subprocess.Popen(
            ['ros2', 'run', 'amr_bringup', 'circle_path_node.py',
             f'{radius:.3f}', f'{angle:.1f}', f'{CIRCLE_ANGULAR_SPEED:.3f}'],
            preexec_fn=os.setsid, env=env, text=True,
            stdout=subprocess.PIPE, stderr=subprocess.DEVNULL)
        self._poll_move_proc()

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
        self.btn_move_forward.config(
            state=('normal' if running else 'disabled'),
            text='เดินหน้า (Move Forward)')
        self.btn_square_path.config(
            state=('normal' if running else 'disabled'),
            text='วิ่งเป็นสี่เหลี่ยม (Square Path)')
        self.btn_circle_path.config(
            state=('normal' if running else 'disabled'),
            text='วิ่งวงกลม (Circle Path)')
        self._set_card_enabled(self.btn_mapping, not running)
        self._set_card_enabled(self.btn_navigation, not running)

    def _show_save_name_dialog(self, default_name):
        result = {'name': None}
        win = tk.Toplevel(self)
        win.title('ตั้งชื่อแผนที่')
        win.configure(bg=CARD_BG)
        win.transient(self)
        win.resizable(False, False)

        tk.Label(win, text='ตั้งชื่อแผนที่', font=self.f_status,
                 bg=CARD_BG, fg=TEXT_MAIN).pack(padx=20, pady=(18, 8), anchor='w')

        entry_var = tk.StringVar(value=default_name)
        entry = tk.Entry(win, textvariable=entry_var, font=self.f_button_sub,
                          relief='solid', bd=1)
        entry.pack(padx=20, fill='x', ipady=6)

        hint = tk.Label(win, text='ใช้ตัวอักษร ตัวเลข - และ _ เท่านั้น',
                         font=self.f_small, bg=CARD_BG, fg=TEXT_MUTED, anchor='w')
        hint.pack(padx=20, pady=(6, 0), fill='x')

        btn_row = tk.Frame(win, bg=CARD_BG)
        btn_row.pack(fill='x', padx=20, pady=16)

        def confirm(_e=None):
            name = sanitize_map_name(entry_var.get())
            if not name:
                hint.config(text='กรุณาใส่ชื่อที่ถูกต้อง', fg=DANGER)
                return
            result['name'] = name
            win.destroy()

        def cancel(_e=None):
            win.destroy()

        tk.Button(btn_row, text='ยกเลิก', font=self.f_button_sub,
                  bg='#ffffff', fg=TEXT_MUTED, relief='solid', bd=1,
                  padx=14, pady=8, cursor='hand2', command=cancel
                  ).pack(side='right')
        tk.Button(btn_row, text='บันทึก', font=self.f_button_sub,
                  bg=ACCENT, fg='white', activebackground=ACCENT_DARK,
                  relief='flat', padx=16, pady=8, cursor='hand2',
                  command=confirm).pack(side='right', padx=(0, 8))

        win.bind('<Return>', confirm)
        win.bind('<Escape>', cancel)
        win.protocol('WM_DELETE_WINDOW', cancel)

        self._center_toplevel(win, 380, 190)
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
        self.btn_save_map.config(state='disabled', text='กำลังบันทึก...')

        def do_save():
            result = subprocess.run(
                ['ros2', 'run', 'nav2_map_server', 'map_saver_cli',
                 '-f', map_path, '--ros-args', '-p', 'save_map_timeout:=10.0'],
                capture_output=True, text=True, timeout=20)
            ok = result.returncode == 0 and os.path.exists(map_path + '.yaml')
            self.after(0, lambda: self._after_save(ok, name))
        threading.Thread(target=do_save, daemon=True).start()

    def _after_save(self, ok, name):
        self.btn_save_map.config(state='normal', text='บันทึกแผนที่ (Save Map)')
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
        self.destroy()


def main():
    app = AmrControlPanel()
    app.mainloop()


if __name__ == '__main__':
    main()
