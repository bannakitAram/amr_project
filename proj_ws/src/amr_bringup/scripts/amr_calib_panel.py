#!/usr/bin/env python3
"""GUI front-end for the localization-calibration tests
(calib_rotate_node.py, calib_straight_node.py) plus the per-wheel radius
calculator (calc_wheel_radius.py, section 3).

CalibPanel is a tk.Toplevel, not a tk.Tk root, so amr_control_panel.py can
open it as a popup window from its own "Calibrate Localization" button
(pass its Tk root as `master`) without creating a second Tk mainloop. It
can also still be run standalone via `ros2 run amr_bringup
amr_calib_panel.py` (main() below creates a hidden root to host it).

It only drives the two existing calibration nodes as subprocesses (same
Popen/poll/SIGINT-cancel pattern as amr_control_panel.py's "Move
forward"/"Square path" buttons) and re-implements their measurement math so
the operator can type the measured value into a field instead of a
terminal prompt. See calib_rotate_node.py and calib_straight_node.py's
module docstrings for the full procedure and the derivation of the
formulas duplicated below - keep both in sync if the formulas change there.

This tool only measures and displays a recommendation. It never edits
firmware or URDF files itself - the operator applies the change by hand,
re-flashes, and re-runs the test to confirm.
"""
import json
import queue
import threading
import math
import os
import signal
import subprocess
import tkinter as tk
from tkinter import messagebox, scrolledtext, ttk

import calc_wheel_radius

# Must match calib_rotate_node.py / calib_straight_node.py / the STM32
# firmware's `wheel_sep` / amr_description/urdf/amr.urdf.xacro's
# `wheel_separation` - keep all four in sync.
WHEEL_SEP_NOMINAL = 0.472

ROTATE_TURNS_DEFAULT = 2.0
ROTATE_SPEED_DEFAULT = 0.3
STRAIGHT_DISTANCE_DEFAULT = 3.0
STRAIGHT_SPEED_DEFAULT = 0.15
# Must match wheel_radius_l/_r in the STM32 firmware (nominal 0.128 m dia).
WHEEL_RADIUS_NOMINAL = 0.064
# Must match GYRO_SCALE_Z in the STM32 firmware (1.0 = uncalibrated).
GYRO_SCALE_Z_NOMINAL = 1.0084
GYRO_MOTOR_TURNS_DEFAULT = 5.0
# Section 5 (accel_slip_test_node.py). Decel stays at the smoother's
# max_decel so the stop distance during the test is the normal one.
ACCEL_LIST_DEFAULT = '0.1,0.2,0.3,0.4,0.5'
ACCEL_VMAX_DEFAULT = 0.2
ACCEL_DECEL_DEFAULT = 0.5
ACCEL_REPS_DEFAULT = 2
# Rotate mode: spin in place; alpha in rad/s^2 (STM32 ramp caps ~2.1),
# decel = smoother max_decel[2], vmax = MPPI wz_max.
ROT_LIST_DEFAULT = '0.5,1.0,1.5,2.0'
ROT_WMAX_DEFAULT = 0.5
ROT_DECEL_DEFAULT = 1.5

SAFETY_BANNER = (
    'ตรวจสอบให้พื้นที่รอบหุ่นยนต์โล่ง ไม่มีคน/สิ่งกีดขวางตลอดเส้นทางทดสอบ '
    'และเตรียม E-stop ให้พร้อมกดตลอดการทดสอบ (ISO 3691-4)')


class CalibPanel(tk.Toplevel):
    def __init__(self, master, on_close=None):
        super().__init__(master)
        self.title('AMR Localization Calibration')
        self.transient(master)
        self._on_close_extra = on_close

        self._proc = None
        self._gyro_proc = None  # gyro_integrate_node.py, runs alongside the rotate test
        self._last_gyro_deg = None
        self._poll_job = None
        self._active_test = None  # 'rotate', 'straight', 'gyro', 'gyro_motor', 'accel'
        self._last_commanded_deg = None
        self._last_commanded_distance = None

        # This panel's content (two sections + a scrolled result box each)
        # is naturally taller than the 1024x600 kiosk panel it's opened on
        # top of (see amr_control_panel.py's geometry('1024x600')), so it
        # has to fit itself into whatever screen is actually available and
        # let the operator scroll to reach anything that doesn't fit,
        # rather than using a fixed size that runs off-screen and leaves
        # controls (like the rotate Start button) unreachable.
        screen_w = self.winfo_screenwidth()
        screen_h = self.winfo_screenheight()
        win_w = min(640, screen_w - 20)
        win_h = min(720, screen_h - 40)
        pos_x = max((screen_w - win_w) // 2, 0)
        pos_y = max((screen_h - win_h) // 2, 0)
        self.geometry(f'{win_w}x{win_h}+{pos_x}+{pos_y}')
        self.minsize(400, 300)

        outer = tk.Frame(self)
        outer.pack(fill='both', expand=True)
        canvas = tk.Canvas(outer, highlightthickness=0)
        vbar = tk.Scrollbar(outer, orient='vertical', command=canvas.yview)
        canvas.configure(yscrollcommand=vbar.set)
        canvas.pack(side='left', fill='both', expand=True)
        vbar.pack(side='right', fill='y')

        content = tk.Frame(canvas)
        content_id = canvas.create_window((0, 0), window=content, anchor='nw')

        def on_content_configure(_e=None):
            canvas.configure(scrollregion=canvas.bbox('all'))

        def on_canvas_configure(e):
            canvas.itemconfigure(content_id, width=e.width)

        content.bind('<Configure>', on_content_configure)
        canvas.bind('<Configure>', on_canvas_configure)

        def on_mousewheel(e):
            canvas.yview_scroll(int(-1 * (e.delta / 120)), 'units')

        canvas.bind_all('<MouseWheel>', on_mousewheel)
        canvas.bind_all('<Button-4>', lambda e: canvas.yview_scroll(-1, 'units'))
        canvas.bind_all('<Button-5>', lambda e: canvas.yview_scroll(1, 'units'))

        self._content = content

        banner = tk.Label(
            content, text=SAFETY_BANNER, fg='white', bg='#c0392b',
            wraplength=win_w - 40, justify='left',
            font=('TkDefaultFont', 10, 'bold'), padx=10, pady=8)
        banner.pack(fill='x')

        self._build_rotate_section()
        ttk.Separator(content, orient='horizontal').pack(fill='x', pady=6)
        self._build_straight_section()
        ttk.Separator(content, orient='horizontal').pack(fill='x', pady=6)
        self._build_radius_section()
        ttk.Separator(content, orient='horizontal').pack(fill='x', pady=6)
        self._build_gyro_section()
        ttk.Separator(content, orient='horizontal').pack(fill='x', pady=6)
        self._build_accel_section()

        self.protocol('WM_DELETE_WINDOW', self._on_close)

    # ------------------------------------------------------------------
    # Rotation test (wheel_sep)
    # ------------------------------------------------------------------
    def _build_rotate_section(self):
        frame = tk.LabelFrame(
            self._content, text='1) ทดสอบหมุนอยู่กับที่ - หา Wheel Separation (track width)',
            padx=10, pady=10)
        frame.pack(fill='x', padx=10, pady=6)

        row = tk.Frame(frame)
        row.pack(fill='x', pady=2)
        tk.Label(row, text='จำนวนรอบ:').pack(side='left')
        self.rotate_turns_var = tk.StringVar(value=str(ROTATE_TURNS_DEFAULT))
        tk.Entry(row, textvariable=self.rotate_turns_var, width=8).pack(side='left', padx=4)
        tk.Label(row, text='ความเร็วหมุน (rad/s):').pack(side='left', padx=(12, 0))
        self.rotate_speed_var = tk.StringVar(value=str(ROTATE_SPEED_DEFAULT))
        tk.Entry(row, textvariable=self.rotate_speed_var, width=8).pack(side='left', padx=4)

        self.btn_rotate = tk.Button(
            frame, text='เริ่มหมุน (Start)', bg='#2980b9', fg='white',
            command=self._on_click_rotate)
        self.btn_rotate.pack(fill='x', pady=6)

        self.rotate_status = tk.Label(frame, text='รอเริ่มการทดสอบ', anchor='w')
        self.rotate_status.pack(fill='x')

        row2 = tk.Frame(frame)
        row2.pack(fill='x', pady=(8, 2))
        tk.Label(row2, text='มุมที่วัดได้จริง (deg):').pack(side='left')
        self.rotate_measured_var = tk.StringVar()
        self.rotate_measured_entry = tk.Entry(
            row2, textvariable=self.rotate_measured_var, width=10, state='disabled')
        self.rotate_measured_entry.pack(side='left', padx=4)

        self.btn_rotate_calc = tk.Button(
            frame, text='คำนวณค่าแก้ไข', state='disabled',
            command=self._on_calc_rotate)
        self.btn_rotate_calc.pack(fill='x', pady=4)

        self.rotate_result = scrolledtext.ScrolledText(frame, height=6, wrap='word')
        self.rotate_result.pack(fill='x')
        self.rotate_result.config(state='disabled')

    def _on_click_rotate(self):
        if self._proc is not None and self._proc.poll() is None:
            self._cancel_active()
            return
        if self._active_test == 'gyro':
            messagebox.showerror('กำลังบันทึก gyro', 'หยุดบันทึก gyro (หัวข้อ 4) ก่อน')
            return

        try:
            turns = float(self.rotate_turns_var.get())
            speed = float(self.rotate_speed_var.get())
        except ValueError:
            messagebox.showerror('ค่าไม่ถูกต้อง', 'กรอกจำนวนรอบ/ความเร็วเป็นตัวเลข')
            return

        if not messagebox.askyesno(
                'ยืนยันการทดสอบ',
                f'ให้หุ่นยนต์หมุนอยู่กับที่ {turns:g} รอบ '
                f'ด้วยความเร็ว {speed:.2f} rad/s ใช่หรือไม่?\n\n' + SAFETY_BANNER):
            return

        self._reset_result(self.rotate_result)
        self.rotate_measured_entry.config(state='disabled')
        self.btn_rotate_calc.config(state='disabled')
        self.rotate_measured_var.set('')
        self._last_commanded_deg = 360.0 * turns
        self._last_gyro_deg = None

        self.btn_rotate.config(text='ยกเลิก (Cancel)')
        self.rotate_status.config(text='กำลังหมุน...')
        env = os.environ.copy()
        self._active_test = 'rotate'
        # Integrate /imu/data yaw over the same rotation so the gyro's scale
        # can be checked against the measured angle too (see
        # gyro_integrate_node.py). Started first so it is subscribed before
        # the robot moves; stopped with SIGINT when the rotate node exits.
        self._gyro_proc = subprocess.Popen(
            ['ros2', 'run', 'amr_bringup', 'gyro_integrate_node.py'],
            preexec_fn=os.setsid, env=env,
            stdin=subprocess.DEVNULL, stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL, text=True)
        self._proc = subprocess.Popen(
            ['ros2', 'run', 'amr_bringup', 'calib_rotate_node.py',
             f'{turns:.4f}', f'{speed:.3f}'],
            preexec_fn=os.setsid, env=env,
            stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL)
        self._poll_proc()

    def _on_calc_rotate(self):
        try:
            measured_deg = float(self.rotate_measured_var.get())
        except ValueError:
            messagebox.showerror('ค่าไม่ถูกต้อง', 'กรอกมุมที่วัดได้เป็นตัวเลข (deg)')
            return
        if abs(measured_deg) < 1e-6:
            messagebox.showerror('ค่าไม่ถูกต้อง', 'มุมที่วัดได้ต้องไม่เป็น 0')
            return

        commanded_deg = self._last_commanded_deg
        corrected = WHEEL_SEP_NOMINAL * commanded_deg / measured_deg

        gyro_text = ''
        gyro_deg = self._last_gyro_deg
        if gyro_deg is not None and abs(gyro_deg) > 1.0:
            gyro_err = (abs(gyro_deg) - abs(measured_deg)) / abs(measured_deg) * 100.0
            gyro_text = (
                f'\nมุมจาก gyro (/imu/data): {abs(gyro_deg):.1f} deg '
                f'(error {gyro_err:+.2f}% เทียบมุมจริง)\n'
                f'gyro_z_scale ที่แนะนำ: {abs(measured_deg) / abs(gyro_deg):.4f}\n'
                '(error เกิน ±2% ควรแก้ scale ของ gyro z ใน firmware)\n')
        elif gyro_deg is not None:
            gyro_text = '\ngyro: ไม่พบการหมุนจาก /imu/data - ตรวจสอบ IMU\n'

        text = (
            f'มุมที่สั่ง (raw /wheel/odom): {commanded_deg:.1f} deg\n'
            f'มุมที่วัดได้จริง: {measured_deg:.1f} deg\n\n'
            f'wheel_sep ปัจจุบัน: {WHEEL_SEP_NOMINAL:.4f} m\n'
            f'wheel_sep ที่แก้ไขแล้ว: {corrected:.4f} m\n'
            f'{gyro_text}\n'
            'ถ้าค่านี้ต่างจากค่าปัจจุบันอย่างมีนัยสำคัญ ให้แก้ไขทั้ง 3 จุดพร้อมกัน:\n'
            '  - stm/src/motor_control_with_pid_optimized.cpp: `wheel_sep`\n'
            '  - amr_description/urdf/amr.urdf.xacro: `wheel_separation`\n'
            '  - ค่า WHEEL_SEP_NOMINAL ในสคริปต์ calibrate ทั้งหมด\n'
            'จากนั้น re-flash STM32 แล้วรันทดสอบนี้ซ้ำเพื่อยืนยันก่อนใช้งานจริง')
        self._set_result(self.rotate_result, text)

    # ------------------------------------------------------------------
    # Straight-line test (wheel speed ratio)
    # ------------------------------------------------------------------
    def _build_straight_section(self):
        frame = tk.LabelFrame(
            self._content, text='2) ทดสอบวิ่งตรง - หาอัตราส่วนความเร็วล้อซ้าย/ขวา',
            padx=10, pady=10)
        frame.pack(fill='both', expand=True, padx=10, pady=6)

        row = tk.Frame(frame)
        row.pack(fill='x', pady=2)
        tk.Label(row, text='ระยะทาง (m):').pack(side='left')
        self.straight_distance_var = tk.StringVar(value=str(STRAIGHT_DISTANCE_DEFAULT))
        tk.Entry(row, textvariable=self.straight_distance_var, width=8).pack(side='left', padx=4)
        tk.Label(row, text='ความเร็ว (m/s):').pack(side='left', padx=(12, 0))
        self.straight_speed_var = tk.StringVar(value=str(STRAIGHT_SPEED_DEFAULT))
        tk.Entry(row, textvariable=self.straight_speed_var, width=8).pack(side='left', padx=4)

        self.btn_straight = tk.Button(
            frame, text='เริ่มวิ่งตรง (Start)', bg='#2980b9', fg='white',
            command=self._on_click_straight)
        self.btn_straight.pack(fill='x', pady=6)

        self.straight_status = tk.Label(frame, text='รอเริ่มการทดสอบ', anchor='w')
        self.straight_status.pack(fill='x')

        row2 = tk.Frame(frame)
        row2.pack(fill='x', pady=(8, 2))
        tk.Label(row2, text='ระยะเยื้องที่วัดได้ (m, + = เยื้องขวา):').pack(side='left')
        self.straight_offset_var = tk.StringVar()
        self.straight_offset_entry = tk.Entry(
            row2, textvariable=self.straight_offset_var, width=10, state='disabled')
        self.straight_offset_entry.pack(side='left', padx=4)

        row3 = tk.Frame(frame)
        row3.pack(fill='x', pady=2)
        tk.Label(row3, text='ระยะจริงที่วิ่งได้ (m, เว้นว่าง = ใช้ค่าที่สั่ง):').pack(side='left')
        self.straight_real_distance_var = tk.StringVar()
        self.straight_real_distance_entry = tk.Entry(
            row3, textvariable=self.straight_real_distance_var, width=10, state='disabled')
        self.straight_real_distance_entry.pack(side='left', padx=4)

        self.btn_straight_calc = tk.Button(
            frame, text='คำนวณค่าแก้ไข', state='disabled',
            command=self._on_calc_straight)
        self.btn_straight_calc.pack(fill='x', pady=4)

        self.straight_result = scrolledtext.ScrolledText(frame, height=10, wrap='word')
        self.straight_result.pack(fill='both', expand=True)
        self.straight_result.config(state='disabled')

    def _on_click_straight(self):
        if self._proc is not None and self._proc.poll() is None:
            self._cancel_active()
            return
        if self._active_test == 'gyro':
            messagebox.showerror('กำลังบันทึก gyro', 'หยุดบันทึก gyro (หัวข้อ 4) ก่อน')
            return

        try:
            distance = float(self.straight_distance_var.get())
            speed = float(self.straight_speed_var.get())
        except ValueError:
            messagebox.showerror('ค่าไม่ถูกต้อง', 'กรอกระยะทาง/ความเร็วเป็นตัวเลข')
            return

        if not messagebox.askyesno(
                'ยืนยันการทดสอบ',
                f'ให้หุ่นยนต์วิ่งตรง {distance:.2f} เมตร '
                f'ด้วยความเร็ว {speed:.2f} m/s ใช่หรือไม่?\n\n' + SAFETY_BANNER):
            return

        self._reset_result(self.straight_result)
        self.straight_offset_entry.config(state='disabled')
        self.straight_real_distance_entry.config(state='disabled')
        self.btn_straight_calc.config(state='disabled')
        self.straight_offset_var.set('')
        self.straight_real_distance_var.set('')
        self._last_commanded_distance = distance

        self.btn_straight.config(text='ยกเลิก (Cancel)')
        self.straight_status.config(text='กำลังวิ่งตรง...')
        env = os.environ.copy()
        self._active_test = 'straight'
        self._proc = subprocess.Popen(
            ['ros2', 'run', 'amr_bringup', 'calib_straight_node.py',
             f'{distance:.3f}', f'{speed:.3f}'],
            preexec_fn=os.setsid, env=env,
            stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL)
        self._poll_proc()

    def _on_calc_straight(self):
        try:
            offset_m = float(self.straight_offset_var.get())
        except ValueError:
            messagebox.showerror('ค่าไม่ถูกต้อง', 'กรอกระยะเยื้องเป็นตัวเลข (m)')
            return
        real_distance_str = self.straight_real_distance_var.get().strip()
        try:
            real_distance = (float(real_distance_str) if real_distance_str
                              else self._last_commanded_distance)
        except ValueError:
            messagebox.showerror('ค่าไม่ถูกต้อง', 'กรอกระยะจริงเป็นตัวเลข (m)')
            return
        if real_distance <= 0:
            messagebox.showerror('ค่าไม่ถูกต้อง', 'ระยะจริงต้องมากกว่า 0')
            return

        dtheta = 2.0 * offset_m / real_distance
        speed_error_ratio = dtheta * WHEEL_SEP_NOMINAL / real_distance
        right_scale_factor = 1.0 - speed_error_ratio
        half_step_factor = 1.0 + (right_scale_factor - 1.0) * 0.5

        text = (
            f'มุมเบี่ยงเบนโดยประมาณ: {dtheta:.4f} rad ({math.degrees(dtheta):.2f} deg)\n\n'
            f'Right-wheel scale factor (แก้เต็ม): {right_scale_factor:.4f}\n'
            f'ค่าแนะนำให้ลองก่อน (half-step): {half_step_factor:.4f}\n\n'
            'ให้แก้ตรงไหนขึ้นกับลักษณะการเบี่ยง:\n'
            '  - เบี่ยงแค่ตอนออกตัวครั้งเดียวแล้ววิ่งตรง: เป็น transient ตอนเร่ง '
            '- ไปจูน launch-boost/startup-balance constants ใน\n'
            '    stm/src/motor_control_with_pid_optimized.cpp แทนการแก้ DAC table\n'
            '  - เบี่ยงเพิ่มขึ้นเรื่อยๆ ตลอดเส้นทาง: เป็น steady-state error '
            '(PID integral แก้เองไม่ได้) มักมาจากเส้นผ่านศูนย์กลางล้อจริงต่างกัน '
            '- ต้องเพิ่ม scale factor ที่จุดแปลง distance/speed ไม่ใช่แก้ DAC table '
            '- ควรให้วิศวกร embedded ตรวจสอบก่อนเพิ่มค่าคงที่ใหม่ในเฟิร์มแวร์\n\n'
            'Re-flash แล้วรันทดสอบนี้ซ้ำ ทำซ้ำจนระยะเยื้องเล็กและไม่ลดลงอีก')
        self._set_result(self.straight_result, text)

        # Pre-fill section 3 with this run (forward; the node closes the
        # loop on /wheel/raw_distance, so odom distance == commanded).
        self.radius_dir_var.set('fwd')
        self.radius_real_var.set(f'{real_distance:g}')
        self.radius_odom_var.set(f'{self._last_commanded_distance:g}')
        self.radius_offset_var.set(f'{offset_m:g}')

    # ------------------------------------------------------------------
    # Wheel radius (left/right) from several straight runs
    # ------------------------------------------------------------------
    def _build_radius_section(self):
        self._radius_runs = []
        frame = tk.LabelFrame(
            self._content,
            text='3) คำนวณรัศมีล้อซ้าย/ขวา (wheel_radius_l / wheel_radius_r)',
            padx=10, pady=10)
        frame.pack(fill='x', padx=10, pady=6)

        tk.Label(
            frame, anchor='w', justify='left', wraplength=560,
            text='ทำหัวข้อ 2 ทั้งวิ่งไปข้างหน้า (fwd) และถอยหลัง/กลับทิศ (rev) '
                 'อย่างน้อยทิศละ 2 รอบ ระยะ >= 3 m แล้วเพิ่มเป็นรอบวัดที่นี่ '
                 '(ผลจากหัวข้อ 2 จะถูกเติมลงช่องด้านล่างอัตโนมัติ) '
                 'ระยะเยื้อง: + = เยื้องไปทางขวาของทิศที่รถเคลื่อนที่').pack(fill='x')

        row = tk.Frame(frame)
        row.pack(fill='x', pady=4)
        tk.Label(row, text='ทิศ:').pack(side='left')
        self.radius_dir_var = tk.StringVar(value='fwd')
        ttk.Combobox(row, textvariable=self.radius_dir_var, values=('fwd', 'rev'),
                     width=5, state='readonly').pack(side='left', padx=4)
        self.radius_real_var = tk.StringVar()
        self.radius_odom_var = tk.StringVar()
        self.radius_offset_var = tk.StringVar()
        for label, var in (('ระยะจริง (m):', self.radius_real_var),
                           ('ระยะ odom (m):', self.radius_odom_var),
                           ('ระยะเยื้อง (m):', self.radius_offset_var)):
            tk.Label(row, text=label).pack(side='left', padx=(8, 0))
            tk.Entry(row, textvariable=var, width=7).pack(side='left', padx=2)

        btns = tk.Frame(frame)
        btns.pack(fill='x', pady=2)
        tk.Button(btns, text='เพิ่มรอบวัด', command=self._on_radius_add).pack(
            side='left', padx=2)
        tk.Button(btns, text='ลบรอบล่าสุด', command=self._on_radius_undo).pack(
            side='left', padx=2)
        tk.Button(btns, text='ล้างทั้งหมด', command=self._on_radius_clear).pack(
            side='left', padx=2)

        self.radius_list = tk.Listbox(frame, height=5)
        self.radius_list.pack(fill='x', pady=4)

        row2 = tk.Frame(frame)
        row2.pack(fill='x', pady=2)
        tk.Label(row2, text='radius_l ปัจจุบัน (m):').pack(side='left')
        self.radius_l_var = tk.StringVar(value=f'{WHEEL_RADIUS_NOMINAL:.5f}')
        tk.Entry(row2, textvariable=self.radius_l_var, width=9).pack(side='left', padx=4)
        tk.Label(row2, text='radius_r:').pack(side='left', padx=(8, 0))
        self.radius_r_var = tk.StringVar(value=f'{WHEEL_RADIUS_NOMINAL:.5f}')
        tk.Entry(row2, textvariable=self.radius_r_var, width=9).pack(side='left', padx=4)
        tk.Label(row2, text='trim gain:').pack(side='left', padx=(8, 0))
        self.radius_gain_var = tk.StringVar(value='0.5')
        tk.Entry(row2, textvariable=self.radius_gain_var, width=5).pack(side='left', padx=4)

        tk.Button(frame, text='คำนวณรัศมีล้อ', command=self._on_radius_calc).pack(
            fill='x', pady=4)
        self.radius_result = scrolledtext.ScrolledText(frame, height=12, wrap='word')
        self.radius_result.pack(fill='x')
        self.radius_result.config(state='disabled')

    def _refresh_radius_list(self):
        self.radius_list.delete(0, 'end')
        for i, (d, real, odom, y) in enumerate(self._radius_runs, 1):
            self.radius_list.insert(
                'end', f'{i}. {d}  จริง {real:g} m  odom {odom:g} m  เยื้อง {y:+g} m')

    def _on_radius_add(self):
        try:
            real = float(self.radius_real_var.get())
            odom = float(self.radius_odom_var.get())
            y = float(self.radius_offset_var.get())
        except ValueError:
            messagebox.showerror('ค่าไม่ถูกต้อง', 'กรอกระยะจริง/odom/เยื้องเป็นตัวเลข')
            return
        if real <= 0 or odom <= 0:
            messagebox.showerror('ค่าไม่ถูกต้อง', 'ระยะจริงและระยะ odom ต้องมากกว่า 0')
            return
        self._radius_runs.append((self.radius_dir_var.get(), real, odom, y))
        self._refresh_radius_list()

    def _on_radius_undo(self):
        if self._radius_runs:
            self._radius_runs.pop()
            self._refresh_radius_list()

    def _on_radius_clear(self):
        self._radius_runs.clear()
        self._refresh_radius_list()
        self._reset_result(self.radius_result)

    def _on_radius_calc(self):
        if not self._radius_runs:
            messagebox.showerror('ไม่มีข้อมูล', 'เพิ่มรอบวัดอย่างน้อย 1 รอบก่อน')
            return
        try:
            rl = float(self.radius_l_var.get())
            rr = float(self.radius_r_var.get())
            gain = float(self.radius_gain_var.get())
        except ValueError:
            messagebox.showerror('ค่าไม่ถูกต้อง', 'กรอก radius/trim gain เป็นตัวเลข')
            return
        if rl <= 0 or rr <= 0 or not 0.0 <= gain <= 1.0:
            messagebox.showerror('ค่าไม่ถูกต้อง', 'radius ต้อง > 0 และ trim gain อยู่ในช่วง 0-1')
            return

        res = calc_wheel_radius.compute(
            self._radius_runs, rl, rr, WHEEL_SEP_NOMINAL, gain)
        s, t = res['scale'], res['mismatch']
        lines = [
            f'Scale เฉลี่ย (ทั้งสองล้อ): {s:.4f} ({(s - 1) * 100:+.2f}%)',
            f'ส่วนต่างล้อซ้าย-ขวา (L-R)/D: {t * 100:+.3f}% (ใช้ x{gain:g} รอบนี้)',
            '',
            f'wheel_radius_l: {rl:.5f} -> {res["new_l"]:.5f} m',
            f'wheel_radius_r: {rr:.5f} -> {res["new_r"]:.5f} m',
            '',
            'แก้ใน stm/src/motor_control_with_pid_optimized.cpp:',
            f'const float wheel_radius_l = {res["new_l"]:.5f}f;',
            f'const float wheel_radius_r = {res["new_r"]:.5f}f;',
        ]
        if res['warnings']:
            lines += ['', 'คำเตือน:'] + [f'  - {w}' for w in res['warnings']]
        lines += [
            '',
            'ห้ามแก้ ENCODER_PPR เพื่อชดเชย ให้แก้เฉพาะรัศมีล้อ แล้ว re-flash, '
            'ทดสอบความเร็วต่ำ (เตรียม E-stop) และวัดซ้ำจนระยะผิด <1% '
            'และเยื้อง <2 cm/5 m']
        self._set_result(self.radius_result, '\n'.join(lines))

    # ------------------------------------------------------------------
    # Gyro scale by hand rotation (independent of wheel slip)
    # ------------------------------------------------------------------
    def _build_gyro_section(self):
        frame = tk.LabelFrame(
            self._content, text='4) Gyro scale (หมุนด้วยมือ หรือ มอเตอร์หมุนเอง)',
            padx=10, pady=10)
        frame.pack(fill='x', padx=10, pady=6)

        mode_row = tk.Frame(frame)
        mode_row.pack(fill='x', pady=2)
        tk.Label(mode_row, text='โหมด:').pack(side='left')
        self.gyro_mode_var = tk.StringVar(value='hand')
        for value, label in (('hand', 'หมุนด้วยมือ'), ('motor', 'มอเตอร์หมุนเอง')):
            tk.Radiobutton(
                mode_row, text=label, value=value, variable=self.gyro_mode_var,
                command=self._on_gyro_mode_change).pack(side='left', padx=4)

        self.gyro_help = tk.Label(frame, anchor='w', justify='left', wraplength=560)
        self.gyro_help.pack(fill='x')

        # Motor mode only: commanded via calib_rotate_node.py (stops on
        # /wheel/odom yaw, so the real angle must still be measured by eye).
        self.gyro_motor_row = tk.Frame(frame)
        tk.Label(self.gyro_motor_row, text='จำนวนรอบ:').pack(side='left')
        self.gyro_turns_var = tk.StringVar(value=str(GYRO_MOTOR_TURNS_DEFAULT))
        tk.Entry(self.gyro_motor_row, textvariable=self.gyro_turns_var, width=6).pack(side='left', padx=4)
        tk.Label(self.gyro_motor_row, text='ความเร็ว (rad/s):').pack(side='left', padx=(8, 0))
        self.gyro_speed_var = tk.StringVar(value=str(ROTATE_SPEED_DEFAULT))
        tk.Entry(self.gyro_motor_row, textvariable=self.gyro_speed_var, width=6).pack(side='left', padx=4)
        self.gyro_dir_var = tk.StringVar(value='ccw')
        for value, label in (('ccw', 'CCW'), ('cw', 'CW')):
            tk.Radiobutton(
                self.gyro_motor_row, text=label, value=value,
                variable=self.gyro_dir_var).pack(side='left', padx=2)

        self.btn_gyro = tk.Button(
            frame, text='เริ่มบันทึก gyro', bg='#2980b9', fg='white',
            command=self._on_click_gyro)
        self.btn_gyro.pack(fill='x', pady=6)
        self._gyro_btn_anchor = self.btn_gyro
        self.gyro_status = tk.Label(frame, text='รอเริ่มบันทึก', anchor='w')
        self.gyro_status.pack(fill='x')

        # Real angle from a wall seen by the LiDAR (scan_lines_node.py):
        # independent of wheel slip and of the robot drifting off its spot.
        lidar = tk.LabelFrame(
            frame, text='วัดมุมจริงด้วย LiDAR (ไม่ต้องวัดด้วยมือ - ไม่ขึ้นกับการไถล)',
            padx=6, pady=4)
        lidar.pack(fill='x', pady=4)
        tk.Label(
            lidar, anchor='w', justify='left', wraplength=540,
            text='ต้องเห็นผนังตรงยาว ≥ 0.8 m ห่าง 0.3-6 m  รถต้องนิ่งตอนกดวัด\n'
                 'ลำดับ: วัดผนัง (ก่อน) -> เริ่มบันทึก/หมุน -> หยุด -> วัดผนัง (หลัง) -> มุมจริงจะเติมให้อัตโนมัติ'
        ).pack(fill='x')
        lrow = tk.Frame(lidar)
        lrow.pack(fill='x', pady=2)
        self.btn_wall_before = tk.Button(
            lrow, text='วัดผนัง (ก่อนหมุน)', command=lambda: self._measure_wall('before'))
        self.btn_wall_before.pack(side='left', expand=True, fill='x', padx=2)
        self.btn_wall_after = tk.Button(
            lrow, text='วัดผนัง (หลังหมุน)', command=lambda: self._measure_wall('after'))
        self.btn_wall_after.pack(side='left', expand=True, fill='x', padx=2)
        self.wall_status = tk.Label(lidar, text='ยังไม่ได้วัด', anchor='w', justify='left')
        self.wall_status.pack(fill='x')
        self._wall_before = None   # list of lines from scan_lines_node.py
        self._wall_after = None
        self._wall_proc = None

        row = tk.Frame(frame)
        row.pack(fill='x', pady=2)
        tk.Label(row, text='มุมจริง (deg):').pack(side='left')
        self.gyro_real_var = tk.StringVar(value='1800')
        tk.Entry(row, textvariable=self.gyro_real_var, width=8).pack(side='left', padx=4)
        tk.Label(row, text='GYRO_SCALE_Z ปัจจุบัน:').pack(side='left', padx=(12, 0))
        self.gyro_scale_var = tk.StringVar(value=f'{GYRO_SCALE_Z_NOMINAL:.4f}')
        tk.Entry(row, textvariable=self.gyro_scale_var, width=8).pack(side='left', padx=4)

        self.btn_gyro_calc = tk.Button(
            frame, text='คำนวณ gyro scale', state='disabled',
            command=self._on_calc_gyro)
        self.btn_gyro_calc.pack(fill='x', pady=4)
        self.gyro_result = scrolledtext.ScrolledText(frame, height=7, wrap='word')
        self.gyro_result.pack(fill='x')
        self.gyro_result.config(state='disabled')
        self._hand_gyro_deg = None
        self._wall_after = None  # a new run needs a fresh "after" wall
        self._on_gyro_mode_change()

    def _on_gyro_mode_change(self):
        if self._active_test in ('gyro', 'gyro_motor'):
            return
        if self.gyro_mode_var.get() == 'motor':
            self.gyro_help.config(
                text='1. ทำเครื่องหมายบนพื้นและตัวรถ เคลียร์พื้นที่รอบรถ ให้รถนิ่ง 10 วินาที\n'
                     '2. กดเริ่ม รถจะหมุนอยู่กับที่ตามจำนวนรอบ (หยุดด้วย wheel odom จึงอาจไม่ตรงเครื่องหมายพอดี)\n'
                     '3. วัดมุมจริงจากเครื่องหมาย (รอบเต็ม ± มุมที่ขาด/เกิน) กรอกแล้วกดคำนวณ  ทำทั้ง CW และ CCW')
            self.gyro_motor_row.pack(fill='x', pady=2, before=self._gyro_btn_anchor)
            self.btn_gyro.config(text='เริ่มหมุนด้วยมอเตอร์ (Start)')
        else:
            self.gyro_help.config(
                text='1. กด E-stop/ปิดมอเตอร์ ทำเครื่องหมายบนพื้นและตัวรถ ให้รถนิ่ง 10 วินาที\n'
                     '2. กดเริ่มบันทึก แล้วเข็นหมุนรถรอบตัวเองให้ครบหลายรอบ (เช่น 5 รอบ = 1800°) '
                     'จนเครื่องหมายตรงกันพอดี\n'
                     '3. กดหยุดบันทึก กรอกมุมจริงแล้วกดคำนวณ  ทำทั้ง CW และ CCW แล้วใช้ค่าเฉลี่ย\n'
                     'หมายเหตุ: ถ้าล้อไม่หมุนขณะเข็น (ยกล้อ/ล้อไถล) ZUPT ใน firmware อาจกลืนมุม - ใช้โหมดมอเตอร์แทน')
            self.gyro_motor_row.pack_forget()
            self.btn_gyro.config(text='เริ่มบันทึก gyro')

    def _start_gyro_proc(self):
        self._gyro_proc = subprocess.Popen(
            ['ros2', 'run', 'amr_bringup', 'gyro_integrate_node.py'],
            preexec_fn=os.setsid, env=os.environ.copy(),
            stdin=subprocess.DEVNULL, stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL, text=True)

    def _finish_gyro(self, deg):
        self._hand_gyro_deg = deg
        if deg is None:
            self.gyro_status.config(text='ไม่ได้รับข้อมูล /imu/data - ตรวจสอบ bringup')
            return
        self.gyro_status.config(text=f'บันทึกเสร็จ: gyro วัดได้ {deg:+.1f} deg (CCW = +)')
        self.btn_gyro_calc.config(state='normal')
        self._update_wall_real_angle()

    # ------------------------------------------------------------------
    # Real rotation angle from a LiDAR wall (scan_lines_node.py)
    # ------------------------------------------------------------------
    def _measure_wall(self, which):
        if self._wall_proc is not None:
            return
        if self._active_test is not None:
            messagebox.showerror('กำลังทดสอบอยู่', 'วัดผนังได้เฉพาะตอนรถนิ่ง (ก่อนเริ่ม/หลังหยุดบันทึก)')
            return
        if which == 'before':
            self._wall_before = None
            self._wall_after = None
        elif self._wall_before is None:
            messagebox.showerror('ยังไม่ได้วัดก่อนหมุน', 'กด "วัดผนัง (ก่อนหมุน)" ก่อนเริ่มหมุน')
            return
        self.wall_status.config(text='กำลังวัดผนัง... (รถต้องนิ่ง)')
        self.btn_wall_before.config(state='disabled')
        self.btn_wall_after.config(state='disabled')
        self._wall_proc = subprocess.Popen(
            ['ros2', 'run', 'amr_bringup', 'scan_lines_node.py', '10'],
            preexec_fn=os.setsid, env=os.environ.copy(),
            stdin=subprocess.DEVNULL, stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL, text=True)
        self._wall_which = which
        self.after(200, self._poll_wall)

    def _poll_wall(self):
        proc = self._wall_proc
        if proc is None:
            return
        if proc.poll() is None:
            self.after(200, self._poll_wall)
            return
        self._wall_proc = None
        self.btn_wall_before.config(state='normal')
        self.btn_wall_after.config(state='normal')
        out = proc.stdout.read() if proc.stdout else ''
        lines = None
        for line in out.splitlines():
            if line.startswith('LINES '):
                try:
                    lines = json.loads(line[len('LINES '):])
                except ValueError:
                    lines = None
        if proc.returncode == 2:
            self.wall_status.config(text='ไม่ได้รับข้อมูล /scan - ตรวจสอบ LiDAR/bringup')
            return
        if not lines:
            self.wall_status.config(text='ไม่พบผนังตรง (ยาว ≥ 0.8 m ห่าง 0.3-6 m) - ย้ายรถให้เห็นผนัง')
            return
        if self._wall_which == 'before':
            self._wall_before = lines
            w = lines[0]
            self.wall_status.config(
                text=f'ก่อนหมุน: ผนังยาว {w["length_m"]:.2f} m ห่าง {w["dist_m"]:.2f} m '
                     f'มุม {w["angle_deg"]:.2f}° - เริ่มหมุนได้')
        else:
            self._wall_after = lines
            self._update_wall_real_angle()

    @staticmethod
    def _wrap(deg, half):
        # wrap into (-half, half]
        return (deg + half) % (2 * half) - half

    def _update_wall_real_angle(self):
        # Needs: wall before, wall after, and the gyro's signed total.
        if not (self._wall_before and self._wall_after) or self._hand_gyro_deg is None:
            if self._wall_after and self._hand_gyro_deg is None:
                self.wall_status.config(text='วัดผนังหลังหมุนแล้ว - รอผล gyro (กดหยุดบันทึก)')
            return
        gyro = self._hand_gyro_deg
        ref = self._wall_before[0]
        # A wall at angle a in the robot frame appears at a - rot after the
        # robot turns rot (CCW +). A line is only defined mod 180, so pick
        # the after-wall closest to where the gyro says it should be, then
        # resolve the full turns from the gyro (valid while the gyro error
        # is well under 90 deg - it's ~1%).
        predicted = ref['angle_deg'] - gyro
        best = min(self._wall_after,
                   key=lambda w: abs(self._wrap(w['angle_deg'] - predicted, 90.0)))
        miss = self._wrap(best['angle_deg'] - predicted, 90.0)
        if abs(miss) > 20.0:
            self.wall_status.config(
                text=f'ไม่พบผนังเดิมหลังหมุน (ต่างจากที่คาด {miss:+.1f}°) - '
                     'ใช้มุมจริงแบบวัดด้วยมือ หรือวัดใหม่')
            return
        real = gyro - miss   # signed, CCW +
        self.gyro_real_var.set(f'{abs(real):.1f}')
        self.wall_status.config(
            text=f'มุมจริงจาก LiDAR: {real:+.1f}° (gyro {gyro:+.1f}°, ต่าง {-miss:+.2f}°)\n'
                 f'ผนังยาว {best["length_m"]:.2f} m  rms {best["rms_m"] * 1000:.0f} mm '
                 '- เติมในช่องมุมจริงแล้ว กดคำนวณได้เลย')

    def _on_click_gyro(self):
        if self._active_test == 'gyro_motor':
            self._cancel_active()
            return
        if self.gyro_mode_var.get() == 'motor' and self._active_test != 'gyro':
            self._start_gyro_motor()
            return
        if self._active_test == 'gyro':
            deg = self._stop_gyro()
            self._active_test = None
            self._on_gyro_mode_change()
            self._finish_gyro(deg)
            return
        if self._active_test is not None:
            messagebox.showerror('กำลังทดสอบอยู่', 'รอให้การทดสอบอื่นเสร็จก่อน')
            return
        if not messagebox.askyesno(
                'ยืนยัน', 'กด E-stop / ปิดมอเตอร์แล้ว และรถนิ่งอย่างน้อย 10 วินาทีใช่หรือไม่?'):
            return
        self._reset_result(self.gyro_result)
        self.btn_gyro_calc.config(state='disabled')
        self._hand_gyro_deg = None
        self._wall_after = None  # a new run needs a fresh "after" wall
        self._active_test = 'gyro'
        self._start_gyro_proc()
        self.btn_gyro.config(text='หยุดบันทึก (Stop)')
        self.gyro_status.config(text='กำลังบันทึก... หมุนรถด้วยมือแล้วกดหยุด')

    def _start_gyro_motor(self):
        if self._active_test is not None or (self._proc is not None and self._proc.poll() is None):
            messagebox.showerror('กำลังทดสอบอยู่', 'รอให้การทดสอบอื่นเสร็จก่อน')
            return
        try:
            turns = abs(float(self.gyro_turns_var.get()))
            speed = abs(float(self.gyro_speed_var.get()))
        except ValueError:
            messagebox.showerror('ค่าไม่ถูกต้อง', 'กรอกจำนวนรอบ/ความเร็วเป็นตัวเลข')
            return
        if turns <= 0 or not 0.1 <= speed <= 0.6:
            messagebox.showerror('ค่าไม่ถูกต้อง', 'จำนวนรอบต้อง > 0 และความเร็ว 0.1-0.6 rad/s')
            return
        ccw = self.gyro_dir_var.get() == 'ccw'
        if not messagebox.askyesno(
                'ยืนยันการทดสอบ',
                f'ให้หุ่นยนต์หมุนอยู่กับที่ {turns:g} รอบ ({"CCW" if ccw else "CW"}) '
                f'ด้วยความเร็ว {speed:.2f} rad/s ใช่หรือไม่?\n'
                'ปล่อย E-stop แล้ว และรถนิ่งอย่างน้อย 10 วินาที\n\n' + SAFETY_BANNER):
            return
        self._reset_result(self.gyro_result)
        self.btn_gyro_calc.config(state='disabled')
        self._hand_gyro_deg = None
        self._wall_after = None  # a new run needs a fresh "after" wall
        self.gyro_real_var.set(f'{360.0 * turns:g}')
        self._active_test = 'gyro_motor'
        self._start_gyro_proc()
        # calib_rotate_node.py: negative turns = CW
        self._proc = subprocess.Popen(
            ['ros2', 'run', 'amr_bringup', 'calib_rotate_node.py',
             f'{turns if ccw else -turns:.4f}', f'{speed:.3f}'],
            preexec_fn=os.setsid, env=os.environ.copy(),
            stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL)
        self.btn_gyro.config(text='ยกเลิก (Cancel)')
        self.gyro_status.config(text='กำลังหมุนด้วยมอเตอร์...')
        self._poll_proc()

    def _on_calc_gyro(self):
        try:
            real = abs(float(self.gyro_real_var.get()))
            old = float(self.gyro_scale_var.get())
        except ValueError:
            messagebox.showerror('ค่าไม่ถูกต้อง', 'กรอกมุมจริง/scale เป็นตัวเลข')
            return
        deg = self._hand_gyro_deg
        if deg is None or abs(deg) < 1.0 or real < 1.0 or old <= 0:
            messagebox.showerror('ค่าไม่ถูกต้อง', 'ไม่มีการหมุนที่บันทึกได้ หรือค่ามุม/scale ไม่ถูกต้อง')
            return
        ratio = real / abs(deg)
        new = old * ratio
        lines = [
            f'มุมจริง: {real:.1f} deg   มุมจาก gyro: {abs(deg):.1f} deg '
            f'({"CCW" if deg > 0 else "CW"})',
            f'error ของ gyro: {(abs(deg) / real - 1) * 100:+.2f}%',
            '',
            f'GYRO_SCALE_Z: {old:.4f} -> {new:.4f}',
            'แก้ใน stm/src/motor_control_with_pid_optimized.cpp:',
            f'const float GYRO_SCALE_Z = {new:.4f}f;',
        ]
        if real < 720:
            lines += ['', 'คำเตือน: หมุนน้อยกว่า 2 รอบ ความแม่นต่ำ ควรหมุน >= 5 รอบ']
        if abs(ratio - 1) > 0.05:
            lines += ['', 'คำเตือน: ต่างเกิน 5% (เกินสเปก MPU ±3%) - ตรวจ bias/การนับรอบ/การติดตั้ง IMU ก่อนใช้ค่า']
        lines += ['', 'ทดสอบทั้ง CW และ CCW แล้วใช้ค่าเฉลี่ย  re-flash แล้ววัดซ้ำเพื่อยืนยัน']
        self._set_result(self.gyro_result, '\n'.join(lines))

    # ------------------------------------------------------------------
    # Accel / wheel-slip step test (accel_slip_test_node.py)
    # ------------------------------------------------------------------
    def _build_accel_section(self):
        self._accel_lines = queue.Queue()
        frame = tk.LabelFrame(
            self._content,
            text='5) ทดสอบความเร่ง / ล้อลื่น (เดินตรง และ หมุนอยู่กับที่)',
            padx=10, pady=10)
        frame.pack(fill='x', padx=10, pady=6)

        mode_row = tk.Frame(frame)
        mode_row.pack(fill='x', pady=2)
        tk.Label(mode_row, text='โหมด:').pack(side='left')
        self.accel_mode_var = tk.StringVar(value='linear')
        for value, label in (('linear', 'เดินตรง (accel)'),
                             ('rotate', 'หมุนอยู่กับที่ (angular accel)')):
            tk.Radiobutton(
                mode_row, text=label, value=value, variable=self.accel_mode_var,
                command=self._on_accel_mode_change).pack(side='left', padx=4)

        self.accel_help = tk.Label(frame, anchor='w', justify='left', wraplength=560)
        self.accel_help.pack(fill='x')

        row = tk.Frame(frame)
        row.pack(fill='x', pady=2)
        self.accel_list_label = tk.Label(row)
        self.accel_list_label.pack(side='left')
        self.accel_list_var = tk.StringVar(value=ACCEL_LIST_DEFAULT)
        tk.Entry(row, textvariable=self.accel_list_var, width=22).pack(side='left', padx=4)

        row2 = tk.Frame(frame)
        row2.pack(fill='x', pady=2)
        self.accel_vmax_label = tk.Label(row2)
        self.accel_vmax_label.pack(side='left')
        self.accel_vmax_var = tk.StringVar(value=str(ACCEL_VMAX_DEFAULT))
        tk.Entry(row2, textvariable=self.accel_vmax_var, width=6).pack(side='left', padx=4)
        tk.Label(row2, text='รอบต่อค่า:').pack(side='left', padx=(8, 0))
        self.accel_reps_var = tk.StringVar(value=str(ACCEL_REPS_DEFAULT))
        tk.Entry(row2, textvariable=self.accel_reps_var, width=4).pack(side='left', padx=4)
        self.accel_stop_var = tk.BooleanVar(value=True)
        tk.Checkbutton(row2, text='หยุดเมื่อเจอลื่น',
                       variable=self.accel_stop_var).pack(side='left', padx=8)

        self.btn_accel = tk.Button(
            frame, text='เริ่มทดสอบความเร่ง (Start)', bg='#2980b9', fg='white',
            command=self._on_click_accel)
        self.btn_accel.pack(fill='x', pady=6)

        self.accel_status = tk.Label(frame, text='รอเริ่มการทดสอบ', anchor='w')
        self.accel_status.pack(fill='x')

        self.accel_result = scrolledtext.ScrolledText(
            frame, height=10, wrap='word', font='TkFixedFont')
        self.accel_result.pack(fill='both', expand=True)
        self.accel_result.config(state='disabled')
        self._on_accel_mode_change()

    def _on_accel_mode_change(self):
        rot = self.accel_mode_var.get() == 'rotate'
        if rot:
            self.accel_help.config(text=(
                'หุ่นหมุนอยู่กับที่ สลับ CCW/CW ไล่ angular accel ทีละค่า '
                'เทียบ yaw rate จากล้อ กับ gyro z (แม่นกว่าโหมดเดินตรง). '
                'ต้องเคลียร์พื้นที่เป็นวงกลมรัศมี >= 1.1 m รอบหุ่น '
                '(node หยุดเองถ้า LiDAR เห็นของในรัศมี 0.8 m), ใส่ payload เต็ม, '
                'ห้ามสั่ง Navigation/teleop ระหว่างทดสอบ'))
            self.accel_list_label.config(text='angular accel (rad/s², คั่นด้วย ,):')
            self.accel_vmax_label.config(text='ความเร็วหมุนสูงสุด (rad/s):')
            self.accel_list_var.set(ROT_LIST_DEFAULT)
            self.accel_vmax_var.set(str(ROT_WMAX_DEFAULT))
        else:
            self.accel_help.config(text=(
                'หุ่นวิ่งเดินหน้า/ถอยหลังสลับกัน (~0.6 m) ไล่ความเร่งทีละค่า '
                'เทียบความเร็วล้อ (encoder) กับความเร็วตัวรถ (IMU). '
                'ต้องมีทางโล่งหน้า-หลังอย่างน้อย 1.5 m, ใส่ payload เต็ม, '
                'ห้ามสั่ง Navigation/teleop ระหว่างทดสอบ'))
            self.accel_list_label.config(text='ความเร่ง (m/s², คั่นด้วย ,):')
            self.accel_vmax_label.config(text='ความเร็วสูงสุด (m/s):')
            self.accel_list_var.set(ACCEL_LIST_DEFAULT)
            self.accel_vmax_var.set(str(ACCEL_VMAX_DEFAULT))

    def _on_click_accel(self):
        if self._proc is not None and self._proc.poll() is None:
            self._cancel_active()
            return
        if self._active_test == 'gyro':
            messagebox.showerror('กำลังบันทึก gyro', 'หยุดบันทึก gyro (หัวข้อ 4) ก่อน')
            return
        try:
            accels = sorted(float(a) for a in self.accel_list_var.get().split(',')
                            if a.strip())
            vmax = float(self.accel_vmax_var.get())
            reps = int(self.accel_reps_var.get())
        except ValueError:
            messagebox.showerror('ค่าไม่ถูกต้อง', 'กรอกความเร่ง/ความเร็ว/จำนวนรอบเป็นตัวเลข')
            return
        rot = self.accel_mode_var.get() == 'rotate'
        vlim = 0.6 if rot else 0.3
        if not accels or min(accels) <= 0 or not 0 < vmax <= vlim or reps < 1:
            messagebox.showerror(
                'ค่าไม่ถูกต้อง',
                f'ความเร่งต้อง > 0, ความเร็วสูงสุด 0 < v <= {vlim:g} '
                f'{"rad/s" if rot else "m/s"}, รอบ >= 1')
            return

        if rot:
            what = (f'ทดสอบหมุนอยู่กับที่ angular accel '
                    f'{", ".join(f"{a:g}" for a in accels)} rad/s² '
                    f'ที่ {vmax:.2f} rad/s ค่าละ {reps} รอบ (CCW/CW สลับกัน)')
            guard = 'node จะหยุดเองถ้า LiDAR เห็นสิ่งกีดขวางในรัศมี 0.8 m รอบหุ่น'
        else:
            what = (f'ทดสอบความเร่ง {", ".join(f"{a:g}" for a in accels)} m/s² '
                    f'ที่ความเร็ว {vmax:.2f} m/s ค่าละ {reps} รอบ '
                    '(เดินหน้า/ถอยหลังสลับกัน)')
            guard = 'node จะหยุดเองถ้า LiDAR เห็นสิ่งกีดขวางในทางวิ่ง < 0.6 m'
        if not messagebox.askyesno(
                'ยืนยันการทดสอบ',
                what + ' ใช่หรือไม่?\n\n'
                'การทดสอบนี้สั่ง /cmd_vel ตรง ไม่ผ่าน collision_monitor - '
                + guard + ' แต่ต้องมีคนคุม E-stop ตลอด\n\n' + SAFETY_BANNER):
            return

        self._reset_result(self.accel_result)
        self._accel_results = []
        self._accel_summary = None
        self.btn_accel.config(text='ยกเลิก (Cancel)')
        self.accel_status.config(text='กำลังเริ่ม...')
        cmd = ['ros2', 'run', 'amr_bringup', 'accel_slip_test_node.py',
               '--accels', ','.join(f'{a:g}' for a in accels),
               '--vmax', f'{vmax:.3f}',
               '--decel', f'{ROT_DECEL_DEFAULT if rot else ACCEL_DECEL_DEFAULT:g}',
               '--reps', str(reps), '--mode', 'rotate' if rot else 'linear']
        if self.accel_stop_var.get():
            cmd.append('--stop-on-slip')
        self._active_test = 'accel'
        self._proc = subprocess.Popen(
            cmd, preexec_fn=os.setsid, env=os.environ.copy(),
            stdin=subprocess.DEVNULL, stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL, text=True)
        proc = self._proc

        # Read progress lines on a thread; the Tk side drains the queue in
        # _poll_accel_lines (Tk widgets must only be touched from mainloop).
        def reader():
            for line in proc.stdout:
                self._accel_lines.put(line.rstrip('\n'))
        self._accel_reader = threading.Thread(target=reader, daemon=True)
        self._accel_reader.start()
        self._poll_accel_lines()
        self._poll_proc()

    def _poll_accel_lines(self):
        while True:
            try:
                line = self._accel_lines.get_nowait()
            except queue.Empty:
                break
            tag, _, payload = line.partition(' ')
            if tag == 'PROGRESS':
                self.accel_status.config(text=payload)
            elif tag == 'RESULT':
                try:
                    self._accel_results.append(json.loads(payload))
                except ValueError:
                    continue
                self._render_accel()
            elif tag == 'SUMMARY':
                try:
                    self._accel_summary = json.loads(payload)
                except ValueError:
                    continue
                self._render_accel()
        # keep draining until the reader hit EOF - SUMMARY is the last line
        # and may arrive after _poll_proc has already seen the exit
        if self._accel_reader.is_alive() or not self._accel_lines.empty():
            self.after(200, self._poll_accel_lines)

    def _render_accel(self):
        def pct(v):
            return '   -  ' if v is None else f'{v * 100:5.1f}%'
        words = {'ok': 'ไม่ลื่น', 'unclear': 'ไม่แน่ชัด', 'slip': 'ลื่น', None: '-'}
        rot = any(r.get('mode') == 'rotate' for r in self._accel_results)
        unit = 'rad/s²' if rot else 'm/s²'
        head = 'alpha [rad/s²]' if rot else 'accel [m/s²]  '
        lines = [f'{head} slip เร่ง   slip เบรก   ผล (ค่ากลางของทุกรอบ)']
        for r in self._accel_results:
            note = words[r.get('verdict_accel')]
            if r.get('caster_lag_ms') is not None:
                note += f', gyro ตามล้อช้า {r["caster_lag_ms"]:.0f} ms'
            if r.get('verdict_decel') in ('unclear', 'slip'):
                note += f', เบรก{words[r["verdict_decel"]]}'
            if r.get('stop_overshoot', 0.0) > 0.02:
                note += f', ล้อหมุนกลับตอนหยุด {r["stop_overshoot"]:.2f} m/s'
            if r['events'] < r['expected_events']:
                note += f' (จับช่วงเร่ง/เบรกได้ {r["events"]}/{r["expected_events"]})'
            lines.append(f'   {r["accel"]:5.2f}      {pct(r["slip_accel"])}     '
                         f'{pct(r["slip_decel"])}     {note}')
        s = self._accel_summary
        if s is not None:
            lines.append('')
            target = 'max_accel[2] (angular)' if rot else 'max_accel'
            if s['a_limit'] is not None:
                lines.append(f'เริ่มลื่นที่ a_limit = {s["a_limit"]:g} {unit}')
                lines.append(f'แนะนำ velocity_smoother {target} ≈ '
                             f'{s["suggest_accel"]:g} {unit} (55% ของ a_limit)')
            elif s['max_ok'] is not None:
                cap = '~2.1 rad/s²' if rot else '0.5 m/s²'
                lines.append(f'ไม่ลื่นจนถึง {s["max_ok"]:g} {unit} '
                             f'(a_limit สูงกว่านี้ - เกิน {cap} ต้องแก้ firmware ramp)')
            if s.get('unclear'):
                band = ('5-15%' if rot else '10-25%')
                why = ('ใกล้เกณฑ์ - อาจเริ่มลื่นหรือ caster ลาก' if rot
                       else 'อยู่ในช่วง noise ของการวัด (IMU/ตัวรถโยก)')
                lines.append('ไม่แน่ชัดที่ ' + ', '.join(f'{a:g}' for a in s['unclear'])
                             + f' {unit} - slip {band} {why}: ทดสอบซ้ำด้วยรอบมากขึ้น '
                             'และดูล้อด้วยตา - ไม่มีค่าแนะนำจากผลนี้')
            if rot:
                lines.append('gyro ตามล้อช้า = เวลาที่ตัวรถเริ่มหมุนตามหลังล้อ '
                             '(รวม lag ของ odom filter) - ค่าที่เพิ่มขึ้นมาก/ไม่เท่ากัน '
                             'ระหว่างรอบ = caster กลับตัวไม่ทัน')
            lines.append('')
            lines.append('ค่านี้เป็นคำแนะนำเท่านั้น - แก้ nav2_params.yaml เอง, '
                         'commit, วัดระยะหยุดจริง และสร้าง tag ใหม่ก่อนใช้งานในไลน์')
        self._set_result(self.accel_result, '\n'.join(lines))

    # ------------------------------------------------------------------
    # Shared subprocess plumbing
    # ------------------------------------------------------------------
    def _reset_result(self, widget):
        widget.config(state='normal')
        widget.delete('1.0', 'end')
        widget.config(state='disabled')

    def _set_result(self, widget, text):
        widget.config(state='normal')
        widget.delete('1.0', 'end')
        widget.insert('1.0', text)
        widget.config(state='disabled')

    def _cancel_active(self):
        if self._wall_proc is not None and self._wall_proc.poll() is None:
            try:
                os.killpg(os.getpgid(self._wall_proc.pid), signal.SIGINT)
            except Exception:
                pass
        if self._proc is not None and self._proc.poll() is None:
            try:
                os.killpg(os.getpgid(self._proc.pid), signal.SIGINT)
            except Exception:
                pass
        self._stop_gyro()

    def _stop_gyro(self):
        """SIGINT gyro_integrate_node.py and parse its final GYRO_DEG line."""
        proc = self._gyro_proc
        self._gyro_proc = None
        if proc is None:
            return None
        try:
            if proc.poll() is None:
                os.killpg(os.getpgid(proc.pid), signal.SIGINT)
            out, _ = proc.communicate(timeout=5)
        except Exception:
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
            except Exception:
                pass
            return None
        for line in (out or '').splitlines():
            if line.startswith('GYRO_DEG='):
                try:
                    return float(line.split()[0].split('=')[1])
                except (IndexError, ValueError):
                    return None
        return None

    def _poll_proc(self):
        proc = self._proc
        if proc is None:
            return
        ret = proc.poll()
        if ret is None:
            self._poll_job = self.after(200, self._poll_proc)
            return

        self._poll_job = None
        self._proc = None
        test = self._active_test
        self._active_test = None

        if test == 'rotate':
            self.btn_rotate.config(text='เริ่มหมุน (Start)')
            self._last_gyro_deg = self._stop_gyro()
            if ret == 0:
                self.rotate_status.config(text='เสร็จสิ้น - กรอกมุมที่วัดได้จริงแล้วกดคำนวณ')
                self.rotate_measured_entry.config(state='normal')
                self.btn_rotate_calc.config(state='normal')
            elif ret == 3:
                self.rotate_status.config(text='หยุดกลางทาง: timeout ระหว่างหมุน')
                messagebox.showwarning('หยุดกลางทาง', 'timeout ระหว่างหมุน (หุ่นติดขัด/สั่งความเร็วสูงไป)')
            elif ret == 4:
                self.rotate_status.config(text='ไม่พบข้อมูล /wheel/odom')
                messagebox.showwarning('ไม่พบข้อมูล', 'ไม่ได้รับข้อมูลจาก /wheel/odom - ตรวจสอบ bringup')
            else:
                self.rotate_status.config(text=f'ยกเลิก/หยุด (exit code {ret})')
        elif test == 'gyro_motor':
            self._on_gyro_mode_change()
            # let the robot settle so the gyro captures the full stop
            self.after(1000, lambda: self._finish_gyro_motor(ret))
        elif test == 'straight':
            self.btn_straight.config(text='เริ่มวิ่งตรง (Start)')
            if ret == 0:
                self.straight_status.config(text='เสร็จสิ้น - กรอกระยะเยื้องที่วัดได้จริงแล้วกดคำนวณ')
                self.straight_offset_entry.config(state='normal')
                self.straight_real_distance_entry.config(state='normal')
                self.btn_straight_calc.config(state='normal')
            elif ret == 1:
                self.straight_status.config(text='หยุดกลางทาง: timeout ระหว่างวิ่งตรง')
                messagebox.showwarning('หยุดกลางทาง', 'timeout ระหว่างวิ่งตรง (หุ่นติดขัด/encoder นิ่ง)')
            elif ret == 2:
                self.straight_status.config(text='ไม่พบข้อมูล /wheel/raw_distance')
                messagebox.showwarning('ไม่พบข้อมูล', 'ไม่ได้รับข้อมูลจาก /wheel/raw_distance - ตรวจสอบ bringup')
            else:
                self.straight_status.config(text=f'ยกเลิก/หยุด (exit code {ret})')

        elif test == 'accel':
            self.btn_accel.config(text='เริ่มทดสอบความเร่ง (Start)')
            if ret == 0:
                self.accel_status.config(text='เสร็จสิ้น')
            elif ret == 2:
                self.accel_status.config(text='ไม่พบข้อมูล /wheel/odom หรือ /imu/data')
                messagebox.showwarning('ไม่พบข้อมูล', 'ไม่ได้รับข้อมูล /wheel/odom หรือ /imu/data - ตรวจสอบ bringup')
            elif ret == 5:
                self.accel_status.config(text='หยุด: พบสิ่งกีดขวางในทางวิ่ง')
                messagebox.showwarning('พบสิ่งกีดขวาง', 'LiDAR พบสิ่งกีดขวางในทางวิ่ง < 0.6 m - หยุดทดสอบ')
            else:
                self.accel_status.config(text=f'ยกเลิก/หยุด (exit code {ret})')

    def _finish_gyro_motor(self, ret):
        deg = self._stop_gyro()
        if ret == 0:
            self._finish_gyro(deg)
            if deg is not None:
                self.gyro_status.config(
                    text=f'หมุนเสร็จ: gyro วัดได้ {deg:+.1f} deg - วัดมุมจริงจากเครื่องหมาย กรอกแล้วกดคำนวณ')
        elif ret == 3:
            self.gyro_status.config(text='หยุดกลางทาง: timeout ระหว่างหมุน')
        elif ret == 4:
            self.gyro_status.config(text='ไม่พบข้อมูล /wheel/odom - ตรวจสอบ bringup')
        else:
            self.gyro_status.config(text=f'ยกเลิก/หยุด (exit code {ret})')

    def _on_close(self):
        self._cancel_active()
        self.unbind_all('<MouseWheel>')
        self.unbind_all('<Button-4>')
        self.unbind_all('<Button-5>')
        self.destroy()
        if self._on_close_extra is not None:
            self._on_close_extra()


def main():
    root = tk.Tk()
    root.withdraw()
    CalibPanel(root, on_close=root.destroy)
    root.mainloop()


if __name__ == '__main__':
    main()
