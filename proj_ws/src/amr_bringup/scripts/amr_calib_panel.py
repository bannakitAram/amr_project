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
        self._poll_job = None
        self._active_test = None  # 'rotate' or 'straight'
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

        self.btn_rotate.config(text='ยกเลิก (Cancel)')
        self.rotate_status.config(text='กำลังหมุน...')
        env = os.environ.copy()
        self._active_test = 'rotate'
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

        text = (
            f'มุมที่สั่ง (raw /wheel/odom): {commanded_deg:.1f} deg\n'
            f'มุมที่วัดได้จริง: {measured_deg:.1f} deg\n\n'
            f'wheel_sep ปัจจุบัน: {WHEEL_SEP_NOMINAL:.4f} m\n'
            f'wheel_sep ที่แก้ไขแล้ว: {corrected:.4f} m\n\n'
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
        if self._proc is not None and self._proc.poll() is None:
            try:
                os.killpg(os.getpgid(self._proc.pid), signal.SIGINT)
            except Exception:
                pass

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
