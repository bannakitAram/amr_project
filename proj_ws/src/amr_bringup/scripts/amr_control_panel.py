#!/usr/bin/env python3
"""Minimal desktop launcher for the AMR: pick Mapping or Navigation mode,
start/stop the corresponding bringup, and (in Mapping mode) save the map -
without needing to type any ros2 launch commands by hand.
"""
import glob
import os
import re
import signal
import subprocess
import threading
import time
import tkinter as tk
from tkinter import font as tkfont
from tkinter import messagebox, ttk

from ament_index_python.packages import get_package_share_directory

MAPS_DIR = os.path.join(
    get_package_share_directory('amr_navigation'), '..', '..', '..', '..',
    'src', 'amr_navigation', 'maps')
MAPS_DIR = os.path.normpath(MAPS_DIR)

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


class AmrControlPanel(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title('AMR Control')
        self.configure(bg=BG)
        self.geometry('620x480')
        self.minsize(560, 440)

        self._proc = None
        self._mode = None  # 'mapping' | 'navigation'
        self._status_poll_job = None

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
        header.pack(fill='x', padx=28, pady=(24, 8))
        tk.Label(header, text='AMR Control', font=self.f_title,
                 bg=BG, fg=TEXT_MAIN).pack(anchor='w')
        tk.Label(header, text='เลือกโหมดการทำงาน', font=self.f_small,
                 bg=BG, fg=TEXT_MUTED).pack(anchor='w')

        cards = tk.Frame(self, bg=BG)
        cards.pack(fill='x', padx=28, pady=8)
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
        status_card.pack(fill='x', padx=28, pady=16)
        inner = tk.Frame(status_card, bg=CARD_BG)
        inner.pack(fill='x', padx=18, pady=14)

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
                                      justify='left', wraplength=520)
        self.detail_label.pack(fill='x', pady=(6, 0))

        # Action row (Save map / Stop)
        actions = tk.Frame(self, bg=BG)
        actions.pack(fill='x', padx=28, pady=(0, 24))

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

    # ---------- mode launches ----------
    def _on_click_mapping(self):
        if self._proc is not None:
            return
        self._launch([
            'ros2', 'launch', 'amr_bringup', 'bringup.launch.py',
            'rviz:=false', 'foxglove:=true', 'slam:=true', 'navigation:=false',
        ], mode='mapping', label='โหมดสร้างแผนที่ (Mapping)')

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
        self.destroy()


def main():
    app = AmrControlPanel()
    app.mainloop()


if __name__ == '__main__':
    main()
