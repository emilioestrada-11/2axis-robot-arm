import tkinter as tk
from tkinter import ttk, messagebox
import serial
import serial.tools.list_ports
import threading
import time
import math
import re

# ---------------- SCARA ARM SIMULATION ----------------
ARM1 = 140
ARM2 = 140
BASE_X = -100
BASE_Y = 0

class SCARASim:
    def __init__(self, canvas):
        self.canvas = canvas
        self.joint_radius = 5
        self.arm_line_width = 3
        self.arm_lines = []
        self.joints = []
        self.grid_size = 50
        self.draw_grid_and_axes()

    def map_coord(self, x, y):
        scale = 450 / 600
        return 225 + x * scale, 200 - y * scale

    def draw_grid_and_axes(self):
        self.canvas.delete("grid")
        for i in range(-300, 301, self.grid_size):
            x1, y1 = self.map_coord(i, -300)
            x2, y2 = self.map_coord(i, 300)
            self.canvas.create_line(x1, y1, x2, y2, fill='lightgray', dash=(2,2), tags="grid")
            x1, y1 = self.map_coord(-300, i)
            x2, y2 = self.map_coord(300, i)
            self.canvas.create_line(x1, y1, x2, y2, fill='lightgray', dash=(2,2), tags="grid")

        x1, y1 = self.map_coord(-300, 0)
        x2, y2 = self.map_coord(300, 0)
        self.canvas.create_line(x1, y1, x2, y2, fill='red', width=2, tags="grid")
        x1, y1 = self.map_coord(0, -300)
        x2, y2 = self.map_coord(0, 300)
        self.canvas.create_line(x1, y1, x2, y2, fill='blue', width=2, tags="grid")

        for i in range(-300, 301, 50):
            if i != 0:
                x, y = self.map_coord(i, -15)
                self.canvas.create_text(x, y, text=str(i), tags="grid", font=("Arial", 7))
                x, y = self.map_coord(-38, i)
                self.canvas.create_text(x, y, text=str(i), tags="grid", font=("Arial", 7))

        x, y = self.map_coord(0, 0)
        self.canvas.create_oval(x-3, y-3, x+3, y+3, fill='black', tags="grid")
        self.canvas.create_text(x+15, y-15, text="(0,0)", tags="grid", font=("Arial", 9, "bold"))

        bx, by = self.map_coord(BASE_X, BASE_Y)
        self.canvas.create_oval(bx-4, by-4, bx+4, by+4, fill='purple', tags="grid")
        self.canvas.create_text(bx+15, by-15, text=f"Base\n({BASE_X},{BASE_Y})", tags="grid", font=("Arial", 7))

    def forward_kinematics(self, theta1, theta2):
        x1 = BASE_X + ARM1 * math.cos(theta1)
        y1 = BASE_Y + ARM1 * math.sin(theta1)
        x2 = x1 + ARM2 * math.cos(theta1 + theta2)
        y2 = y1 + ARM2 * math.sin(theta1 + theta2)
        return (x1, y1), (x2, y2)

    def draw_arm(self, theta1, theta2):
        self.canvas.delete("arm")
        elbow, end = self.forward_kinematics(theta1, theta2)
        bx, by = self.map_coord(BASE_X, BASE_Y)
        ex, ey = self.map_coord(*elbow)
        fx, fy = self.map_coord(*end)

        self.canvas.create_line(bx, by, ex, ey, width=self.arm_line_width, fill='blue', tags="arm")
        self.canvas.create_line(ex, ey, fx, fy, width=self.arm_line_width, fill='green', tags="arm")
        self.canvas.create_oval(bx-5, by-5, bx+5, by+5, fill='black', tags="arm")
        self.canvas.create_oval(ex-5, ey-5, ex+5, ey+5, fill='red', tags="arm")
        self.canvas.create_oval(fx-5, fy-5, fx+5, fy+5, fill='orange', tags="arm")
        self.canvas.create_text(fx+20, fy-20, text=f"({end[0]:.1f}, {end[1]:.1f})",
                                tags="arm", fill="darkgreen", font=("Arial", 8))


# ---------------- G-CODE EXECUTOR ----------------
class GCodeExecutor:
    TOOL_CHANGE_POS = (170, 0)
    INCH_TO_MM = 25.4

    def __init__(self, scara_sim):
        self.scara = scara_sim
        self.reset()

    def reset(self):
        self.x = 0.0
        self.y = 0.0
        self.feed_rate = 30.0
        self.absolute = True
        self.units_mm = True
        self.tool = 1
        self.program_end = False
        self.segments = []
        self.events = []

    def _to_mm(self, v):
        return v * self.INCH_TO_MM if not self.units_mm else v

    @staticmethod
    def _interpolate(x0, y0, x1, y1, step=2.0):
        dist = math.sqrt((x1-x0)**2 + (y1-y0)**2)
        if dist == 0:
            return [(x0, y0)]
        n = max(2, int(dist / step))
        return [(x0 + (x1-x0)*t/n, y0 + (y1-y0)*t/n) for t in range(n+1)]

    def _check_reachable(self, x, y):
        rel_x = x - BASE_X
        rel_y = y - BASE_Y
        d = (rel_x*rel_x + rel_y*rel_y - ARM1*ARM1 - ARM2*ARM2) / (2*ARM1*ARM2)
        return abs(d) <= 1

    def _check_points(self, pts):
        return [self._check_reachable(x, y) for x, y in pts]

    def parse_line(self, raw):
        line = re.sub(r'\(.*?\)', '', raw)
        line = re.sub(r';.*', '', line).strip().upper()
        if not line:
            return

        all_tokens = [(m.group(1), float(m.group(2)))
                      for m in re.finditer(r'([A-Z])\s*(-?\d+\.?\d*)', line)]

        params = {}
        for key, val in all_tokens:
            if key not in ('G', 'M'):
                params[key] = val

        g_codes = [int(val) for key, val in all_tokens if key == 'G']
        m_codes = [int(val) for key, val in all_tokens if key == 'M']

        motion_g = None

        for g in g_codes:
            if g == 90:
                self.absolute = True
            elif g == 91:
                self.absolute = False
            elif g == 20:
                self.units_mm = False
            elif g == 21:
                self.units_mm = True
            elif g in (0, 1):
                motion_g = g

        if motion_g is not None:
            if 'F' in params:
                self.feed_rate = self._to_mm(params['F'])

            tx_raw = params.get('X', self.x if self.absolute else 0)
            ty_raw = params.get('Y', self.y if self.absolute else 0)
            tx_mm = self._to_mm(tx_raw)
            ty_mm = self._to_mm(ty_raw)

            if self.absolute:
                tx, ty = tx_mm, ty_mm
            else:
                tx = self.x + tx_mm
                ty = self.y + ty_mm

            motion = 'G0' if motion_g == 0 else 'G1'
            pts = self._interpolate(self.x, self.y, tx, ty)
            reach = self._check_points(pts)

            self.segments.append({
                'type': motion,
                'points': pts,
                'feed': self.feed_rate,
                'reachable': reach
            })
            self.x, self.y = tx, ty

        for m in m_codes:
            if m == 2:
                self.program_end = True
                return
            if m == 6:
                tool_num = int(params.get('T', self.tool + 1))
                self.tool = tool_num
                tx, ty = self.TOOL_CHANGE_POS
                pts = self._interpolate(self.x, self.y, tx, ty)
                reach = self._check_points(pts)
                self.segments.append({
                    'type': 'toolchange',
                    'points': pts,
                    'feed': self.feed_rate,
                    'reachable': reach
                })
                self.x, self.y = tx, ty
                return

    def run(self, gcode_text):
        self.reset()
        for raw in gcode_text.splitlines():
            if self.program_end:
                break
            self.parse_line(raw)
        return self.segments, self.events


# ---------------- GUI ----------------
class SCARAGUI:
    def __init__(self, master):
        self.master = master
        self.master.title("SCARA ESP32 GUI with G-Code")
        self.master.geometry("1100x800")
        self.master.resizable(True, True)

        self.serial_port = None
        self.running = True
        self.current_position = (0, 0)
        self.gcode_executor = None
        self.gcode_sending = False  # track if we are currently sending g-code

        main_paned = ttk.PanedWindow(master, orient=tk.HORIZONTAL)
        main_paned.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)

        left_frame = ttk.Frame(main_paned)
        main_paned.add(left_frame, weight=1)

        right_frame = ttk.Frame(main_paned)
        main_paned.add(right_frame, weight=1)

        # ---- LEFT COLUMN ----
        serial_frame = ttk.LabelFrame(left_frame, text="ESP32 Connection")
        serial_frame.pack(fill="x", padx=5, pady=5)

        self.port_var = tk.StringVar()
        self.port_combo = ttk.Combobox(serial_frame, textvariable=self.port_var, width=12)
        self.port_combo.pack(side="left", padx=5, pady=5)

        self.connect_btn = ttk.Button(serial_frame, text="Connect", command=self.connect)
        self.connect_btn.pack(side="left", padx=5)

        self.disconnect_btn = ttk.Button(serial_frame, text="Disconnect",
                                         command=self.disconnect, state="disabled")
        self.disconnect_btn.pack(side="left", padx=5)

        self.refresh_btn = ttk.Button(serial_frame, text="Refresh", command=self.refresh_ports)
        self.refresh_btn.pack(side="left", padx=5)

        self.status_label = ttk.Label(serial_frame, text="Not connected", width=20)
        self.status_label.pack(side="left", padx=10)

        btn_frame = ttk.LabelFrame(left_frame, text="Shapes / Commands")
        btn_frame.pack(fill="x", padx=5, pady=5)

        btn_row1 = ttk.Frame(btn_frame)
        btn_row1.pack(fill="x", pady=2)
        self.line_btn = ttk.Button(btn_row1, text="Line", command=lambda: self.send_cmd("LINE"), width=10)
        self.line_btn.pack(side="left", padx=5)
        self.square_btn = ttk.Button(btn_row1, text="Square", command=lambda: self.send_cmd("SQUARE"), width=10)
        self.square_btn.pack(side="left", padx=5)
        self.triangle_btn = ttk.Button(btn_row1, text="Triangle", command=lambda: self.send_cmd("TRIANGLE"), width=10)
        self.triangle_btn.pack(side="left", padx=5)

        btn_row2 = ttk.Frame(btn_frame)
        btn_row2.pack(fill="x", pady=2)
        self.circle_btn = ttk.Button(btn_row2, text="Circle", command=lambda: self.send_cmd("CIRCLE"), width=10)
        self.circle_btn.pack(side="left", padx=5)
        self.home_btn = ttk.Button(btn_row2, text="Home (0,0)", command=lambda: self.send_cmd("HOME"), width=10)
        self.home_btn.pack(side="left", padx=5)
        self.estop_btn = ttk.Button(btn_row2, text="E-Stop", command=lambda: self.send_cmd("ESTOP"), width=10)
        self.estop_btn.pack(side="left", padx=5)

        pos_frame = ttk.LabelFrame(left_frame, text="Current Position")
        pos_frame.pack(fill="x", padx=5, pady=5)
        self.pos_label = ttk.Label(pos_frame, text="X: 0.0 mm, Y: 0.0 mm", font=("Arial", 12))
        self.pos_label.pack(pady=5)

        canvas_frame = ttk.LabelFrame(left_frame, text="SCARA Arm Simulation")
        canvas_frame.pack(fill="both", expand=True, padx=5, pady=5)
        self.canvas = tk.Canvas(canvas_frame, width=450, height=500, bg="white")
        self.canvas.pack(padx=5, pady=5)
        self.arm_sim = SCARASim(self.canvas)

        manual_frame = ttk.LabelFrame(left_frame, text="Manual Position")
        manual_frame.pack(fill="x", padx=5, pady=5)
        ttk.Label(manual_frame, text="X:").pack(side="left", padx=2)
        self.x_entry = ttk.Entry(manual_frame, width=8)
        self.x_entry.pack(side="left", padx=2)
        self.x_entry.insert(0, "0")
        ttk.Label(manual_frame, text="Y:").pack(side="left", padx=2)
        self.y_entry = ttk.Entry(manual_frame, width=8)
        self.y_entry.pack(side="left", padx=2)
        self.y_entry.insert(0, "0")
        self.move_btn = ttk.Button(manual_frame, text="Move To",
                                   command=self.send_manual_position, width=10)
        self.move_btn.pack(side="left", padx=5)

        self.set_buttons_state("disabled")
        self.refresh_ports()
        self.master.after(1000, self.auto_connect_com3)

        # ---- RIGHT COLUMN ----
        gcode_frame = ttk.LabelFrame(right_frame, text="G-Code Input")
        gcode_frame.pack(fill="both", expand=True, padx=5, pady=5)

        text_frame = ttk.Frame(gcode_frame)
        text_frame.pack(fill="both", expand=True, padx=5, pady=5)

        self.gcode_text = tk.Text(text_frame, height=20, width=40, font=("Courier", 10))
        self.gcode_text.pack(side="left", fill="both", expand=True)

        scrollbar = ttk.Scrollbar(text_frame, orient="vertical", command=self.gcode_text.yview)
        scrollbar.pack(side="right", fill="y")
        self.gcode_text.configure(yscrollcommand=scrollbar.set)

        example_gcode = """G21 ; mm mode
G90 ; absolute positioning
G0 X0 Y0
G1 X50 Y0 F100
G1 X50 Y-50
G1 X0 Y-50
G1 X0 Y0
M2 ; end program"""
        self.gcode_text.insert("1.0", example_gcode)

        gcode_btn_frame = ttk.Frame(gcode_frame)
        gcode_btn_frame.pack(fill="x", padx=5, pady=5)

        self.run_gcode_btn = ttk.Button(gcode_btn_frame, text="Run G-Code",
                                        command=self.run_gcode, width=15)
        self.run_gcode_btn.pack(side="left", padx=5)
        self.clear_gcode_btn = ttk.Button(gcode_btn_frame, text="Clear",
                                          command=self.clear_gcode, width=10)
        self.clear_gcode_btn.pack(side="left", padx=5)
        self.load_example_btn = ttk.Button(gcode_btn_frame, text="Load Example",
                                           command=self.load_example, width=12)
        self.load_example_btn.pack(side="left", padx=5)
        self.send_gcode_btn = ttk.Button(gcode_btn_frame, text="Send to Robot",
                                         command=self.send_gcode_to_robot, width=12)
        self.send_gcode_btn.pack(side="left", padx=5)
        self.stop_gcode_btn = ttk.Button(gcode_btn_frame, text="Stop Send",
                                         command=self.stop_gcode_send, width=10,
                                         state="disabled")
        self.stop_gcode_btn.pack(side="left", padx=5)

        # G-code send progress / log
        log_frame = ttk.LabelFrame(right_frame, text="G-Code Send Log")
        log_frame.pack(fill="both", expand=True, padx=5, pady=5)

        log_text_frame = ttk.Frame(log_frame)
        log_text_frame.pack(fill="both", expand=True, padx=5, pady=5)

        self.log_text = tk.Text(log_text_frame, height=10, width=40,
                                font=("Courier", 9), state="disabled", bg="#1e1e1e", fg="#00ff88")
        self.log_text.pack(side="left", fill="both", expand=True)

        log_scrollbar = ttk.Scrollbar(log_text_frame, orient="vertical", command=self.log_text.yview)
        log_scrollbar.pack(side="right", fill="y")
        self.log_text.configure(yscrollcommand=log_scrollbar.set)

        self.thread = threading.Thread(target=self.read_serial, daemon=True)
        self.thread.start()

    # ------------------------------------------------------------------ helpers

    def log(self, msg):
        """Append a message to the G-code send log (thread-safe)."""
        def _append():
            self.log_text.config(state="normal")
            self.log_text.insert(tk.END, msg + "\n")
            self.log_text.see(tk.END)
            self.log_text.config(state="disabled")
        self.master.after(0, _append)

    def set_buttons_state(self, state):
        self.line_btn.config(state=state)
        self.square_btn.config(state=state)
        self.triangle_btn.config(state=state)
        self.circle_btn.config(state=state)
        self.home_btn.config(state=state)
        self.estop_btn.config(state=state)
        self.move_btn.config(state=state)

    def refresh_ports(self):
        ports = [p.device for p in serial.tools.list_ports.comports()]
        self.port_combo['values'] = ports
        if ports and not self.port_var.get():
            self.port_combo.current(0)

    def auto_connect_com3(self):
        ports = [p.device for p in serial.tools.list_ports.comports()]
        if "COM3" in ports:
            self.port_var.set("COM3")
            self.connect()
        else:
            self.status_label.config(text="COM3 not found - select manually")

    def connect(self):
        port = self.port_var.get()
        if not port:
            messagebox.showwarning("No Port", "Please select a COM port")
            return
        try:
            self.serial_port = serial.Serial(port, 115200, timeout=0.1)
            self.status_label.config(text=f"Connected: {port}")
            self.connect_btn.config(state="disabled")
            self.disconnect_btn.config(state="normal")
            self.set_buttons_state("normal")
            time.sleep(0.5)
            self.send_cmd("STATUS")
        except Exception as e:
            messagebox.showerror("Connection Error", str(e))

    def disconnect(self):
        self.gcode_sending = False
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
        self.serial_port = None
        self.status_label.config(text="Disconnected")
        self.connect_btn.config(state="normal")
        self.disconnect_btn.config(state="disabled")
        self.set_buttons_state("disabled")

    def send_cmd(self, cmd):
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.write((cmd + "\n").encode())
            print(f"Sent: {cmd}")
        else:
            messagebox.showwarning("Not Connected", "Please connect to ESP32 first")

    def send_manual_position(self):
        try:
            x = float(self.x_entry.get())
            y = float(self.y_entry.get())
            self.send_cmd(f"G0 X{x} Y{y}")
        except ValueError:
            messagebox.showerror("Invalid Input", "Please enter valid numbers for X and Y")

    # ------------------------------------------------------------------ serial read thread

    def read_serial(self):
        while self.running:
            if self.serial_port and self.serial_port.is_open:
                try:
                    if hasattr(self.serial_port, 'is_open') and self.serial_port.is_open:
                        if self.serial_port.in_waiting:
                            line = self.serial_port.readline().decode().strip()
                            print(f"Received: {line}")
                            if line.startswith("POS:"):
                                _, coords = line.split(":")
                                x_str, y_str = coords.split(",")
                                x = float(x_str)
                                y = float(y_str)
                                self.current_position = (x, y)
                                self.master.after(0, self.update_position_label, x, y)
                                try:
                                    rel_x = x - BASE_X
                                    rel_y = y - BASE_Y
                                    d = (rel_x*rel_x + rel_y*rel_y - ARM1*ARM1 - ARM2*ARM2) / (2*ARM1*ARM2)
                                    if abs(d) <= 1:
                                        theta2 = math.acos(d)
                                        theta1 = math.atan2(rel_y, rel_x) - math.atan2(
                                            ARM2*math.sin(theta2), ARM1 + ARM2*math.cos(theta2))
                                        self.master.after(0, self.arm_sim.draw_arm, theta1, theta2)
                                except Exception as e:
                                    print(f"IK error: {e}")
                except (OSError, AttributeError) as e:
                    print(f"Serial port error during read: {e}")
                    break
                except Exception as e:
                    print(f"Serial read error: {e}")
            time.sleep(0.05)

    def update_position_label(self, x, y):
        self.pos_label.config(text=f"X: {x:.1f} mm, Y: {y:.1f} mm")

    # ------------------------------------------------------------------ G-code simulation

    def clear_gcode(self):
        self.gcode_text.delete("1.0", tk.END)

    def load_example(self):
        self.gcode_text.delete("1.0", tk.END)
        example = """G21 ; mm mode
G90 ; absolute positioning
G0 X0 Y0
G1 X75 Y0 F200
G1 X75 Y-75
G1 X0 Y-75
G1 X0 Y0
G0 X100 Y-50
G1 X150 Y0
M2"""
        self.gcode_text.insert("1.0", example)

    def run_gcode(self):
        gcode = self.gcode_text.get("1.0", tk.END)
        self.gcode_executor = GCodeExecutor(self.arm_sim)
        segments, _ = self.gcode_executor.run(gcode)
        self.animate_gcode_path(segments)

    def animate_gcode_path(self, segments):
        def animate_step(seg_idx=0, point_idx=0):
            if seg_idx >= len(segments):
                return
            seg = segments[seg_idx]
            pts = seg['points']
            reach = seg['reachable']

            if point_idx >= len(pts):
                self.master.after(10, animate_step, seg_idx + 1, 0)
                return

            x, y = pts[point_idx]
            if reach[point_idx]:
                try:
                    rel_x = x - BASE_X
                    rel_y = y - BASE_Y
                    d = (rel_x*rel_x + rel_y*rel_y - ARM1*ARM1 - ARM2*ARM2) / (2*ARM1*ARM2)
                    if abs(d) <= 1:
                        theta2 = math.acos(d)
                        theta1 = math.atan2(rel_y, rel_x) - math.atan2(
                            ARM2*math.sin(theta2), ARM1 + ARM2*math.cos(theta2))
                        self.arm_sim.draw_arm(theta1, theta2)
                except:
                    pass

            self.master.after(20, animate_step, seg_idx, point_idx + 1)

        animate_step()

    # ------------------------------------------------------------------ G-code send to robot

    def stop_gcode_send(self):
        """Abort the in-progress G-code send."""
        self.gcode_sending = False
        self.log(">>> Send aborted by user")
        self.master.after(0, lambda: self.send_gcode_btn.config(state="normal"))
        self.master.after(0, lambda: self.stop_gcode_btn.config(state="disabled"))

    def send_gcode_to_robot(self):
        if not self.serial_port or not self.serial_port.is_open:
            messagebox.showwarning("Not Connected", "Please connect to ESP32 first")
            return
        if self.gcode_sending:
            messagebox.showinfo("Busy", "Already sending G-code. Press Stop Send to abort.")
            return

        gcode = self.gcode_text.get("1.0", tk.END)

        # Modal codes that the GUI's GCodeExecutor handles but the ESP32 doesn't need
        GUI_ONLY_CODES = {'G20', 'G21', 'G90', 'G91', 'M2', 'M30'}

        # Codes that signal end of program — stop sending after these
        END_CODES = {'M2', 'M30'}

        self.gcode_sending = True
        self.send_gcode_btn.config(state="disabled")
        self.stop_gcode_btn.config(state="normal")
        self.log("=== Starting G-code send ===")

        # Track modal state so we can pass correct absolute/relative coords
        absolute_mode = True
        units_mm = True
        current_x = 0.0
        current_y = 0.0
        feed_rate = 100.0  # mm/min default

        def send_lines():
            nonlocal absolute_mode, units_mm, current_x, current_y, feed_rate

            lines = gcode.splitlines()
            total = len(lines)

            for line_num, raw in enumerate(lines, start=1):
                if not self.gcode_sending:
                    break

                # Strip comments and whitespace
                line = re.sub(r'\(.*?\)', '', raw)
                line = re.sub(r';.*', '', line).strip().upper()
                if not line:
                    continue

                # Parse all tokens
                tokens = {m.group(1): float(m.group(2))
                          for m in re.finditer(r'([A-Z])\s*(-?\d+\.?\d*)', line)}

                g_codes = [int(v) for k, v in
                           re.findall(r'([G])\s*(\d+\.?\d*)', line)]
                m_codes = [int(v) for k, v in
                           re.findall(r'([M])\s*(\d+\.?\d*)', line)]

                # ---- Handle modal state locally ----
                for g in g_codes:
                    if g == 90:
                        absolute_mode = True
                    elif g == 91:
                        absolute_mode = False
                    elif g == 20:
                        units_mm = False
                    elif g == 21:
                        units_mm = True

                # ---- Check for end-of-program ----
                if any(f'M{c}' in line.replace(' ', '') or
                       (f'M {c}' in line) for c in m_codes if c in {2, 30}):
                    self.log(f"Line {line_num}: M2/M30 — program end, stopping send")
                    break

                # ---- Skip GUI-only modal codes ----
                stripped = line.replace(' ', '')
                if any(stripped.startswith(code.replace(' ', '')) for code in GUI_ONLY_CODES):
                    self.log(f"Line {line_num}: Skipping (GUI-only): {line}")
                    continue

                # ---- Only send G0 / G1 motion commands ----
                has_motion = any(g in (0, 1) for g in g_codes)
                if not has_motion:
                    self.log(f"Line {line_num}: Skipping (no motion): {line}")
                    continue

                # ---- Resolve target XY ----
                INCH_TO_MM = 25.4
                def to_mm(v):
                    return v * INCH_TO_MM if not units_mm else v

                if absolute_mode:
                    tx = to_mm(tokens.get('X', current_x))
                    ty = to_mm(tokens.get('Y', current_y))
                else:
                    tx = current_x + to_mm(tokens.get('X', 0.0))
                    ty = current_y + to_mm(tokens.get('Y', 0.0))

                if 'F' in tokens:
                    feed_rate = to_mm(tokens['F'])

                # ---- Build clean command for the ESP32 ----
                motion_type = 'G0' if 0 in g_codes else 'G1'
                cmd = f"{motion_type} X{tx:.3f} Y{ty:.3f}"
                if motion_type == 'G1':
                    cmd += f" F{feed_rate:.1f}"

                self.log(f"Line {line_num}/{total}: Sending → {cmd}")

                # Send the command
                if self.serial_port and self.serial_port.is_open:
                    self.serial_port.write((cmd + "\n").encode())
                else:
                    self.log("ERROR: Serial port closed mid-send")
                    break

                # ---- Wait for robot acknowledgment ----
                # The ESP32 sends "STATE:IDLE" after every G0/G1 completes,
                # and "POS:x,y" during movement.
                deadline = time.time() + 60  # 60 s max per move
                ack_received = False
                while time.time() < deadline and self.gcode_sending:
                    if self.serial_port and self.serial_port.is_open:
                        try:
                            if self.serial_port.in_waiting:
                                resp = self.serial_port.readline().decode().strip()
                                if resp:
                                    self.log(f"  Robot: {resp}")
                                # "STATE:IDLE" is sent by the ESP32 after every move completes
                                if "STATE:IDLE" in resp or resp == "OK":
                                    ack_received = True
                                    break
                                # Also accept a POS: update as confirmation of
                                # arrival (fallback if firmware doesn't send STATE:IDLE)
                                if resp.startswith("POS:"):
                                    # give a brief window for STATE:IDLE to follow
                                    time.sleep(0.1)
                                    ack_received = True
                                    break
                                if "UNREACHABLE" in resp or "ERROR" in resp.upper():
                                    self.log(f"  WARNING: {resp}")
                                    ack_received = True  # move on even on error
                                    break
                        except Exception as e:
                            self.log(f"  Serial error: {e}")
                            break
                    time.sleep(0.02)

                if not ack_received:
                    self.log(f"  WARNING: Timeout waiting for ack on line {line_num} — continuing")

                current_x, current_y = tx, ty

            # Done
            self.gcode_sending = False
            self.log("=== G-code send complete ===")
            self.master.after(0, lambda: self.send_gcode_btn.config(state="normal"))
            self.master.after(0, lambda: self.stop_gcode_btn.config(state="disabled"))

        threading.Thread(target=send_lines, daemon=True).start()

    # ------------------------------------------------------------------ close

    def on_closing(self):
        self.running = False
        self.gcode_sending = False
        time.sleep(0.2)
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
            time.sleep(0.1)
        self.master.destroy()


# ---------------- RUN ----------------
if __name__ == "__main__":
    root = tk.Tk()
    gui = SCARAGUI(root)
    root.protocol("WM_DELETE_WINDOW", gui.on_closing)
    root.mainloop()