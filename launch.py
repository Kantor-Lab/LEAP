#!/usr/bin/env python3
"""
LEAP Robot Launch Control Panel
High-contrast field UI for ROS2 subsystem management.
Designed for outdoor / bright sunlight use.
Run without sourcing your ROS workspace — it sources automatically.
"""

import tkinter as tk
from tkinter import font as tkfont
import subprocess
import threading
import os
import signal
import time
from datetime import datetime

# Attempt to import rclpy for native node subscription
try:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import NavSatFix
    RCLPY_AVAILABLE = True
except ImportError:
    RCLPY_AVAILABLE = False

# ═══════════════════════════════════════════════════════════
#  CONFIG — edit to match your setup
# ═══════════════════════════════════════════════════════════

WINDOW_WIDTH  = 1000
WINDOW_HEIGHT = 900

ROS_SETUP_SCRIPT = os.path.expanduser("~/LEAP/ros2_ws/install/setup.bash")

LOG_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "logs")

SUBSYSTEMS = [
    {
        "id": "zenoh",
        "label": "ZENOH",
        "description": "RMW Zenoh DDS bridge",
        "cmd": "ros2 run rmw_zenoh_cpp rmw_zenohd",
        "toggles": [],
    },
    {
        "id": "foxglove",
        "label": "FOXGLOVE",
        "description": "Foxglove visualization bridge",
        "cmd": "ros2 run foxglove_bridge foxglove_bridge --ros-args -p asset_uri_allowlist:=\"['package://.*']\"",
        "toggles": [],
    },
    {
        "id": "sensors",
        "label": "SENSORS",
        "description": "Amiga sensor stack",
        "cmd": "ros2 launch leap_launch amiga_sensors.launch.py",
        "toggles": [
            {"id": "yolo", "label": "YOLO", "arg": "yolo:=true",  "off_arg": "yolo:=false"},
            {"id": "rviz", "label": "RVIZ", "arg": "rviz:=true",  "off_arg": "rviz:=false"},
        ],
    },
    {
        "id": "localization",
        "label": "LOCALIZATION",
        "description": "Amiga localization stack",
        "cmd": "ros2 launch leap_launch amiga_localization.launch.py",
        "toggles": [
            {
                "id": "map_mode",
                "label": "MAP MODE",
                "arg": "use_map:=true",
                "off_arg": "use_map:=false",
                "off_label": "ODOM ONLY",
                "on_label": "ODOM + MAP",
            },
        ],
    },
    {
        "id": "navigation",
        "label": "NAVIGATION",
        "description": "Amiga navigation stack",
        "cmd": "ros2 launch leap_launch amiga_navigation.launch.py",
        "toggles": [],
    },
    {
        "id": "record",
        "label": "RECORD",
        "description": "Amiga rosbag recording stack",
        "cmd": "ros2 launch leap_launch amiga_record.launch.py",
        "toggles": [],
    },
]

# ═══════════════════════════════════════════════════════════
#  COLORS — high contrast / sunlight readable
# ═══════════════════════════════════════════════════════════
C = {
    # Backgrounds
    "bg":           "#E8EDF0",   # light blue-grey page bg
    "bg2":          "#F5F7F9",   # lighter — log panel bg
    "panel":        "#FFFFFF",   # card bg
    "panel_on":     "#9BC99B",   # card bg when enabled
    "header_bg":    "#1A2B3C",   # dark navy header bar

    # Borders
    "border":       "#B0BEC5",   # card border
    "border_on":    "#2E7D32",   # card border when enabled
    "divider":      "#90A4AE",   # section label color

    # Text
    "text":         "#0D1B2A",   # primary text — near black
    "text_dim":     "#455A64",   # secondary/description text
    "text_on_dark": "#E8EDF0",   # text on dark backgrounds
    "text_muted":   "#78909C",   # timestamps, muted info

    # Accents
    "green":        "#002900",   # almost-black text for high contrast
    "green_bg":     "#00E676",   # highly vivid neon/lime green
    "green_btn":    "#00C853",   # vivid launch button bg
    "green_btn_hi": "#00E676",   # launch button hover (brighter)

    "amber":        "#E65100",   # warnings / toggle on (dark orange — readable on white)
    "amber_bg":     "#FFE0B2",   # amber pill bg

    "red":          "#B71C1C",   # error / stop
    "red_bg":       "#FFCDD2",   # red pill bg
    "red_btn":      "#C62828",   # stop button
    "red_btn_hi":   "#B71C1C",

    "blue":         "#0D47A1",   # neutral info
    "blue_bg":      "#BBDEFB",

    "grey_btn":     "#546E7A",   # inactive button
    "grey_btn_hi":  "#37474F",
    "grey_bg":      "#ECEFF1",   # inactive card bg
    "grey_text":    "#607D8B",   # inactive text

    # Log colors (on light bg)
    "log_system":   "#0D47A1",
    "log_info":     "#1B5E20",
    "log_warn":     "#E65100",
    "log_error":    "#B71C1C",
    "log_proc":     "#455A64",
    "log_start":    "#1B5E20",
    "log_stop":     "#C62828",
}


# ═══════════════════════════════════════════════════════════
#  HELPERS
# ═══════════════════════════════════════════════════════════
def source_cmd(cmd):
    if os.path.exists(ROS_SETUP_SCRIPT):
        return f"source {ROS_SETUP_SCRIPT} && {cmd}"
    for fb in [
        "/opt/ros/humble/setup.bash",
        "/opt/ros/iron/setup.bash",
        "/opt/ros/jazzy/setup.bash",
        "/opt/ros/rolling/setup.bash",
    ]:
        if os.path.exists(fb):
            return f"source {fb} && {cmd}"
    return cmd

def ensure_log_dir():
    os.makedirs(LOG_DIR, exist_ok=True)

def open_session_log():
    ensure_log_dir()
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    path = os.path.join(LOG_DIR, f"session_{ts}.log")
    return open(path, "w", buffering=1)


# ═══════════════════════════════════════════════════════════
#  ROS 2 GPS WORKER
# ═══════════════════════════════════════════════════════════
class GPSWorker:
    def __init__(self, callback, log_fn):
        self.callback = callback
        self.log_fn = log_fn
        self.node = None
        self.running = False
        self.thread = None

    def start(self):
        if self.running or not RCLPY_AVAILABLE:
            return
        self.running = True
        self.thread = threading.Thread(target=self._run_ros, daemon=True)
        self.thread.start()

    def stop(self):
        if not self.running:
            return
        self.running = False
        if self.node:
            self.node.destroy_node()
            self.node = None

    def _run_ros(self):
        try:
            if not rclpy.ok():
                rclpy.init()
            self.node = Node('gui_gps_status_node')
            self.node.create_subscription(NavSatFix, '/fix', self._cb, 10)
            self.log_fn("[GPS] Subscriber thread started", "system")
            while self.running and rclpy.ok():
                try:
                    rclpy.spin_once(self.node, timeout_sec=0.1)
                except rclpy.executors.ExternalShutdownException:
                    break
        except Exception as e:
            self.log_fn(f"[GPS] Worker error: {e}", "error")
        finally:
            if self.node:
                self.node.destroy_node()

    def _cb(self, msg):
        if msg.status.status < 0:
            status = "NO FIX"
        elif sum(msg.position_covariance) < 0.5:
            status = "FIX"
        else:
            status = "FLOAT"
        self.callback(status)


# ═══════════════════════════════════════════════════════════
#  PROCESS MANAGER
# ═══════════════════════════════════════════════════════════
class ProcessManager:
    def __init__(self):
        self.procs = {}

    def launch(self, sid, cmd, log_fn):
        if sid in self.procs and self.procs[sid].poll() is None:
            log_fn(f"[{sid}] already running", "warn")
            return False
        try:
            proc = subprocess.Popen(
                ["bash", "-c", source_cmd(cmd)],
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                preexec_fn=os.setsid,
                text=True, bufsize=1,
            )
            self.procs[sid] = proc
            threading.Thread(target=self._stream, args=(sid, proc, log_fn), daemon=True).start()
            return True
        except Exception as e:
            log_fn(f"[{sid}] LAUNCH FAILED: {e}", "error")
            return False

    def _stream(self, sid, proc, log_fn):
        try:
            for line in proc.stdout:
                log_fn(f"[{sid}] {line.rstrip()}", "proc")
        except Exception:
            pass
        rc = proc.wait()
        log_fn(f"[{sid}] exited (rc={rc})", "warn" if rc != 0 else "info")

    def kill(self, sid, log_fn):
        proc = self.procs.get(sid)
        if proc is None or proc.poll() is not None:
            log_fn(f"[{sid}] not running", "warn")
            return
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
            log_fn(f"[{sid}] SIGTERM sent", "info")
        except Exception as e:
            log_fn(f"[{sid}] kill error: {e}", "error")

    def kill_all(self, log_fn):
        for sid in list(self.procs.keys()):
            self.kill(sid, log_fn)

    def is_running(self, sid):
        proc = self.procs.get(sid)
        return proc is not None and proc.poll() is None


# ═══════════════════════════════════════════════════════════
#  MAIN GUI
# ═══════════════════════════════════════════════════════════
class RobotLauncherApp:
    def __init__(self, root):
        self.root = root
        self.pm = ProcessManager()
        self._session_log = open_session_log()
        self.enabled = {s["id"]: tk.BooleanVar(value=False) for s in SUBSYSTEMS}
        self.toggle_state = {
            s["id"]: {t["id"]: tk.BooleanVar(value=False) for t in s["toggles"]}
            for s in SUBSYSTEMS
        }
        self.gps_worker = None
        self.gps_last_msg_time = 0

        self._build_window()
        self._start_clock()
        self._start_status_poll()
        self._log("SYSTEM INITIALIZED", "system")
        self._log(f"ROS setup  : {ROS_SETUP_SCRIPT}", "info")
        self._log(f"Session log: {self._session_log.name}", "info")
        self._log("Awaiting operator input...", "info")
        if not RCLPY_AVAILABLE:
            self._log("rclpy not found — source ROS to enable GPS indicator.", "warn")

    # ── Window ────────────────────────────────
    def _build_window(self):
        self.root.title("LEAP // ROBOT CONTROL")
        self.root.configure(bg=C["bg"])
        self.root.geometry("1000x700")  # Shrunk default height for small monitors
        self.root.minsize(700, 400)
        self.root.resizable(True, True)

        try:
            self.font_mono   = tkfont.Font(family="Courier New", size=10)
            self.font_small  = tkfont.Font(family="Courier New", size=9)
            self.font_label  = tkfont.Font(family="Courier New", size=12, weight="bold")
            self.font_card   = tkfont.Font(family="Courier New", size=13, weight="bold")
            self.font_btn    = tkfont.Font(family="Courier New", size=11, weight="bold")
            self.font_big    = tkfont.Font(family="Courier New", size=15, weight="bold")
            self.font_toggle = tkfont.Font(family="Courier New", size=10, weight="bold")
            self.font_title  = tkfont.Font(family="Courier New", size=14, weight="bold")
        except Exception:
            self.font_mono   = tkfont.Font(family="TkFixedFont", size=10)
            self.font_small  = tkfont.Font(family="TkFixedFont", size=9)
            self.font_label  = tkfont.Font(family="TkFixedFont", size=12, weight="bold")
            self.font_card   = tkfont.Font(family="TkFixedFont", size=13, weight="bold")
            self.font_btn    = tkfont.Font(family="TkFixedFont", size=11, weight="bold")
            self.font_big    = tkfont.Font(family="TkFixedFont", size=15, weight="bold")
            self.font_toggle = tkfont.Font(family="TkFixedFont", size=10, weight="bold")
            self.font_title  = tkfont.Font(family="TkFixedFont", size=14, weight="bold")

        # ── SCROLLABLE WRAPPER ──
        self.main_container = tk.Frame(self.root, bg=C["bg"])
        self.main_container.pack(fill="both", expand=True)

        self.canvas = tk.Canvas(self.main_container, bg=C["bg"], highlightthickness=0)
        self.v_scrollbar = tk.Scrollbar(self.main_container, orient="vertical", command=self.canvas.yview)
        self.canvas.configure(yscrollcommand=self.v_scrollbar.set)

        self.v_scrollbar.pack(side="right", fill="y")
        self.canvas.pack(side="left", fill="both", expand=True)

        self.content_frame = tk.Frame(self.canvas, bg=C["bg"])
        self.canvas_window = self.canvas.create_window((0, 0), window=self.content_frame, anchor="nw")

        self.content_frame.bind("<Configure>", lambda e: self.canvas.configure(scrollregion=self.canvas.bbox("all")))
        self.canvas.bind("<Configure>", lambda e: self.canvas.itemconfig(self.canvas_window, width=e.width))

        # Cross-platform mousewheel bindings
        self.canvas.bind_all("<MouseWheel>", self._on_mousewheel)
        self.canvas.bind_all("<Button-4>", self._on_mousewheel)
        self.canvas.bind_all("<Button-5>", self._on_mousewheel)

        # ── TOUCH/DRAG SCROLLING BINDINGS ──
        self.canvas.bind_all("<ButtonPress-1>", self._on_touch_press)
        self.canvas.bind_all("<B1-Motion>", self._on_touch_drag)
        self.canvas.bind_all("<ButtonRelease-1>", self._on_touch_release)

        # ── BYPASSING PANEDWINDOW ──
        self.top_frame = tk.Frame(self.content_frame, bg=C["bg"])
        self.top_frame.pack(fill="x", padx=10, pady=(8, 0))
        
        self.bot_frame = tk.Frame(self.content_frame, bg=C["bg"])
        self.bot_frame.pack(fill="x", padx=10, pady=8)

        self._build_header()
        self._build_subsystems()
        self._build_control_bar()
        self._build_log()

    def _on_mousewheel(self, event):
        # Allow scrolling inside the log terminal independently 
        if isinstance(event.widget, tk.Text):
            return
            
        if event.num == 4 or event.delta > 0:
            self.canvas.yview_scroll(-1, "units")
        elif event.num == 5 or event.delta < 0:
            self.canvas.yview_scroll(1, "units")

    def _on_touch_press(self, event):
        # Ignore if touching the log text box or scrollbar
        if isinstance(event.widget, (tk.Text, tk.Scrollbar)):
            return
        
        # Record fresh start points
        self._touch_start_y = event.y_root
        self._touch_start_yview = self.canvas.yview()[0]
        self._is_dragging = False

    def _on_touch_drag(self, event):
        if isinstance(event.widget, (tk.Text, tk.Scrollbar)):
            return
            
        # If there is no valid start point (e.g., release was triggered), abort immediately
        if getattr(self, "_touch_start_y", None) is None:
            return
            
        delta_y = event.y_root - self._touch_start_y
        
        # ── DEADZONE CHECK ──
        if not self._is_dragging:
            if abs(delta_y) > 10:
                self._is_dragging = True
            return
            
        # ── ACTIVE DRAG ──
        if self._is_dragging:
            content_height = self.content_frame.winfo_height()
            
            if content_height <= self.canvas.winfo_height():
                return
                
            fraction_delta = delta_y / content_height
            new_yview = self._touch_start_yview - fraction_delta
            
            self.canvas.yview_moveto(new_yview)

    def _on_touch_release(self, event):
        # The moment the finger lifts, wipe the tracking variables 
        # so phantom drag events are ignored until a new press occurs.
        self._is_dragging = False
        self._touch_start_y = None

    # ── Header ────────────────────────────────
    def _build_header(self):
        hdr = tk.Frame(self.top_frame, bg=C["header_bg"], pady=10)
        hdr.pack(fill="x")

        tk.Label(
            hdr, text="  LEAP  \\  ROBOT CONTROL PANEL",
            font=self.font_title, bg=C["header_bg"], fg=C["text_on_dark"],
            anchor="w",
        ).pack(side="left", padx=10)

        self.blink_var = tk.StringVar(value="●")
        tk.Label(
            hdr, textvariable=self.blink_var,
            font=self.font_label, bg=C["header_bg"], fg="#00E676"
        ).pack(side="right", padx=(0, 8))

        tk.Label(
            hdr, text="ONLINE",
            font=self.font_small, bg=C["header_bg"], fg="#80CBC4"
        ).pack(side="right")

        self.clock_var = tk.StringVar(value="")
        tk.Label(
            self.top_frame, textvariable=self.clock_var,
            font=self.font_small, bg=C["bg"], fg=C["text_muted"], anchor="w"
        ).pack(fill="x", padx=4, pady=(4, 0))

    # ── Subsystem cards ───────────────────────
    def _build_subsystems(self):
        outer = tk.Frame(self.top_frame, bg=C["bg"])
        outer.pack(fill="x", pady=(6, 2))

        lbl_row = tk.Frame(outer, bg=C["bg"])
        lbl_row.pack(fill="x", padx=4)
        tk.Label(
            lbl_row, text="SUBSYSTEMS",
            font=self.font_label, bg=C["bg"], fg=C["text_dim"]
        ).pack(side="left")
        tk.Frame(lbl_row, bg=C["divider"], height=2).pack(
            side="left", fill="x", expand=True, padx=(8, 0), pady=6
        )

        grid = tk.Frame(outer, bg=C["bg"])
        grid.pack(fill="x", padx=4, pady=4)
        grid.columnconfigure(0, weight=1)
        grid.columnconfigure(1, weight=1)

        for i, sub in enumerate(SUBSYSTEMS):
            row, col = divmod(i, 2)
            card = self._build_card(grid, sub)
            card.grid(row=row, column=col, sticky="nsew", padx=5, pady=5)

    def _build_card(self, parent, sub):
        sid = sub["id"]

        # Outer shadow frame
        shadow = tk.Frame(parent, bg=C["border"], bd=0)
        card = tk.Frame(shadow, bg=C["panel"], padx=12, pady=10)
        card.pack(fill="both", expand=True, padx=1, pady=1)
        sub["_card"] = card
        sub["_shadow"] = shadow

        # ── Top row: toggle + label + status pill ──
        top = tk.Frame(card, bg=C["panel"])
        top.pack(fill="x")

        # ON/OFF toggle — MASSIVE for easy tapping outdoors
        toggle_btn = tk.Button(
            top, text="OFF",
            font=self.font_big,  # Upgraded to the larger font
            bg=C["grey_bg"], fg=C["grey_text"],
            activebackground=C["green_bg"], activeforeground=C["green"],
            relief="flat", bd=0, cursor="hand2",
            width=6, pady=14,    # Much taller and wider hit box
            command=lambda s=sub: self._toggle_subsystem(s),
        )
        toggle_btn.pack(side="left", padx=(0, 15))
        sub["_btn"] = toggle_btn

        tk.Label(
            top, text=sub["label"],
            font=self.font_card, bg=C["panel"], fg=C["text"],
        ).pack(side="left")

        # Status pill (running indicator)
        pill = tk.Label(
            top, text="  IDLE  ",
            font=self.font_small,
            bg=C["grey_bg"], fg=C["grey_text"],
            relief="flat", padx=6, pady=3,
        )
        pill.pack(side="right")
        sub["_pill"] = pill

        # Separator (Description removed, adjusted padding to compensate)
        tk.Frame(card, bg=C["border"], height=1).pack(fill="x", pady=(12, 6))

        # Toggles
        for tog in sub["toggles"]:
            self._build_toggle(card, sid, tog)

        # GPS widget for sensors card
        if sid == "sensors":
            gps_row = tk.Frame(card, bg=C["panel"])
            gps_row.pack(fill="x", pady=(4, 0))
            tk.Label(
                gps_row, text="GPS:",
                font=self.font_toggle, bg=C["panel"], fg=C["text_dim"]
            ).pack(side="left")
            init_text  = "NO RCLPY" if not RCLPY_AVAILABLE else "OFFLINE"
            init_bg    = C["grey_bg"]
            init_fg    = C["grey_text"]
            self.gps_label = tk.Label(
                gps_row, text=f"  {init_text}  ",
                font=self.font_toggle,
                bg=init_bg, fg=init_fg,
                padx=6, pady=2, relief="flat",
            )
            self.gps_label.pack(side="left", padx=(8, 0))

        return shadow

    def _build_toggle(self, parent, sid, tog):
        var = self.toggle_state[sid][tog["id"]]
        on_label  = tog.get("on_label",  tog["label"] + " ON")
        off_label = tog.get("off_label", tog["label"] + " OFF")

        row = tk.Frame(parent, bg=C["panel"])
        row.pack(fill="x", pady=3)

        def refresh():
            if var.get():
                b.config(
                    text=f"  ✓  {on_label}  ",
                    bg=C["amber_bg"], fg=C["amber"],
                    # Lock the active state to the ON colors
                    activebackground=C["amber_bg"], activeforeground=C["amber"],
                    relief="flat",
                )
            else:
                b.config(
                    text=f"  ○  {off_label}  ",
                    bg=C["grey_bg"], fg=C["grey_text"],
                    # Lock the active state to the OFF colors
                    activebackground=C["grey_bg"], activeforeground=C["grey_text"],
                    relief="flat",
                )

        b = tk.Button(
            row,
            font=self.font_toggle,
            relief="flat", bd=0, cursor="hand2",
            pady=5,
            command=lambda: [var.set(not var.get()), refresh()],
        )
        b.pack(side="left", fill="x", expand=True)
        tog["_btn"] = b
        
        # Call refresh immediately to set the initial colors
        refresh()

    def _toggle_subsystem(self, sub):
        self.enabled[sub["id"]].set(not self.enabled[sub["id"]].get())
        self._refresh_card(sub)

    def _refresh_card(self, sub):
        sid     = sub["id"]
        enabled = self.enabled[sid].get()
        running = self.pm.is_running(sid)

        # Toggle button
        if enabled:
            sub["_btn"].config(
                text="ON", bg=C["green_bg"], fg=C["green"],
                activebackground=C["green_bg"],
            )
        else:
            sub["_btn"].config(
                text="OFF", bg=C["grey_bg"], fg=C["grey_text"],
                activebackground=C["grey_bg"],
            )

        # Status pill
        if running:
            sub["_pill"].config(text="  RUNNING  ", bg=C["green_bg"], fg=C["green"])
        else:
            sub["_pill"].config(text="  IDLE  ",    bg=C["grey_bg"],  fg=C["grey_text"])

        # Card bg tint
        bg = C["panel_on"] if (enabled or running) else C["panel"]
        border = C["border_on"] if (enabled or running) else C["border"]
        sub["_card"].config(bg=bg)
        sub["_shadow"].config(bg=border)
        for child in sub["_card"].winfo_children():
            try:
                child.config(bg=bg)
            except Exception:
                pass

    # ── Control bar ───────────────────────────
    def _build_control_bar(self):
        bar = tk.Frame(self.top_frame, bg=C["bg"])
        bar.pack(fill="x", pady=(6, 4), padx=4)

        lbl_row = tk.Frame(bar, bg=C["bg"])
        lbl_row.pack(fill="x")
        tk.Label(
            lbl_row, text="CONTROL",
            font=self.font_label, bg=C["bg"], fg=C["text_dim"]
        ).pack(side="left")
        tk.Frame(lbl_row, bg=C["divider"], height=2).pack(
            side="left", fill="x", expand=True, padx=(8, 0), pady=6
        )

        btn_row = tk.Frame(bar, bg=C["bg"])
        btn_row.pack(fill="x", pady=(6, 0))

        self.start_btn = self._big_btn(
            btn_row, "▶   LAUNCH ENABLED",
            C["green_btn"], "#FFFFFF", C["green_btn_hi"],
            self._launch_enabled,
        )
        self.start_btn.pack(side="left", padx=(0, 10), ipady=4)

        self.stop_btn = self._big_btn(
            btn_row, "■   STOP ALL",
            C["grey_btn"], "#FFFFFF", C["grey_btn_hi"],
            self._stop_all,
        )
        self.stop_btn.pack(side="left", padx=(0, 10), ipady=4)
        self.stop_btn.config(state="disabled", cursor="arrow")

        self.clear_btn = self._big_btn(
            btn_row, "⌫   CLEAR LOG",
            C["grey_btn"], "#FFFFFF", C["grey_btn_hi"],
            self._clear_log,
        )
        self.clear_btn.pack(side="right", ipady=4)

    def _big_btn(self, parent, text, bg, fg, hover_bg, cmd):
        btn = tk.Button(
            parent, text=text,
            font=self.font_big,
            bg=bg, fg=fg,
            activebackground=hover_bg, activeforeground=fg,
            relief="flat", bd=0, cursor="hand2",
            padx=20, pady=10,
            command=cmd,
        )
        btn.bind("<Enter>", lambda e, b=btn, h=hover_bg: b.config(bg=h))
        btn.bind("<Leave>", lambda e, b=btn, ob=bg: b.config(bg=ob))
        return btn

    def _update_control_buttons(self):
        any_running = any(self.pm.is_running(s["id"]) for s in SUBSYSTEMS)
        if any_running:
            self.start_btn.config(
                state="disabled", cursor="arrow",
                bg=C["grey_btn"], text="▶   RUNNING..."
            )
            self.stop_btn.config(
                state="normal", cursor="hand2",
                bg=C["red_btn"], text="■   STOP ALL"
            )
        else:
            self.start_btn.config(
                state="normal", cursor="hand2",
                bg=C["green_btn"], text="▶   LAUNCH ENABLED"
            )
            self.stop_btn.config(
                state="disabled", cursor="arrow",
                bg=C["grey_btn"], text="■   STOP ALL"
            )

    # ── Log panel ─────────────────────────────
    def _build_log(self):
        outer = tk.Frame(self.bot_frame, bg=C["bg"])
        outer.pack(fill="both", expand=True, padx=4)

        lbl_row = tk.Frame(outer, bg=C["bg"])
        lbl_row.pack(fill="x")
        tk.Label(
            lbl_row, text="SYSTEM LOG",
            font=self.font_label, bg=C["bg"], fg=C["text_dim"]
        ).pack(side="left")
        tk.Frame(lbl_row, bg=C["divider"], height=2).pack(
            side="left", fill="x", expand=True, padx=(8, 0), pady=6
        )

        border = tk.Frame(outer, bg=C["border"], bd=0)
        border.pack(fill="both", expand=True, pady=4)

        inner = tk.Frame(border, bg=C["bg2"])
        inner.pack(fill="both", expand=True, padx=1, pady=1)

        self.log_text = tk.Text(
            inner,
            height=20,  # <-- THIS PREVENTS IT FROM TAKING OVER
            font=self.font_mono,
            bg=C["bg2"], fg=C["text"],
            insertbackground=C["text"],
            selectbackground=C["blue_bg"],
            relief="flat", bd=0,
            wrap="word",
            state="disabled",
            padx=8, pady=6,
        )
        self.log_text.pack(side="left", fill="both", expand=True)

        sb = tk.Scrollbar(inner, command=self.log_text.yview,
                          bg=C["bg2"], troughcolor=C["bg"])
        sb.pack(side="right", fill="y")
        self.log_text.config(yscrollcommand=sb.set)

        self.log_text.tag_config("system", foreground=C["log_system"], font=self.font_btn)
        self.log_text.tag_config("info",   foreground=C["log_info"])
        self.log_text.tag_config("warn",   foreground=C["log_warn"])
        self.log_text.tag_config("error",  foreground=C["log_error"], font=self.font_btn)
        self.log_text.tag_config("proc",   foreground=C["log_proc"])
        self.log_text.tag_config("start",  foreground=C["log_start"], font=self.font_btn)
        self.log_text.tag_config("stop",   foreground=C["log_stop"],  font=self.font_btn)

    # ── Logging ───────────────────────────────
    def _log(self, msg, tag="info"):
        ts   = datetime.now().strftime("%H:%M:%S")
        line = f"[{ts}] {msg}\n"
        try:
            self._session_log.write(line)
        except Exception:
            pass

        def _do():
            self.log_text.config(state="normal")
            self.log_text.insert("end", line, tag)
            if int(self.log_text.index("end-1c").split(".")[0]) > 600:
                self.log_text.delete("1.0", "100.0")
            self.log_text.see("end")
            self.log_text.config(state="disabled")
        self.root.after(0, _do)

    def _clear_log(self):
        self.log_text.config(state="normal")
        self.log_text.delete("1.0", "end")
        self.log_text.config(state="disabled")
        self._log("Log cleared (file log unaffected)", "info")

    # ── Actions ───────────────────────────────
    def _build_cmd(self, sub):
        cmd = sub["cmd"]
        for tog in sub["toggles"]:
            sid = sub["id"]
            cmd += f" {tog['arg'] if self.toggle_state[sid][tog['id']].get() else tog['off_arg']}"
        return cmd

    def _launch_enabled(self):
        if not any(self.enabled[s["id"]].get() for s in SUBSYSTEMS):
            self._log("No subsystems enabled — nothing to launch.", "warn")
            return
        self._log("=" * 52, "system")
        self._log("LAUNCH SEQUENCE INITIATED", "system")
        self._log("=" * 52, "system")

        for sub in SUBSYSTEMS:
            sid = sub["id"]
            if not self.enabled[sid].get():
                continue
            cmd = self._build_cmd(sub)
            self._log(f"STARTING {sub['label']}: {cmd}", "start")

            if sid == "sensors" and RCLPY_AVAILABLE:
                if not self.gps_worker:
                    self.gps_worker = GPSWorker(self._update_gps_status, self._log)
                self.gps_worker.start()
                self.gps_last_msg_time = time.time()
                self._gps_watchdog()

            def _do(s=sub, c=cmd):
                ok = self.pm.launch(s["id"], c, self._log)
                if ok:
                    self._refresh_card(s)
                    self.root.after(0, self._update_control_buttons)
            threading.Thread(target=_do, daemon=True).start()
            time.sleep(0.3)

    def _stop_all(self):
        self._log("=" * 52, "stop")
        self._log("STOP ALL INITIATED", "stop")
        self._log("=" * 52, "stop")
        if self.gps_worker:
            self.gps_worker.stop()
            self.gps_worker = None
        if hasattr(self, "gps_label"):
            self.gps_label.config(text="  OFFLINE  ", bg=C["grey_bg"], fg=C["grey_text"])

        def _do():
            self.pm.kill_all(self._log)
            time.sleep(1)
            for sub in SUBSYSTEMS:
                self.root.after(0, self._refresh_card, sub)
            self.root.after(1100, self._update_control_buttons)
        threading.Thread(target=_do, daemon=True).start()

    # ── GPS ───────────────────────────────────
    def _update_gps_status(self, status):
        cfg = {
            "FIX":    (C["green_bg"],  C["green"]),
            "FLOAT":  (C["amber_bg"],  C["amber"]),
            "NO FIX": (C["red_bg"],    C["red"]),
        }
        bg, fg = cfg.get(status, (C["grey_bg"], C["grey_text"]))
        def _do():
            self.gps_last_msg_time = time.time()
            if hasattr(self, "gps_label"):
                self.gps_label.config(text=f"  {status}  ", bg=bg, fg=fg)
        self.root.after(0, _do)

    def _gps_watchdog(self):
        if not self.gps_worker or not self.gps_worker.running:
            return
        if time.time() - self.gps_last_msg_time > 5.0:
            if hasattr(self, "gps_label"):
                self.gps_label.config(
                    text="  TIMEOUT  ", bg=C["red_bg"], fg=C["red"]
                )
        self.root.after(1000, self._gps_watchdog)

    # ── Periodic updates ──────────────────────
    def _start_clock(self):
        self._blink = True
        def tick():
            now = datetime.now().strftime("%Y-%m-%d  %H:%M:%S")
            self.clock_var.set(f"  {now}   |   WS: {ROS_SETUP_SCRIPT}")
            self._blink = not self._blink
            self.blink_var.set("●" if self._blink else "○")
            self.root.after(500, tick)
        tick()

    def _start_status_poll(self):
        def poll():
            for sub in SUBSYSTEMS:
                self._refresh_card(sub)
            self._update_control_buttons()
            self.root.after(2000, poll)
        poll()

    def on_close(self):
        self._log("Shutting down all processes...", "warn")
        if self.gps_worker:
            self.gps_worker.stop()
        self.pm.kill_all(self._log)
        time.sleep(0.5)
        try:
            self._session_log.close()
        except Exception:
            pass
        self.root.destroy()


# ═══════════════════════════════════════════════════════════
#  ENTRY POINT
# ═══════════════════════════════════════════════════════════
if __name__ == "__main__":
    root = tk.Tk()
    app = RobotLauncherApp(root)
    root.protocol("WM_DELETE_WINDOW", app.on_close)
    root.mainloop()