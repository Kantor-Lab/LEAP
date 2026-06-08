#!/usr/bin/env python3
"""
LEAP Robot Launch Control Panel
Retro hacker terminal UI for ROS2 subsystem management.
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

# ═══════════════════════════════════════════════════════════
#  CONFIG — edit to match your setup
# ═══════════════════════════════════════════════════════════

WINDOW_WIDTH  = 900
WINDOW_HEIGHT = 60

ROS_SETUP_SCRIPT = os.path.expanduser("~/LEAP/ros2_ws/install/setup.bash")
# Fallback system installs are tried automatically if the above doesn't exist.

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
]

# ═══════════════════════════════════════════════════════════
#  COLORS
# ═══════════════════════════════════════════════════════════
C = {
    "bg":          "#0a0f0a",
    "bg2":         "#0d150d",
    "panel":       "#0f1a0f",
    "border":      "#1a3a1a",
    "border_hi":   "#2a6a2a",
    "green":       "#00ff41",
    "green_dim":   "#00cc33",
    "green_dark":  "#004d14",
    "amber":       "#ffb000",
    "amber_dim":   "#cc8800",
    "red":         "#ff2222",
    "red_dim":     "#aa1111",
    "red_dark":    "#330808",
    "cyan":        "#00ffff",
    "white":       "#ccffcc",
    "grey":        "#336633",
    "grey_dark":   "#1a2e1a",
}


# ═══════════════════════════════════════════════════════════
#  HELPERS
# ═══════════════════════════════════════════════════════════
def source_cmd(cmd):
    """Prepend ROS workspace sourcing to a command."""
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
    """Open a new per-session log file and return the file handle."""
    ensure_log_dir()
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    path = os.path.join(LOG_DIR, f"session_{ts}.log")
    return open(path, "w", buffering=1)  # line-buffered


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
        full_cmd = source_cmd(cmd)
        try:
            proc = subprocess.Popen(
                ["bash", "-c", full_cmd],
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                preexec_fn=os.setsid,
                text=True,
                bufsize=1,
            )
            self.procs[sid] = proc
            t = threading.Thread(
                target=self._stream, args=(sid, proc, log_fn), daemon=True
            )
            t.start()
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
        log_fn(f"[{sid}] process exited (rc={rc})", "warn" if rc != 0 else "info")

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

        self._build_window()
        self._start_clock()
        self._start_status_poll()
        self._log("SYSTEM INITIALIZED", "system")
        self._log(f"ROS setup  : {ROS_SETUP_SCRIPT}", "info")
        self._log(f"Session log: {self._session_log.name}", "info")
        self._log("Awaiting operator input...", "info")

    # ── Window ────────────────────────────────
    def _build_window(self):
        self.root.title("LEAP // ROBOT CONTROL")
        self.root.configure(bg=C["bg"])
        self.root.geometry(f"{WINDOW_WIDTH}x{WINDOW_HEIGHT}")
        self.root.minsize(700, 500)
        self.root.resizable(True, True)

        try:
            self.font_mono  = tkfont.Font(family="Courier New", size=10)
            self.font_label = tkfont.Font(family="Courier New", size=11, weight="bold")
            self.font_small = tkfont.Font(family="Courier New", size=8)
            self.font_btn   = tkfont.Font(family="Courier New", size=10, weight="bold")
            self.font_big   = tkfont.Font(family="Courier New", size=13, weight="bold")
        except Exception:
            self.font_mono  = tkfont.Font(family="TkFixedFont", size=10)
            self.font_label = tkfont.Font(family="TkFixedFont", size=11, weight="bold")
            self.font_small = tkfont.Font(family="TkFixedFont", size=8)
            self.font_btn   = tkfont.Font(family="TkFixedFont", size=10, weight="bold")
            self.font_big   = tkfont.Font(family="TkFixedFont", size=13, weight="bold")

        # PanedWindow: top pane = controls (fixed), bottom pane = log (flexible)
        self.paned = tk.PanedWindow(
            self.root,
            orient=tk.VERTICAL,
            bg=C["bg"],
            sashwidth=6,
            sashrelief="flat",
            sashpad=2,
        )
        self.paned.pack(fill="both", expand=True, padx=10, pady=8)

        self.top_frame = tk.Frame(self.paned, bg=C["bg"])
        self.bot_frame = tk.Frame(self.paned, bg=C["bg"])

        self.paned.add(self.top_frame, stretch="never")
        self.paned.add(self.bot_frame, stretch="always", minsize=80)

        self._build_header()
        self._build_subsystems()
        self._build_control_bar()
        self._build_log()

    # ── Header ────────────────────────────────
    def _build_header(self):
        title_row = tk.Frame(self.top_frame, bg=C["border"], pady=5)
        title_row.pack(fill="x")

        tk.Label(
            title_row,
            text="  LEAP \\ ROBOT CONTROL PANEL",
            font=self.font_label,
            bg=C["border"], fg=C["green"],
            anchor="w",
        ).pack(side="left", padx=6)

        self.blink_var = tk.StringVar(value="█")
        tk.Label(
            title_row, textvariable=self.blink_var,
            font=self.font_small, bg=C["border"], fg=C["green"]
        ).pack(side="right", padx=(0, 6))

        tk.Label(
            title_row, text="● ONLINE",
            font=self.font_small, bg=C["border"], fg=C["green_dim"]
        ).pack(side="right")

        self.clock_var = tk.StringVar(value="")
        tk.Label(
            self.top_frame, textvariable=self.clock_var,
            font=self.font_small, bg=C["bg"], fg=C["grey"], anchor="w"
        ).pack(fill="x", pady=(3, 0))

    # ── Subsystem cards ───────────────────────
    def _build_subsystems(self):
        outer = tk.Frame(self.top_frame, bg=C["bg"])
        outer.pack(fill="x", pady=(6, 2))

        tk.Label(
            outer,
            text="── SUBSYSTEMS ──────────────────────────────────────────",
            font=self.font_small, bg=C["bg"], fg=C["border_hi"], anchor="w"
        ).pack(fill="x")

        grid = tk.Frame(outer, bg=C["bg"])
        grid.pack(fill="x", pady=4)
        grid.columnconfigure(0, weight=1)
        grid.columnconfigure(1, weight=1)

        for i, sub in enumerate(SUBSYSTEMS):
            row, col = divmod(i, 2)
            card = self._build_card(grid, sub)
            card.grid(row=row, column=col, sticky="nsew", padx=4, pady=3)

    def _build_card(self, parent, sub):
        sid = sub["id"]
        outer = tk.Frame(parent, bg=C["border"])
        inner = tk.Frame(outer, bg=C["panel"], padx=10, pady=7)
        inner.pack(fill="both", expand=True, padx=1, pady=1)

        top = tk.Frame(inner, bg=C["panel"])
        top.pack(fill="x")

        btn = tk.Button(
            top, text="[ OFF ]",
            font=self.font_btn,
            bg=C["grey_dark"], fg=C["grey"],
            activebackground=C["green_dark"], activeforeground=C["green"],
            relief="flat", bd=0, cursor="hand2",
            command=lambda s=sub: self._toggle_subsystem(s),
            padx=6, pady=2,
        )
        btn.pack(side="left")
        sub["_btn"] = btn

        tk.Label(
            top, text=f"  {sub['label']}",
            font=self.font_label, bg=C["panel"], fg=C["white"]
        ).pack(side="left")

        dot = tk.Label(top, text="◉", font=self.font_small, bg=C["panel"], fg=C["grey"])
        dot.pack(side="right")
        sub["_status_dot"] = dot

        tk.Label(
            inner, text=sub["description"],
            font=self.font_small, bg=C["panel"], fg=C["grey"], anchor="w"
        ).pack(fill="x", pady=(2, 4))

        for tog in sub["toggles"]:
            self._build_toggle(inner, sid, tog)

        return outer

    def _build_toggle(self, parent, sid, tog):
        row = tk.Frame(parent, bg=C["panel"])
        row.pack(fill="x", pady=1)
        var = self.toggle_state[sid][tog["id"]]
        on_label  = tog.get("on_label",  tog["label"] + " ON")
        off_label = tog.get("off_label", tog["label"] + " OFF")

        def refresh(v=None):
            if var.get():
                b.config(text=f"[✓] {on_label}",  fg=C["amber"],  bg=C["grey_dark"])
            else:
                b.config(text=f"[ ] {off_label}", fg=C["grey"],   bg=C["panel"])

        b = tk.Button(
            row, text=f"[ ] {off_label}",
            font=self.font_small,
            bg=C["panel"], fg=C["grey"],
            activebackground=C["grey_dark"], activeforeground=C["amber"],
            relief="flat", bd=0, cursor="hand2",
            command=lambda: [var.set(not var.get()), refresh()],
        )
        b.pack(side="left")
        tog["_btn"] = b
        refresh()

    def _toggle_subsystem(self, sub):
        sid = sub["id"]
        self.enabled[sid].set(not self.enabled[sid].get())
        self._refresh_card(sub)

    def _refresh_card(self, sub):
        sid = sub["id"]
        enabled = self.enabled[sid].get()
        running = self.pm.is_running(sid)

        sub["_btn"].config(
            text="[ ON  ]" if enabled else "[ OFF ]",
            bg=C["green_dark"] if enabled else C["grey_dark"],
            fg=C["green"]      if enabled else C["grey"],
        )
        sub["_status_dot"].config(fg=C["green"] if running else C["grey"])

    # ── Control bar ───────────────────────────
    def _build_control_bar(self):
        bar = tk.Frame(self.top_frame, bg=C["bg"])
        bar.pack(fill="x", pady=(4, 2))

        tk.Label(
            bar,
            text="── CONTROL ─────────────────────────────────────────────",
            font=self.font_small, bg=C["bg"], fg=C["border_hi"], anchor="w"
        ).pack(fill="x")

        btn_row = tk.Frame(bar, bg=C["bg"])
        btn_row.pack(fill="x", pady=5)

        self.start_btn = self._big_btn(
            btn_row, "▶  LAUNCH ENABLED",
            C["green_dark"], C["green"], self._launch_enabled
        )
        self.start_btn.pack(side="left", padx=(0, 8))

        self.stop_btn = self._big_btn(
            btn_row, "■  STOP ALL",
            C["grey_dark"], C["grey"], self._stop_all
        )
        self.stop_btn.pack(side="left", padx=(0, 8))
        self.stop_btn.config(state="disabled", cursor="arrow")

        self.clear_btn = self._big_btn(
            btn_row, "⌫  CLEAR LOG",
            C["grey_dark"], C["grey"], self._clear_log
        )
        self.clear_btn.pack(side="right")

    def _big_btn(self, parent, text, bg, fg, cmd):
        btn = tk.Button(
            parent, text=text,
            font=self.font_big,
            bg=bg, fg=fg,
            activebackground=C["bg"], activeforeground=fg,
            relief="flat", bd=0, cursor="hand2",
            padx=16, pady=7,
            command=cmd,
        )
        btn.bind("<Enter>", lambda e, b=btn, f=fg: b.config(bg=f, fg=C["bg"]))
        btn.bind("<Leave>", lambda e, b=btn, ob=bg, of=fg: b.config(bg=ob, fg=of))
        return btn

    def _update_control_buttons(self):
        any_running = any(self.pm.is_running(s["id"]) for s in SUBSYSTEMS)
        if any_running:
            self.start_btn.config(
                state="disabled", cursor="arrow",
                bg=C["grey_dark"], fg=C["grey"], text="▶  RUNNING..."
            )
            self.stop_btn.config(
                state="normal", cursor="hand2",
                bg=C["red_dark"], fg=C["red"], text="■  STOP ALL"
            )
        else:
            self.start_btn.config(
                state="normal", cursor="hand2",
                bg=C["green_dark"], fg=C["green"], text="▶  LAUNCH ENABLED"
            )
            self.stop_btn.config(
                state="disabled", cursor="arrow",
                bg=C["grey_dark"], fg=C["grey"], text="■  STOP ALL"
            )

    # ── Log panel ─────────────────────────────
    def _build_log(self):
        outer = tk.Frame(self.bot_frame, bg=C["bg"])
        outer.pack(fill="both", expand=True)

        tk.Label(
            outer,
            text="── SYSTEM LOG ──────────────────────────────────────────",
            font=self.font_small, bg=C["bg"], fg=C["border_hi"], anchor="w"
        ).pack(fill="x")

        border = tk.Frame(outer, bg=C["border"])
        border.pack(fill="both", expand=True, pady=3)

        inner = tk.Frame(border, bg=C["bg2"])
        inner.pack(fill="both", expand=True, padx=1, pady=1)

        self.log_text = tk.Text(
            inner,
            font=self.font_mono,
            bg=C["bg2"], fg=C["green_dim"],
            insertbackground=C["green"],
            selectbackground=C["green_dark"],
            relief="flat", bd=0,
            wrap="word",
            state="disabled",
        )
        self.log_text.pack(side="left", fill="both", expand=True)

        sb = tk.Scrollbar(inner, command=self.log_text.yview, bg=C["bg2"], troughcolor=C["bg"])
        sb.pack(side="right", fill="y")
        self.log_text.config(yscrollcommand=sb.set)

        self.log_text.tag_config("system", foreground=C["cyan"])
        self.log_text.tag_config("info",   foreground=C["green_dim"])
        self.log_text.tag_config("warn",   foreground=C["amber_dim"])
        self.log_text.tag_config("error",  foreground=C["red"])
        self.log_text.tag_config("proc",   foreground=C["grey"])
        self.log_text.tag_config("start",  foreground=C["green"])
        self.log_text.tag_config("stop",   foreground=C["red_dim"])

    # ── Logging ───────────────────────────────
    def _log(self, msg, tag="info"):
        ts = datetime.now().strftime("%H:%M:%S")
        line = f"[{ts}] {msg}\n"

        # Write to session log file
        try:
            self._session_log.write(line)
        except Exception:
            pass

        def _do():
            self.log_text.config(state="normal")
            self.log_text.insert("end", line, tag)
            line_count = int(self.log_text.index("end-1c").split(".")[0])
            if line_count > 600:
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

        def _do():
            self.pm.kill_all(self._log)
            time.sleep(1)
            for sub in SUBSYSTEMS:
                self.root.after(0, self._refresh_card, sub)
            self.root.after(1100, self._update_control_buttons)

        threading.Thread(target=_do, daemon=True).start()

    # ── Periodic updates ──────────────────────
    def _start_clock(self):
        self._blink = True

        def tick():
            now = datetime.now().strftime("%Y-%m-%d  %H:%M:%S")
            self.clock_var.set(f"  SYS_TIME: {now}   WS: {ROS_SETUP_SCRIPT}")
            self._blink = not self._blink
            self.blink_var.set("█" if self._blink else " ")
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