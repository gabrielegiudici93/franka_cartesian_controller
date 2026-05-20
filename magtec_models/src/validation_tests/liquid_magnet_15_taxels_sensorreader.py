#!/usr/bin/env python3
"""
Hybrid 15-taxel visualizer:
- GUI/rendering style from liquid_manget_15_taxels.py
- Sensor connection from SensorReader (same stack as your working visualizers)
"""

import os
import sys
import time
from datetime import datetime
from pathlib import Path
import numpy as np
import tkinter as tk
from tkinter import ttk, messagebox
import matplotlib

matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.patches import Circle

# Import SensorReader from your working visualization stack
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import importlib.util

spec = importlib.util.spec_from_file_location(
    "points_real_time_predictor",
    os.path.join(os.path.dirname(__file__), "10_points_real_time_predictor.py"),
)
points_module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(points_module)
SensorReader = points_module.SensorReader

# Lazy-loaded recording deps
imageio = None
ImageGrab = None

NUM_TAXELS = 15
RENDER_FPS = 30
FORCE_RANGE = 1200.0
DEADZONE = 25.0
MAX_VIZ_MOVE = 0.055


class SensorReaderAdapter:
    """Adapts SensorReader to liquid GUI expectations."""

    def __init__(self):
        self.sensor_reader = SensorReader(enable_ft_sensor=False)
        self.current_data = np.zeros((NUM_TAXELS, 3), dtype=float)
        self.is_calibrating = False
        self.running = False

    def start(self):
        self.sensor_reader.start_sensors()
        self.running = True
        self.is_calibrating = True
        return True

    def stop(self):
        self.running = False
        self.sensor_reader.stop_sensors()

    def trigger_calibration(self):
        self.sensor_reader.fast_recalibration_active = True
        self.sensor_reader.fast_recalibration_samples = []
        self.sensor_reader.fast_recalibration_start_time = None
        self.is_calibrating = True

    def poll(self):
        self.current_data = self.sensor_reader.get_stretchmagtec_data()
        if self.sensor_reader.stretchmagtec_is_calibrated and not self.sensor_reader.fast_recalibration_active:
            self.is_calibrating = False


class TaxelVisualizer:
    def __init__(self, root, reader):
        self.root = root
        self.reader = reader
        self.root.bind("<KeyPress-v>", self.toggle_recording_event)
        self.root.bind("<KeyPress-V>", self.toggle_recording_event)

        self.fig, self.ax = plt.subplots(figsize=(10, 7), facecolor="#f8f9fa")
        self.ax.set_xlim(-0.05, 0.65)
        self.ax.set_ylim(-0.05, 0.45)
        self.ax.set_aspect("equal")
        self.ax.set_xticks([])
        self.ax.set_yticks([])

        self.circles = []
        self.base_positions = []
        self.arrows = []

        layout = {
            2: (4, 0), 1: (4, 1), 3: (4, 2), 5: (3, 0), 4: (3, 1), 6: (3, 2),
            8: (2, 0), 7: (2, 1), 9: (2, 2), 11: (1, 0), 10: (1, 1), 12: (1, 2),
            13: (0, 0), 14: (0, 1), 15: (0, 2),
        }

        for i in range(1, 16):
            c_idx, r_idx = layout[i]
            x, y = c_idx * 0.13 + 0.06, r_idx * 0.13 + 0.06
            self.base_positions.append((x, y))
            self.ax.plot([x - 0.01, x + 0.01], [y, y], color="#dddddd", lw=1, zorder=1)
            self.ax.plot([x, x], [y - 0.01, y + 0.01], color="#dddddd", lw=1, zorder=1)
            self.ax.text(x, y + 0.02, f"T{i}", color="#bbbbbb", ha="center", fontsize=8, zorder=0)
            arrow = self.ax.arrow(x, y, 0, 0, width=0.002, head_width=0.008, color="#007bff", zorder=3)
            self.arrows.append(arrow)
            circle = Circle((x, y), 0.01, color="gray", alpha=0.5, zorder=2)
            self.ax.add_patch(circle)
            self.circles.append(circle)

        self.canvas = FigureCanvasTkAgg(self.fig, master=root)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

        ctrl = ttk.Frame(root)
        ctrl.pack(fill=tk.X, padx=10, pady=5)
        ttk.Button(ctrl, text="Calibration", command=self.do_calibrate).pack(side=tk.LEFT)
        self.record_button = ttk.Button(ctrl, text="Start Recording (V)", command=self.toggle_recording)
        self.record_button.pack(side=tk.LEFT, padx=8)
        self.status_label = ttk.Label(ctrl, text="", font=("Arial", 10, "bold"))
        self.status_label.pack(side=tk.LEFT, padx=20)
        self.record_label = ttk.Label(ctrl, text="REC: OFF", font=("Arial", 10, "bold"), foreground="gray")
        self.record_label.pack(side=tk.LEFT, padx=8)

        self.is_recording = False
        self.video_writer = None
        self.record_fps = 20
        self.record_interval_ms = int(1000 / self.record_fps)
        self.record_after_id = None
        self.video_path = None

    def do_calibrate(self):
        self.reader.trigger_calibration()
        self.status_label.config(text="Calibrating...", foreground="#d9534f")

    def toggle_recording_event(self, _event):
        self.toggle_recording()

    def toggle_recording(self):
        if not self._ensure_recording_backend():
            return
        if not self.is_recording:
            self.start_recording()
        else:
            self.stop_recording()

    def _ensure_recording_backend(self):
        global imageio, ImageGrab
        try:
            if imageio is None:
                import imageio.v2 as _imageio

                imageio = _imageio
            if ImageGrab is None:
                from PIL import ImageGrab as _ImageGrab

                ImageGrab = _ImageGrab
            probe = imageio.get_writer("/tmp/.cursor_rec_probe.mp4", fps=5, macro_block_size=1)
            probe.close()
            try:
                os.remove("/tmp/.cursor_rec_probe.mp4")
            except OSError:
                pass
            return True
        except Exception as e:
            self.status_label.config(text=f"REC backend error: {e}", foreground="red")
            return False

    def start_recording(self):
        project_root = Path(__file__).resolve().parents[2]
        out_dir = project_root / "plots" / "gui_recordings"
        out_dir.mkdir(parents=True, exist_ok=True)
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.video_path = out_dir / f"liquid_magnet_{stamp}.mp4"
        self.video_writer = imageio.get_writer(str(self.video_path), fps=self.record_fps, macro_block_size=1)
        self.is_recording = True
        self.record_label.config(text="REC: ON", foreground="red")
        self.record_button.config(text="Stop Recording (V)")
        self.status_label.config(text=f"Recording -> {self.video_path}", foreground="green")
        self._capture_video_frame()

    def stop_recording(self):
        self.is_recording = False
        if self.record_after_id is not None:
            try:
                self.root.after_cancel(self.record_after_id)
            except Exception:
                pass
            self.record_after_id = None
        if self.video_writer is not None:
            self.video_writer.close()
            self.video_writer = None
        self.record_label.config(text=f"REC: OFF ({self.video_path})", foreground="gray")
        self.record_button.config(text="Start Recording (V)")

    def _capture_video_frame(self):
        if not self.is_recording or self.video_writer is None:
            return
        try:
            self.root.update_idletasks()
            x = self.root.winfo_rootx()
            y = self.root.winfo_rooty()
            w = self.root.winfo_width()
            h = self.root.winfo_height()
            frame = ImageGrab.grab(bbox=(x, y, x + w, y + h))
            self.video_writer.append_data(np.array(frame))
        except Exception as e:
            self.status_label.config(text=f"REC frame error: {e}", foreground="red")
        if self.is_recording and self.root.winfo_exists():
            self.record_after_id = self.root.after(self.record_interval_ms, self._capture_video_frame)

    def animate(self):
        self.reader.poll()
        if not self.reader.is_calibrating and self.status_label.cget("text") == "Calibrating...":
            self.status_label.config(text="")

        data = self.reader.current_data
        for i in range(15):
            fx, fy, fz = data[i]
            bx, by = self.base_positions[i]
            f_resultant = np.sqrt(fx ** 2 + fy ** 2)
            if f_resultant < DEADZONE:
                fx, fy, fz, f_resultant = 0, 0, 0, 0
            dx = -1 * np.sign(fx) * (abs(fx / FORCE_RANGE) ** 0.7) * MAX_VIZ_MOVE
            dy = -1 * np.sign(fy) * (abs(fy / FORCE_RANGE) ** 0.7) * MAX_VIZ_MOVE

            self.arrows[i].remove()
            if abs(fx) > 0.5 or abs(fy) > 0.5:
                self.arrows[i] = self.ax.arrow(
                    bx, by, dx, dy, width=0.002, head_width=0.009,
                    color="#007bff", length_includes_head=True, zorder=3
                )
            else:
                self.arrows[i] = self.ax.arrow(bx, by, 0, 0, width=0, head_width=0)

            self.circles[i].center = (bx + dx, by + dy)
            radius = 0.01 + (f_resultant / FORCE_RANGE) * 0.035
            self.circles[i].set_radius(np.clip(radius, 0.003, 0.045))
            if f_resultant > (FORCE_RANGE * 0.25):
                self.circles[i].set_color("#ff4444")
            elif f_resultant > 1.0:
                self.circles[i].set_color("#555555")
            else:
                self.circles[i].set_color("#aaaaaa")

        self.canvas.draw_idle()
        self.root.after(int(1000 / RENDER_FPS), self.animate)


def main():
    root = tk.Tk()
    root.title("Liquid Magnet 15 Taxels (SensorReader Connection)")
    reader = SensorReaderAdapter()
    gui = TaxelVisualizer(root, reader)

    if reader.start():
        gui.do_calibrate()
        gui.animate()

        def on_close():
            if gui.is_recording:
                gui.stop_recording()
            reader.stop()
            root.destroy()

        root.protocol("WM_DELETE_WINDOW", on_close)
        root.mainloop()
    else:
        messagebox.showerror("Error", "Could not start SensorReader connection.")


if __name__ == "__main__":
    main()
