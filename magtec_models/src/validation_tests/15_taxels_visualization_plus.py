#!/usr/bin/env python3
"""
15 Taxels Real-Time Visualization (extended)

Based on the restored 15-taxel visualizer, with:
- Original 3x5 taxel animated view
- Live XYZ plots for all 15 channels in one flow
- Video recording toggle with key "v"
"""

import os
import sys
import time
from collections import deque
from datetime import datetime
from pathlib import Path

import numpy as np
import tkinter as tk
from tkinter import ttk

# Add parent directory to path for imports
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from franka_controller.config import *  # noqa: F403,F401

# Import SensorReader from existing code
import importlib.util

spec = importlib.util.spec_from_file_location(
    "points_real_time_predictor",
    os.path.join(os.path.dirname(__file__), "10_points_real_time_predictor.py"),
)
points_module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(points_module)
SensorReader = points_module.SensorReader

import matplotlib

matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.patches import Circle

# Lazy-loaded recording deps (resolved at runtime on button press).
imageio = None
ImageGrab = None


TAXEL_ROWS = 3
TAXEL_COLS = 5
NUM_TAXELS = TAXEL_ROWS * TAXEL_COLS

TAXEL_SIZE = 0.1
LINEAR_RANGE = 7.0
POST_LINEAR_SCALE = 0.1
MIN_RADIUS = 0.001
MAX_RADIUS = 0.025
BASE_RADIUS = 0.001
MAX_MOVEMENT = 0.08
# FZ expansion scaling:
FZ_EXPANSION_GAIN = 1.0   # bulk magnet (original)
# FZ_EXPANSION_GAIN = 10.0  # soft silicone scaling value

# Hysteresis compensation (visualization-only): if a taxel stays almost unchanged
# while above displayed Fz threshold for this duration, re-zero its local XYZ baseline.
HYSTERESIS_HOLD_SEC = 0.5
HYSTERESIS_DELTA_THR = 120.0  # raw units
HYSTERESIS_FZ_DISPLAY_THR = 1.2  # threshold on displayed Fz value


class TaxelsVisualizationPlus:
    def __init__(self, root, sensor_reader):
        self.root = root
        self.sensor_reader = sensor_reader

        self.forces = np.zeros((NUM_TAXELS, 3))
        self.xyz_baseline = np.zeros((NUM_TAXELS, 3), dtype=float)
        self.xyz_baseline_ready = False
        self._hyst_last_corr = np.zeros((NUM_TAXELS, 3), dtype=float)
        self._hyst_hold_start = np.zeros(NUM_TAXELS, dtype=float)
        self.fz_deadzone = 0.12  # displayed units (after scale_factor)

        self.update_interval = 20  # 50Hz GUI update
        self.update_running = False
        self.update_after_id = None
        self.plot_refresh_divider = 2
        self._frame_count = 0

        # Time-series buffers
        self.max_points = 1000
        self.time_buffer = deque(maxlen=self.max_points)
        self.xyz_buffer = np.zeros((self.max_points, NUM_TAXELS, 3), dtype=float)
        self._buffer_count = 0

        # Video recording
        self.is_recording = False
        self.video_writer = None
        self.record_fps = 20
        self.record_interval_ms = int(1000 / self.record_fps)
        self.record_after_id = None
        self.video_path = None

        # Shared Y scale for all XYZ plots
        self.xyz_y_limit = 2500

        self.create_gui()

    def create_gui(self):
        self.root.title("15 Taxels Visualization Plus")
        self.root.geometry("1550x950")
        self.root.bind("<KeyPress-v>", self.toggle_recording_event)
        self.root.bind("<KeyPress-V>", self.toggle_recording_event)
        self.root.bind("<KeyPress-plus>", self.increase_xyz_scale_event)
        self.root.bind("<KeyPress-KP_Add>", self.increase_xyz_scale_event)
        self.root.bind("<KeyPress-minus>", self.decrease_xyz_scale_event)
        self.root.bind("<KeyPress-KP_Subtract>", self.decrease_xyz_scale_event)
        self.root.bind("<KeyPress-bracketright>", self.increase_xyz_scale_small_event)  # ]
        self.root.bind("<KeyPress-bracketleft>", self.decrease_xyz_scale_small_event)  # [

        main_frame = ttk.Frame(self.root)
        main_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)

        control_frame = ttk.Frame(main_frame)
        control_frame.pack(fill=tk.X, pady=(0, 10))

        self.start_button = ttk.Button(control_frame, text="Start Sensors", command=self.start_sensors)
        self.start_button.pack(side=tk.LEFT, padx=5)

        self.stop_button = ttk.Button(control_frame, text="Stop Sensors", command=self.stop_sensors, state=tk.DISABLED)
        self.stop_button.pack(side=tk.LEFT, padx=5)

        self.recalibrate_button = ttk.Button(
            control_frame, text="Fast Recalibrate", command=self.fast_recalibrate, state=tk.DISABLED
        )
        self.recalibrate_button.pack(side=tk.LEFT, padx=5)

        self.record_button = ttk.Button(control_frame, text="Start Recording (V)", command=self.toggle_recording)
        self.record_button.pack(side=tk.LEFT, padx=5)

        self.scale_up_button = ttk.Button(control_frame, text="XYZ +10000 (+)", command=self.increase_xyz_scale)
        self.scale_up_button.pack(side=tk.LEFT, padx=5)

        self.scale_down_button = ttk.Button(control_frame, text="XYZ -10000 (-)", command=self.decrease_xyz_scale)
        self.scale_down_button.pack(side=tk.LEFT, padx=5)

        self.scale_up_small_button = ttk.Button(control_frame, text="XYZ +500 (])", command=self.increase_xyz_scale_small)
        self.scale_up_small_button.pack(side=tk.LEFT, padx=5)

        self.scale_down_small_button = ttk.Button(control_frame, text="XYZ -500 ([)", command=self.decrease_xyz_scale_small)
        self.scale_down_small_button.pack(side=tk.LEFT, padx=5)

        self.status_label = ttk.Label(control_frame, text="Status: Stopped", foreground="red")
        self.status_label.pack(side=tk.LEFT, padx=20)

        self.record_label = ttk.Label(control_frame, text="REC: OFF", foreground="gray")
        self.record_label.pack(side=tk.LEFT, padx=10)

        self.scale_label = ttk.Label(control_frame, text=f"XYZ Y-lim: +/-{self.xyz_y_limit}", foreground="blue")
        self.scale_label.pack(side=tk.LEFT, padx=10)

        viz_frame = ttk.Frame(main_frame)
        viz_frame.pack(fill=tk.BOTH, expand=True)

        # Left: taxel animation
        left = ttk.Frame(viz_frame)
        left.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(0, 8))

        # Right: xyz plots
        right = ttk.Frame(viz_frame)
        right.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(8, 0))

        self.create_taxel_view(left)
        self.create_xyz_plots(right)

    def create_taxel_view(self, parent):
        self.fig, self.ax = plt.subplots(figsize=(9, 7), facecolor="white")
        self.ax.set_xlim(-0.1, 0.6)
        self.ax.set_ylim(-0.1, 0.4)
        self.ax.set_aspect("equal")
        self.ax.set_title("15 Taxels Real-Time Visualization (3x5 Grid)", fontsize=13, fontweight="bold", color="black")
        self.ax.grid(True, alpha=0.3, linestyle="--", color="#888888")
        self.ax.set_facecolor("white")
        self.ax.set_xticks([])
        self.ax.set_yticks([])

        self.ax.annotate("", xy=(0.55, 0.35), xytext=(0.50, 0.35), arrowprops=dict(arrowstyle="->", lw=2.5, color="red"))
        self.ax.text(0.525, 0.37, "X", ha="center", va="bottom", fontsize=11, fontweight="bold", color="red")
        self.ax.annotate("", xy=(0.50, 0.35), xytext=(0.50, 0.30), arrowprops=dict(arrowstyle="->", lw=2.5, color="green"))
        self.ax.text(0.48, 0.325, "Y", ha="right", va="center", fontsize=11, fontweight="bold", color="green")
        self.ax.set_autoscale_on(False)

        self.taxel_positions = []
        self.circles = []
        self.taxel_labels = []
        self.force_texts = []

        taxel_to_grid = {
            1: (4, 0),
            2: (4, 1),
            3: (4, 2),
            4: (3, 0),
            5: (3, 1),
            6: (3, 2),
            7: (2, 0),
            8: (2, 1),
            9: (2, 2),
            10: (1, 0),
            11: (1, 1),
            12: (1, 2),
            13: (0, 0),
            14: (0, 1),
            15: (0, 2),
        }

        for taxel_id in range(1, NUM_TAXELS + 1):
            col, row = taxel_to_grid[taxel_id]
            x_center = col * 0.12 + 0.06
            y_center = row * 0.12 + 0.06
            self.taxel_positions.append((x_center, y_center))

            label = self.ax.text(
                x_center, y_center, f"T{taxel_id}", ha="center", va="center", fontsize=8, fontweight="bold", color="#333333", alpha=0.7
            )
            self.taxel_labels.append(label)

            # Dynamic force text (same behavior as base visualizer)
            force_text = self.ax.text(x_center, y_center - 0.04, "", ha="center", va="top", fontsize=6, color="black")
            force_text.set_visible(False)
            self.force_texts.append(force_text)

            circle = Circle((x_center, y_center), BASE_RADIUS, color="black", alpha=0.3, edgecolor="#333333", linewidth=0.5)
            circle.set_visible(False)
            self.ax.add_patch(circle)
            self.circles.append(circle)

        self.canvas = FigureCanvasTkAgg(self.fig, parent)
        self.canvas.draw()
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        self.canvas.mpl_connect("draw_event", self.on_draw)
        self.background = None

    def create_xyz_plots(self, parent):
        self.fig_xyz, self.axes_xyz = plt.subplots(3, 1, figsize=(8, 7), sharex=True, facecolor="white")
        labels = ["X (all 15 channels)", "Y (all 15 channels)", "Z (all 15 channels)"]
        colors = plt.cm.tab20(np.linspace(0, 1, NUM_TAXELS))

        self.xyz_lines = [[], [], []]
        for comp_idx, ax in enumerate(self.axes_xyz):
            for ch in range(NUM_TAXELS):
                (line,) = ax.plot([], [], color=colors[ch], lw=0.9, alpha=0.9)
                self.xyz_lines[comp_idx].append(line)
            ax.set_ylabel(labels[comp_idx], fontsize=9)
            ax.grid(True, alpha=0.25)
            ax.set_ylim(-self.xyz_y_limit, self.xyz_y_limit)
        self.axes_xyz[-1].set_xlabel("Time (s)")
        self.fig_xyz.tight_layout()

        self.canvas_xyz = FigureCanvasTkAgg(self.fig_xyz, parent)
        self.canvas_xyz.draw()
        self.canvas_xyz.get_tk_widget().pack(fill=tk.BOTH, expand=True)

    def on_draw(self, _event):
        # Match base implementation: hide dynamic artists, snapshot only static background.
        for circle in self.circles:
            circle.set_visible(False)
        for force_text in self.force_texts:
            force_text.set_visible(False)
        self.canvas.draw_idle()
        self.canvas.flush_events()
        self.background = self.canvas.copy_from_bbox(self.ax.bbox)

    def start_sensors(self):
        try:
            self.sensor_reader.start_sensors()
            self.xyz_baseline[:, :] = 0.0
            self.xyz_baseline_ready = False
            self.update_running = True
            self.start_button.config(state=tk.DISABLED)
            self.stop_button.config(state=tk.NORMAL)
            self.recalibrate_button.config(state=tk.NORMAL)
            self.status_label.config(text="Status: Running (Calibrating...)", foreground="orange")
            self.update_visualization()
        except Exception as e:
            print(f"Failed to start sensors: {e}")

    def stop_sensors(self):
        self.update_running = False
        if self.update_after_id is not None:
            try:
                self.root.after_cancel(self.update_after_id)
            except Exception:
                pass
            self.update_after_id = None
        self.sensor_reader.stop_sensors()
        self.start_button.config(state=tk.NORMAL)
        self.stop_button.config(state=tk.DISABLED)
        self.recalibrate_button.config(state=tk.DISABLED)
        self.status_label.config(text="Status: Stopped", foreground="red")

    def fast_recalibrate(self):
        if not self.sensor_reader.running:
            import tkinter.messagebox as messagebox

            messagebox.showwarning("Warning", "Sensors must be running to recalibrate!")
            return
        if self.sensor_reader.fast_recalibration_active:
            import tkinter.messagebox as messagebox

            messagebox.showinfo("Info", "Recalibration already in progress. Please wait...")
            return

        self.sensor_reader.fast_recalibration_active = True
        self.sensor_reader.fast_recalibration_samples = []
        self.sensor_reader.fast_recalibration_start_time = None
        self.xyz_baseline_ready = False
        self.xyz_baseline[:, :] = 0.0
        self.recalibrate_button.config(state=tk.DISABLED, text="Recalibrating...")
        self.status_label.config(
            text=f"Status: Fast recalibration active ({self.sensor_reader.fast_recalibration_duration}s)", foreground="orange"
        )
        self.root.after(
            int(self.sensor_reader.fast_recalibration_duration * 1000 + 500),
            lambda: self.recalibrate_button.config(state=tk.NORMAL, text="Fast Recalibrate"),
        )

    def update_visualization(self):
        if not self.update_running:
            return
        try:
            if self.sensor_reader.stretchmagtec_is_calibrated and "Calibrating" in self.status_label.cget("text"):
                self.status_label.config(text="Status: Running", foreground="green")

            sensor_data = self.sensor_reader.get_stretchmagtec_data()  # [15, 3]
            if self.sensor_reader.stretchmagtec_is_calibrated and not self.xyz_baseline_ready:
                # Capture one baseline snapshot right after calibration.
                self.xyz_baseline[:, :] = sensor_data
                self.xyz_baseline_ready = True

            if self.xyz_baseline_ready:
                sensor_data_corr = sensor_data - self.xyz_baseline
            else:
                sensor_data_corr = sensor_data

            # Hysteresis compensation: if displayed Fz stays above threshold and channel
            # is nearly not changing for >HYSTERESIS_HOLD_SEC, absorb residual offset.
            now = time.time()
            for i in range(NUM_TAXELS):
                delta = np.max(np.abs(sensor_data_corr[i] - self._hyst_last_corr[i]))
                z_vis = abs(sensor_data_corr[i, 2]) * 0.001 * FZ_EXPANSION_GAIN
                if z_vis > HYSTERESIS_FZ_DISPLAY_THR and delta <= HYSTERESIS_DELTA_THR:
                    if self._hyst_hold_start[i] == 0.0:
                        self._hyst_hold_start[i] = now
                    elif now - self._hyst_hold_start[i] >= HYSTERESIS_HOLD_SEC:
                        self.xyz_baseline[i] += sensor_data_corr[i]
                        sensor_data_corr[i] = 0.0
                        self._hyst_hold_start[i] = 0.0
                else:
                    self._hyst_hold_start[i] = 0.0
                self._hyst_last_corr[i] = sensor_data_corr[i]

            scale_factor = 0.001
            self.forces[:, 0] = sensor_data_corr[:, 0] * scale_factor
            self.forces[:, 1] = sensor_data_corr[:, 1] * scale_factor
            fz_comp = np.abs(sensor_data_corr[:, 2]) * scale_factor - self.fz_deadzone
            self.forces[:, 2] = np.maximum(0.0, fz_comp)

            # Buffer baseline-compensated values for channel plots.
            self._append_buffer(now, sensor_data_corr)

            self.update_taxel_view()
            self._frame_count += 1
            if self._frame_count % self.plot_refresh_divider == 0:
                self.update_xyz_view()
        except Exception as e:
            print(f"Visualization update error: {e}")

        if self.update_running and self.root.winfo_exists():
            self.update_after_id = self.root.after(self.update_interval, self.update_visualization)

    def _append_buffer(self, timestamp, sensor_data):
        self.time_buffer.append(timestamp)
        if self._buffer_count < self.max_points:
            self.xyz_buffer[self._buffer_count, :, :] = sensor_data
            self._buffer_count += 1
        else:
            self.xyz_buffer[:-1, :, :] = self.xyz_buffer[1:, :, :]
            self.xyz_buffer[-1, :, :] = sensor_data

    def update_taxel_view(self):
        def map_force_to_movement(force_val):
            abs_force = abs(force_val)
            sign = np.sign(force_val)
            if abs_force <= LINEAR_RANGE:
                normalized = abs_force / LINEAR_RANGE
            else:
                normalized = 1.0 + (abs_force - LINEAR_RANGE) * POST_LINEAR_SCALE / LINEAR_RANGE
            return sign * normalized

        for i, (circle, (base_x, base_y)) in enumerate(zip(self.circles, self.taxel_positions)):
            fx, fy, fz = self.forces[i]
            offset_x = map_force_to_movement(fx) * MAX_MOVEMENT
            offset_y = -map_force_to_movement(fy) * MAX_MOVEMENT
            new_x = base_x + offset_x
            new_y = base_y + offset_y

            cell_margin = MAX_RADIUS + 0.005
            cell_left = base_x - MAX_MOVEMENT + cell_margin
            cell_right = base_x + MAX_MOVEMENT - cell_margin
            cell_bottom = base_y - MAX_MOVEMENT + cell_margin
            cell_top = base_y + MAX_MOVEMENT - cell_margin
            new_x = np.clip(new_x, cell_left, cell_right)
            new_y = np.clip(new_y, cell_bottom, cell_top)

            visual_fz = fz * FZ_EXPANSION_GAIN
            if visual_fz <= LINEAR_RANGE:
                normalized_fz = visual_fz / LINEAR_RANGE
            else:
                normalized_fz = 1.0 + (visual_fz - LINEAR_RANGE) * POST_LINEAR_SCALE / LINEAR_RANGE
            radius = BASE_RADIUS + normalized_fz * (MAX_RADIUS - BASE_RADIUS)
            radius = np.clip(radius, MIN_RADIUS, MAX_RADIUS)

            circle.center = (new_x, new_y)
            circle.set_radius(radius)

            if visual_fz <= LINEAR_RANGE:
                circle.set_color("black")
                circle.set_edgecolor("#333333")
                circle.set_alpha(0.6)
            else:
                excess_fz = visual_fz - LINEAR_RANGE
                intensity = np.clip(excess_fz / 10.0, 0.0, 1.0)
                red_value = int(255 * intensity)
                green_blue = int(255 * (1 - intensity))
                circle.set_color(f"#{red_value:02x}{green_blue:02x}{green_blue:02x}")
                circle.set_edgecolor("#ff0000" if intensity > 0.5 else "#333333")
                circle.set_alpha(0.7 + 0.3 * intensity)
            circle.set_visible(True)

            # Update force text exactly once per taxel and keep it readable.
            force_text = self.force_texts[i]
            # Keep force text anchored to static taxel cell to avoid visual ghosting.
            force_text.set_position((base_x, base_y - 0.038))
            force_text.set_text(f"Fx:{fx:.2f} Fy:{fy:.2f} Fz:{fz:.2f}")
            force_text.set_color("black")
            force_text.set_visible(True)

        if self.background is not None:
            # Same blitting approach as base visualizer.
            self.canvas.restore_region(self.background)
            for circle in self.circles:
                self.ax.draw_artist(circle)
            for force_text in self.force_texts:
                self.ax.draw_artist(force_text)
            self.canvas.blit(self.ax.bbox)
        else:
            self.canvas.draw()

    def update_xyz_view(self):
        if self._buffer_count < 2:
            return
        times = np.array(self.time_buffer, dtype=float)
        t0 = times[0]
        t_rel = times - t0
        data = self.xyz_buffer[: self._buffer_count, :, :]

        for comp_idx in range(3):
            ax = self.axes_xyz[comp_idx]
            for ch in range(NUM_TAXELS):
                self.xyz_lines[comp_idx][ch].set_data(t_rel, data[:, ch, comp_idx])
            ax.set_ylim(-self.xyz_y_limit, self.xyz_y_limit)
            ax.set_xlim(t_rel[0], t_rel[-1] if t_rel[-1] > t_rel[0] else t_rel[0] + 1e-3)

        self.canvas_xyz.draw_idle()

    def _refresh_xyz_scale_label(self):
        self.scale_label.config(text=f"XYZ Y-lim: +/-{self.xyz_y_limit}")

    def increase_xyz_scale_event(self, _event):
        self.increase_xyz_scale()

    def decrease_xyz_scale_event(self, _event):
        self.decrease_xyz_scale()

    def increase_xyz_scale_small_event(self, _event):
        self.increase_xyz_scale_small()

    def decrease_xyz_scale_small_event(self, _event):
        self.decrease_xyz_scale_small()

    def increase_xyz_scale(self):
        self.xyz_y_limit += 10000
        self._refresh_xyz_scale_label()

    def decrease_xyz_scale(self):
        self.xyz_y_limit = max(10000, self.xyz_y_limit - 10000)
        self._refresh_xyz_scale_label()

    def increase_xyz_scale_small(self):
        self.xyz_y_limit += 500
        self._refresh_xyz_scale_label()

    def decrease_xyz_scale_small(self):
        self.xyz_y_limit = max(500, self.xyz_y_limit - 500)
        self._refresh_xyz_scale_label()

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
            # Quick writer smoke-check to ensure ffmpeg backend exists.
            _ = imageio.get_writer("/tmp/.video_rec_probe.mp4", fps=5, macro_block_size=1)
            _.close()
            try:
                os.remove("/tmp/.video_rec_probe.mp4")
            except OSError:
                pass
            return True
        except Exception as e:
            self.status_label.config(text=f"Status: REC backend error ({e})", foreground="red")
            return False

    def start_recording(self):
        # Save recordings in project-level plots folder, independent from launch cwd.
        project_root = Path(__file__).resolve().parents[2]
        out_dir = project_root / "plots" / "gui_recordings"
        out_dir.mkdir(parents=True, exist_ok=True)
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.video_path = out_dir / f"taxels_plus_{stamp}.mp4"
        self.video_writer = imageio.get_writer(str(self.video_path), fps=self.record_fps, macro_block_size=1)
        self.is_recording = True
        self.record_label.config(text="REC: ON", foreground="red")
        self.record_button.config(text="Stop Recording (V)")
        self.status_label.config(text=f"Status: Recording -> {self.video_path}", foreground="green")
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
            print(f"[REC] frame capture error: {e}")
        if self.is_recording and self.root.winfo_exists():
            self.record_after_id = self.root.after(self.record_interval_ms, self._capture_video_frame)


def main():
    import argparse

    parser = argparse.ArgumentParser(description="15 Taxels Real-Time Visualization Plus")
    parser.add_argument(
        "--enable-ft-sensor",
        action="store_true",
        help="Enable FT sensor connection (not used in this visualization)",
    )
    args = parser.parse_args()

    root = tk.Tk()
    sensor_reader = SensorReader(enable_ft_sensor=args.enable_ft_sensor)
    app = TaxelsVisualizationPlus(root, sensor_reader)

    def on_closing():
        if app.is_recording:
            app.stop_recording()
        app.stop_sensors()
        root.destroy()

    root.protocol("WM_DELETE_WINDOW", on_closing)
    try:
        root.mainloop()
    except KeyboardInterrupt:
        on_closing()


if __name__ == "__main__":
    main()
