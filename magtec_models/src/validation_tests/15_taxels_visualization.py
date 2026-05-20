#!/usr/bin/env python3
"""
15 Taxels Real-Time Visualization with GPU Acceleration
 
This script provides a fast, GPU-accelerated visualization of 15 magnetic taxels
arranged in a 3x5 grid. Each taxel displays:
- A circle that moves on XY axis based on Fx/Fy forces
- Circle radius increases based on Fz force
 
Features:
- Optimized rendering with matplotlib blitting for fast updates
- Real-time sensor data reading at 100Hz
- Calibrated sensor readings
- Smooth animations
 
Usage:
    python3 src/validation_tests/15_taxels_visualization.py [--enable-ft-sensor]
 
Author: Gabriele Giudici
Date: 2025
"""
 
import os
import sys
import time
import numpy as np
import tkinter as tk
from tkinter import ttk
 
# Add parent directory to path for imports
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from franka_controller.config import *
 
# Import SensorReader from existing code
# Import directly from the file in the same directory
import importlib.util
spec = importlib.util.spec_from_file_location(
    "points_real_time_predictor",
    os.path.join(os.path.dirname(__file__), "10_points_real_time_predictor.py")
)
points_module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(points_module)
SensorReader = points_module.SensorReader
 
# Use matplotlib with optimized backend for fast rendering
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.patches import Circle
 
# Grid layout: 3 rows × 5 columns = 15 taxels
TAXEL_ROWS = 3
TAXEL_COLS = 5
NUM_TAXELS = TAXEL_ROWS * TAXEL_COLS
 
# Visualization parameters
TAXEL_SIZE = 0.1  # Size of each taxel cell (normalized)
LINEAR_RANGE = 7.0  # Linear range for forces (±7 for Fx/Fy, 0-7 for Fz)
POST_LINEAR_SCALE = 0.1  # Scale factor for values beyond linear range (10% of linear rate)
MIN_RADIUS = 0.001  # Minimum circle radius (normalized) - start as a point
MAX_RADIUS = 0.025  # Maximum circle radius (normalized) - smaller circles
BASE_RADIUS = 0.001  # Base radius when Fz = 0 - start as a point
MAX_MOVEMENT = 0.08  # Maximum movement from center (to prevent overlap, increased for visibility)
 
 
class TaxelsVisualization:
    """Fast visualization of 15 taxels with optimized rendering."""
    
    def __init__(self, root, sensor_reader):
        self.root = root
        self.sensor_reader = sensor_reader
        self.running = False
        
        # Data storage
        self.forces = np.zeros((NUM_TAXELS, 3))  # [Fx, Fy, Fz] for each taxel
        
        # Create GUI
        self.create_gui()
        
        # Start update loop
        self.update_interval = 10  # ms (100Hz)
        self.update_running = False
    
    def create_gui(self):
        """Create the GUI window."""
        self.root.title("15 Taxels Real-Time Visualization")
        self.root.geometry("1200x800")
        
        # Main frame
        main_frame = ttk.Frame(self.root)
        main_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        
        # Control frame
        control_frame = ttk.Frame(main_frame)
        control_frame.pack(fill=tk.X, pady=(0, 10))
        
        self.start_button = ttk.Button(control_frame, text="Start Sensors", command=self.start_sensors)
        self.start_button.pack(side=tk.LEFT, padx=5)
        
        self.stop_button = ttk.Button(control_frame, text="Stop Sensors", command=self.stop_sensors, state=tk.DISABLED)
        self.stop_button.pack(side=tk.LEFT, padx=5)
        
        self.recalibrate_button = ttk.Button(control_frame, text="Fast Recalibrate", command=self.fast_recalibrate, state=tk.DISABLED)
        self.recalibrate_button.pack(side=tk.LEFT, padx=5)
        
        self.status_label = ttk.Label(control_frame, text="Status: Stopped", foreground="red")
        self.status_label.pack(side=tk.LEFT, padx=20)
        
        # Visualization frame
        viz_frame = ttk.Frame(main_frame)
        viz_frame.pack(fill=tk.BOTH, expand=True)
        
        self.create_matplotlib_visualization(viz_frame)
    
    def create_matplotlib_visualization(self, parent):
        """Create matplotlib-based visualization with optimized settings."""
        # Use tight layout and disable unnecessary features for speed
        # Light theme
        self.fig, self.ax = plt.subplots(figsize=(12, 7), facecolor='white')
        self.ax.set_xlim(-0.1, 0.6)
        self.ax.set_ylim(-0.1, 0.4)
        self.ax.set_aspect('equal')
        self.ax.set_title("15 Taxels Real-Time Visualization (3×5 Grid)", fontsize=14, fontweight='bold', color='black')
        self.ax.grid(True, alpha=0.3, linestyle='--', color='#888888')
        self.ax.set_facecolor('white')
        # Remove axis labels and ticks
        self.ax.set_xticks([])
        self.ax.set_yticks([])
        self.ax.set_xlabel('')
        self.ax.set_ylabel('')
        
        # Add X/Y direction indicators (arrows) in top-right corner
        # X direction (right): arrow pointing right
        self.ax.annotate('', xy=(0.55, 0.35), xytext=(0.50, 0.35),
                        arrowprops=dict(arrowstyle='->', lw=2.5, color='red'))
        self.ax.text(0.525, 0.37, 'X', ha='center', va='bottom', fontsize=11,
                    fontweight='bold', color='red')
        
        # Y direction (up): arrow pointing up
        self.ax.annotate('', xy=(0.50, 0.35), xytext=(0.50, 0.30),
                        arrowprops=dict(arrowstyle='->', lw=2.5, color='green'))
        self.ax.text(0.48, 0.325, 'Y', ha='right', va='center', fontsize=11,
                    fontweight='bold', color='green')
        
        # Disable autoscale for better performance
        self.ax.set_autoscale_on(False)
        
        # Create grid positions (3 rows × 5 columns)
        # Grid order: columns from right to left, each column bottom to top
        # Column 4 (rightmost): T1 (bottom), T2 (middle), T3 (top)
        # Column 3: T4 (bottom), T5 (middle), T6 (top)
        # Column 2: T7 (bottom), T8 (middle), T9 (top)
        # Column 1: T10 (bottom), T11 (middle), T12 (top)
        # Column 0 (leftmost): T13 (bottom), T14 (middle), T15 (top)
        
        self.taxel_positions = []
        self.circles = []
        self.force_texts = []  # Text objects for displaying Fx, Fy values
        self.taxel_labels = []  # Static taxel ID labels
        
        # Taxel mapping: taxel_id -> (col, row) in grid
        # Grid has 5 columns (right to left) and 3 rows (bottom to top)
        taxel_to_grid = {
            1: (4, 0),  2: (4, 1),  3: (4, 2),   # Rightmost column
            4: (3, 0),  5: (3, 1),  6: (3, 2),   # Column 3
            7: (2, 0),  8: (2, 1),  9: (2, 2),   # Column 2
            10: (1, 0), 11: (1, 1), 12: (1, 2),  # Column 1
            13: (0, 0), 14: (0, 1), 15: (0, 2),  # Leftmost column
        }
        
        # Create positions in taxel order (1-15)
        for taxel_id in range(1, NUM_TAXELS + 1):
            col, row = taxel_to_grid[taxel_id]
            # x increases from left to right, y increases from bottom to top
            x_center = col * 0.12 + 0.06
            y_center = row * 0.12 + 0.06
            self.taxel_positions.append((x_center, y_center))
            
            # Add static taxel label (will be in background)
            taxel_label = self.ax.text(x_center, y_center, f'T{taxel_id}',
                                      ha='center', va='center', fontsize=8, fontweight='bold',
                                      color='#333333', alpha=0.7)  # Dark gray for light theme
            self.taxel_labels.append(taxel_label)
            
            # Create circle for this taxel (dynamic, starts at center position)
            # No static reference marker - only the dynamic circle
            # Hide initially so it's not in background
            circle = Circle((x_center, y_center), BASE_RADIUS, color='black', alpha=0.3, edgecolor='#333333', linewidth=0.5)
            circle.set_visible(False)  # Hide until first update
            self.ax.add_patch(circle)
            self.circles.append(circle)
            
            # Add force values text (dynamic, will be updated each frame)
            # Hide initially so it's not in background
            force_text = self.ax.text(x_center, y_center - 0.04, '',
                                     ha='center', va='top', fontsize=6, color='black')  # Black text for light theme
            force_text.set_visible(False)  # Hide until first update
            self.force_texts.append(force_text)
        
        self.canvas = FigureCanvasTkAgg(self.fig, parent)
        self.canvas.draw()
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        
        # Enable blitting for faster updates (only redraw changed elements)
        self.canvas.mpl_connect('draw_event', self.on_draw)
        self.background = None
    
    def on_draw(self, event):
        """Store background for blitting (includes only static taxel labels)."""
        # Ensure dynamic elements are hidden before saving background
        for circle in self.circles:
            circle.set_visible(False)
        for force_text in self.force_texts:
            force_text.set_visible(False)
        self.canvas.draw_idle()
        self.canvas.flush_events()
        
        # Save background (only static taxel labels, no circles or force texts)
        self.background = self.canvas.copy_from_bbox(self.ax.bbox)
        
        # Note: Dynamic elements will be made visible in update_matplotlib_visualization
    
    def start_sensors(self):
        """Start sensor reading and visualization updates."""
        try:
            self.sensor_reader.start_sensors()
            self.update_running = True
            self.start_button.config(state=tk.DISABLED)
            self.stop_button.config(state=tk.NORMAL)
            self.recalibrate_button.config(state=tk.NORMAL)
            self.status_label.config(text="Status: Running (Calibrating...)", foreground="orange")
            self.update_visualization()
        except Exception as e:
            print(f"Failed to start sensors: {e}")
    
    def stop_sensors(self):
        """Stop sensor reading and visualization updates."""
        self.update_running = False
        self.sensor_reader.stop_sensors()
        self.start_button.config(state=tk.NORMAL)
        self.stop_button.config(state=tk.DISABLED)
        self.recalibrate_button.config(state=tk.DISABLED)
        self.status_label.config(text="Status: Stopped", foreground="red")
    
    def fast_recalibrate(self):
        """Trigger fast recalibration of StretchMagTec sensors."""
        if not self.sensor_reader.running:
            import tkinter.messagebox as messagebox
            messagebox.showwarning("Warning", "Sensors must be running to recalibrate!")
            return
        
        if self.sensor_reader.fast_recalibration_active:
            import tkinter.messagebox as messagebox
            messagebox.showinfo("Info", "Recalibration already in progress. Please wait...")
            return
        
        # Start fast recalibration
        self.sensor_reader.fast_recalibration_active = True
        self.sensor_reader.fast_recalibration_samples = []
        self.sensor_reader.fast_recalibration_start_time = None
        self.recalibrate_button.config(state=tk.DISABLED, text="Recalibrating...")
        self.status_label.config(text=f"Status: Fast recalibration active ({self.sensor_reader.fast_recalibration_duration}s)", foreground="orange")
        
        # Schedule button re-enable after recalibration duration
        self.root.after(int(self.sensor_reader.fast_recalibration_duration * 1000 + 500),
                       lambda: self.recalibrate_button.config(state=tk.NORMAL, text="Fast Recalibrate"))
    
    def update_visualization(self):
        """Update the visualization with latest sensor data."""
        if not self.update_running:
            return
        
        try:
            # Update status if calibration is complete
            if self.sensor_reader.stretchmagtec_is_calibrated and "Calibrating" in self.status_label.cget("text"):
                self.status_label.config(text="Status: Running", foreground="green")
            
            # Get sensor data (calibrated raw values)
            sensor_data = self.sensor_reader.get_stretchmagtec_data()  # Shape: (15, 3)
            
            # Convert raw magnetic values to display values (simple scaling)
            # Just multiply raw values by 0.001, no filtering or clipping
            scale_factor = 0.001
            
            # Map raw values: X->Fx, Y->Fy, Z->Fz
            self.forces[:, 0] = sensor_data[:, 0] * scale_factor  # Fx (raw * 0.001)
            self.forces[:, 1] = sensor_data[:, 1] * scale_factor  # Fy (raw * 0.001)
            self.forces[:, 2] = np.abs(sensor_data[:, 2]) * scale_factor  # Fz (absolute raw * 0.001)
            
            # Update visualization
            self.update_matplotlib_visualization()
            
        except Exception as e:
            print(f"Visualization update error: {e}")
        
        # Schedule next update
        self.root.after(self.update_interval, self.update_visualization)
    
    def update_matplotlib_visualization(self):
        """Update matplotlib visualization with optimized rendering."""
        # Restore background for blitting (only static taxel labels)
        if self.background is not None:
            self.canvas.restore_region(self.background)
        
        # Update all circles
        for i, (circle, (base_x, base_y)) in enumerate(zip(self.circles, self.taxel_positions)):
            fx, fy, fz = self.forces[i]
            
            # Piecewise mapping for Fx/Fy: linear up to ±LINEAR_RANGE, then much slower
            def map_force_to_movement(force_val):
                abs_force = abs(force_val)
                sign = np.sign(force_val)
                if abs_force <= LINEAR_RANGE:
                    # Linear mapping in range [0, LINEAR_RANGE]
                    normalized = abs_force / LINEAR_RANGE
                else:
                    # Beyond LINEAR_RANGE: linear part + slow additional part
                    normalized = 1.0 + (abs_force - LINEAR_RANGE) * POST_LINEAR_SCALE / LINEAR_RANGE
                return sign * normalized
            
            # Calculate new position (move based on Fx/Fy)
            # Fx moves in +X direction, Fy moves in -Y direction (inverted)
            offset_x = map_force_to_movement(fx) * MAX_MOVEMENT
            offset_y = -map_force_to_movement(fy) * MAX_MOVEMENT  # Inverted Fy direction
            new_x = base_x + offset_x
            new_y = base_y + offset_y
            
            # Clamp position to stay within cell bounds (prevent circles from overlapping)
            # Use tighter bounds to ensure circles never touch
            cell_margin = MAX_RADIUS + 0.005  # Extra margin for safety
            cell_left = base_x - MAX_MOVEMENT + cell_margin
            cell_right = base_x + MAX_MOVEMENT - cell_margin
            cell_bottom = base_y - MAX_MOVEMENT + cell_margin
            cell_top = base_y + MAX_MOVEMENT - cell_margin
            new_x = np.clip(new_x, cell_left, cell_right)
            new_y = np.clip(new_y, cell_bottom, cell_top)
            
            # Calculate new radius (based on Fz value, already scaled)
            # Piecewise mapping: linear up to LINEAR_RANGE, then much slower
            if fz <= LINEAR_RANGE:
                # Linear mapping in range [0, LINEAR_RANGE]
                normalized_fz = fz / LINEAR_RANGE
            else:
                # Beyond LINEAR_RANGE: linear part + slow additional part
                normalized_fz = 1.0 + (fz - LINEAR_RANGE) * POST_LINEAR_SCALE / LINEAR_RANGE
            
            # Map normalized Fz to radius range [BASE_RADIUS, MAX_RADIUS]
            radius = BASE_RADIUS + normalized_fz * (MAX_RADIUS - BASE_RADIUS)
            radius = np.clip(radius, MIN_RADIUS, MAX_RADIUS)
            
            # Update circle (dynamic position - no static reference marker)
            circle.center = (new_x, new_y)
            circle.set_radius(radius)
            
            # Color: black up to Fz=7, then increasing red intensity beyond threshold
            if fz <= LINEAR_RANGE:
                # Black for light theme
                circle.set_color('black')
                circle.set_edgecolor('#333333')  # Dark gray edge
                circle.set_alpha(0.6)
            else:
                # Red intensity increases with Fz beyond threshold
                # Normalize excess Fz to [0, 1] for color intensity
                excess_fz = fz - LINEAR_RANGE
                max_excess = 10.0  # Assume max Fz around 17 for full red
                intensity = np.clip(excess_fz / max_excess, 0.0, 1.0)
                # Red color: intensity from black to bright red
                red_value = int(255 * intensity)
                green_blue = int(255 * (1 - intensity))  # Fade from black to red
                circle.set_color(f'#{red_value:02x}{green_blue:02x}{green_blue:02x}')  # Red intensity
                circle.set_edgecolor('#ff0000' if intensity > 0.5 else '#333333')
                circle.set_alpha(0.7 + 0.3 * intensity)  # More opaque as intensity increases
            
            circle.set_visible(True)  # Make visible for first update
            
            # Update force values text (dynamic, follows circle position)
            force_text = self.force_texts[i]
            force_text.set_position((new_x, new_y - radius - 0.01))
            force_text.set_text(f'Fx:{fx:.2f} Fy:{fy:.2f} Fz:{fz:.2f}')
            # Use black text for light theme
            force_text.set_color('black')
            force_text.set_visible(True)  # Make visible for first update
        
        # Blit only the changed circles and force texts (faster than full redraw)
        if self.background is not None:
            # Restore background (contains static taxel labels)
            self.canvas.restore_region(self.background)
            # Draw dynamic elements (circles and force texts)
            for circle in self.circles:
                self.ax.draw_artist(circle)
            for force_text in self.force_texts:
                self.ax.draw_artist(force_text)
            self.canvas.blit(self.ax.bbox)
        else:
            # Fallback to full redraw if background not set
            self.canvas.draw_idle()
 
 
def main():
    import argparse
    
    parser = argparse.ArgumentParser(
        description="15 Taxels Real-Time Visualization",
        formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument(
        '--enable-ft-sensor',
        action='store_true',
        help='Enable FT sensor connection (not used in this visualization)'
    )
    
    args = parser.parse_args()
    
    # Create GUI
    root = tk.Tk()
    sensor_reader = SensorReader(enable_ft_sensor=args.enable_ft_sensor)
    app = TaxelsVisualization(root, sensor_reader)
    
    # Handle window close
    def on_closing():
        app.stop_sensors()
        root.destroy()
    
    root.protocol("WM_DELETE_WINDOW", on_closing)
    
    # Run GUI
    root.mainloop()
 
 
if __name__ == "__main__":
    main()
 
 