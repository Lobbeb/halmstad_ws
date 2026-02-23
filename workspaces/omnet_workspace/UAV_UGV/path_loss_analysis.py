#!/usr/bin/env python3
"""
Path Loss Analysis and Export Tool
Converts communication log data into CSV and optional plots.
"""

import sys
import math
import re
import csv
from pathlib import Path
import argparse


def _write_svg_line_plot(output_file, xs, ys, title, x_label, y_label, line_color="#1f77b4"):
    if not xs or not ys or len(xs) != len(ys):
        return False

    width = 1200
    height = 700
    margin_left = 90
    margin_right = 30
    margin_top = 60
    margin_bottom = 90

    plot_w = width - margin_left - margin_right
    plot_h = height - margin_top - margin_bottom

    x_min, x_max = min(xs), max(xs)
    y_min, y_max = min(ys), max(ys)
    if x_max == x_min:
        x_max = x_min + 1.0
    if y_max == y_min:
        y_max = y_min + 1.0

    def map_x(x):
        return margin_left + (x - x_min) / (x_max - x_min) * plot_w

    def map_y(y):
        return margin_top + (1.0 - (y - y_min) / (y_max - y_min)) * plot_h

    points = " ".join(f"{map_x(x):.2f},{map_y(y):.2f}" for x, y in zip(xs, ys))

    grid_lines = []
    ticks = 10
    for i in range(ticks + 1):
        gx = margin_left + i * plot_w / ticks
        gy = margin_top + i * plot_h / ticks
        xv = x_min + i * (x_max - x_min) / ticks
        yv = y_max - i * (y_max - y_min) / ticks
        grid_lines.append(f'<line x1="{gx:.2f}" y1="{margin_top}" x2="{gx:.2f}" y2="{margin_top + plot_h}" stroke="#e6e6e6" stroke-width="1"/>')
        grid_lines.append(f'<line x1="{margin_left}" y1="{gy:.2f}" x2="{margin_left + plot_w}" y2="{gy:.2f}" stroke="#e6e6e6" stroke-width="1"/>')
        grid_lines.append(f'<text x="{gx:.2f}" y="{margin_top + plot_h + 24}" text-anchor="middle" font-size="12" fill="#333">{xv:.2f}</text>')
        grid_lines.append(f'<text x="{margin_left - 12}" y="{gy + 4:.2f}" text-anchor="end" font-size="12" fill="#333">{yv:.2f}</text>')

    svg = f'''<?xml version="1.0" encoding="UTF-8"?>
<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">
  <rect x="0" y="0" width="{width}" height="{height}" fill="white"/>
  <text x="{width/2:.1f}" y="34" text-anchor="middle" font-size="24" fill="#111">{title}</text>
  {"".join(grid_lines)}
  <line x1="{margin_left}" y1="{margin_top + plot_h}" x2="{margin_left + plot_w}" y2="{margin_top + plot_h}" stroke="#222" stroke-width="2"/>
  <line x1="{margin_left}" y1="{margin_top}" x2="{margin_left}" y2="{margin_top + plot_h}" stroke="#222" stroke-width="2"/>
  <polyline points="{points}" fill="none" stroke="{line_color}" stroke-width="2.5"/>
  <text x="{margin_left + plot_w/2:.1f}" y="{height - 35}" text-anchor="middle" font-size="16" fill="#111">{x_label}</text>
  <text x="28" y="{margin_top + plot_h/2:.1f}" transform="rotate(-90 28,{margin_top + plot_h/2:.1f})" text-anchor="middle" font-size="16" fill="#111">{y_label}</text>
</svg>
'''

    Path(output_file).write_text(svg, encoding="utf-8")
    return True


def create_svg_plots(csv_file="path_loss_data.csv", output_dir=".", output_base="plot"):
    times = []
    path_losses = []
    distances = []

    with open(csv_file, "r", newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            try:
                times.append(float(row["time"]))
                path_losses.append(float(row["path_loss"]))
                distances.append(float(row["distance"]))
            except Exception:
                continue

    if not times:
        print("Error creating SVG plots: no rows in CSV.")
        return False

    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    path_loss_svg = output_dir / f"{output_base}_path_loss.svg"
    distance_svg = output_dir / f"{output_base}_distance.svg"

    ok1 = _write_svg_line_plot(
        path_loss_svg,
        times,
        path_losses,
        "Path Loss Over Time",
        "Time (s)",
        "Path Loss (dB)",
        "#0057b8",
    )
    ok2 = _write_svg_line_plot(
        distance_svg,
        times,
        distances,
        "Distance Over Time",
        "Time (s)",
        "Distance (m)",
        "#2f9e44",
    )
    if ok1:
        print(f"✓ Saved path loss SVG: {path_loss_svg}")
    if ok2:
        print(f"✓ Saved distance SVG: {distance_svg}")
    return ok1 and ok2

def parse_position(pos_str):
    """Parse position string like '(x, y, z)'."""
    match = re.search(r'\(([-\d.]+),\s*([-\d.]+),\s*([-\d.]+)\)', pos_str)
    if match:
        return (float(match.group(1)), float(match.group(2)), float(match.group(3)))
    return None

def calculate_distance(pos1, pos2):
    """Calculate 3D Euclidean distance between two positions."""
    if not pos1 or not pos2:
        return None
    return math.sqrt((pos1[0]-pos2[0])**2 + (pos1[1]-pos2[1])**2 + (pos1[2]-pos2[2])**2)

def free_space_path_loss(distance_m, frequency_ghz=2.4):
    """Calculate Free Space Path Loss in dB."""
    if distance_m <= 0:
        return 0
    frequency_hz = frequency_ghz * 1e9
    path_loss = 20 * math.log10(distance_m) + 20 * math.log10(frequency_hz) + 20 * math.log10(4 * math.pi / 3e8)
    return path_loss

def extract_time(time_str):
    """Extract time value from time string."""
    try:
        return float(time_str)
    except:
        return 0

def export_path_loss_csv(log_file, output_file="path_loss_data.csv"):
    """Parse communication log and export path loss data to CSV."""
    print(f"\nExporting path loss data from: {log_file}")
    print(f"Output file: {output_file}\n")
    
    data_rows = []
    uav_last_pos = None
    ugv_last_pos = None
    reception_count = 0
    
    try:
        with open(log_file, 'r') as f:
            for line_num, line in enumerate(f, 1):
                line = line.strip()
                if not line or not line.startswith('R '):
                    continue
                
                try:
                    # Parse: R receiver seq msg_type msg_id S start_time start_pos -> E end_time end_pos
                    parts = line.split()
                    receiver = parts[1]
                    
                    # Extract positions
                    s_idx = line.find('S ')
                    if s_idx == -1:
                        continue
                    
                    arrow_idx = line.find(' -> E ')
                    if arrow_idx == -1:
                        continue
                    
                    start_part = line[s_idx+2:arrow_idx]
                    end_part = line[arrow_idx+6:]
                    
                    # Extract time and position from end_part
                    # Format: time (x, y, z) m
                    time_match = re.search(r'([\d.]+)\s+\(([-\d.]+),', end_part)
                    if not time_match:
                        continue
                    
                    time_val = float(time_match.group(1))
                    
                    # Get positions
                    start_pos = parse_position(start_part)
                    end_pos = parse_position(end_part)
                    
                    if not start_pos or not end_pos:
                        continue
                    
                    # Update positions based on receiver
                    if 'ugv' in receiver:
                        ugv_last_pos = end_pos
                    elif 'uav' in receiver:
                        uav_last_pos = end_pos
                    
                    # Calculate metrics if we have both positions
                    if uav_last_pos and ugv_last_pos:
                        distance = calculate_distance(uav_last_pos, ugv_last_pos)
                        if distance is not None and distance > 0:
                            path_loss = free_space_path_loss(distance, 2.4)
                            rssi = 20 - path_loss  # Assuming 20 dBm transmit power
                            
                            data_rows.append({
                                'time': time_val,
                                'uav_x': uav_last_pos[0],
                                'uav_y': uav_last_pos[1],
                                'uav_z': uav_last_pos[2],
                                'ugv_x': ugv_last_pos[0],
                                'ugv_y': ugv_last_pos[1],
                                'ugv_z': ugv_last_pos[2],
                                'distance': distance,
                                'path_loss': path_loss,
                                'rssi': rssi
                            })
                            reception_count += 1
                
                except Exception as e:
                    continue
    
    except FileNotFoundError:
        print(f"Error: Communication log file not found: {log_file}")
        return False
    
    # Write to CSV
    if not data_rows:
        print("Error: No data extracted from communication log")
        return False
    
    try:
        with open(output_file, 'w', newline='') as csvfile:
            fieldnames = ['time', 'uav_x', 'uav_y', 'uav_z', 'ugv_x', 'ugv_y', 'ugv_z', 
                         'distance', 'path_loss', 'rssi']
            writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
            
            writer.writeheader()
            writer.writerows(data_rows)
        
        print(f"Successfully exported {reception_count} data points to: {output_file}")
        print(f"\nColumns in CSV:")
        print(f"  - time: Simulation time (seconds)")
        print(f"  - uav_x, uav_y, uav_z: UAV position (meters)")
        print(f"  - ugv_x, ugv_y, ugv_z: UGV position (meters)")
        print(f"  - distance: Distance between UAV and UGV (meters)")
        print(f"  - path_loss: Free Space Path Loss (dB)")
        print(f"  - rssi: Estimated RSSI (dBm)")
        print(f"\nYou can now:")
        print(f"  1. Import this CSV into a spreadsheet (LibreOffice, Excel, etc.)")
        print(f"  2. Create plots of path_loss vs time, distance vs time, etc.")
        print(f"  3. Use Python/matplotlib to create custom plots")
        
        return True
    
    except Exception as e:
        print(f"Error writing CSV file: {e}")
        return False

def create_matplotlib_plots(csv_file="path_loss_data.csv", output_dir=".", output_base="plot"):
    """Create plots using matplotlib if available."""
    try:
        import matplotlib.pyplot as plt
        import pandas as pd
    except ImportError:
        print("\nNote: matplotlib/pandas not installed, using SVG fallback.")
        return create_svg_plots(csv_file, output_dir, output_base)

    try:
        # Read CSV data
        df = pd.read_csv(csv_file)

        # Create figure with subplots
        fig, axes = plt.subplots(2, 2, figsize=(14, 10))
        fig.suptitle('UAV-UGV Communication Analysis', fontsize=16)

        # Plot 1: Path Loss vs Time
        axes[0, 0].plot(df['time'], df['path_loss'], 'b-', linewidth=2, label='Path Loss')
        axes[0, 0].set_xlabel('Time (s)')
        axes[0, 0].set_ylabel('Path Loss (dB)')
        axes[0, 0].set_title('Path Loss Over Time')
        axes[0, 0].grid(True, alpha=0.3)
        axes[0, 0].legend()

        # Plot 2: Distance vs Time
        axes[0, 1].plot(df['time'], df['distance'], 'g-', linewidth=2, label='Distance')
        axes[0, 1].set_xlabel('Time (s)')
        axes[0, 1].set_ylabel('Distance (m)')
        axes[0, 1].set_title('Distance Between UAV and UGV')
        axes[0, 1].grid(True, alpha=0.3)
        axes[0, 1].legend()

        # Plot 3: RSSI vs Time
        axes[1, 0].plot(df['time'], df['rssi'], 'r-', linewidth=2, label='RSSI')
        axes[1, 0].set_xlabel('Time (s)')
        axes[1, 0].set_ylabel('RSSI (dBm)')
        axes[1, 0].set_title('RSSI (Signal Strength) Over Time')
        axes[1, 0].axhline(y=-70, color='orange', linestyle='--', label='Poor threshold')
        axes[1, 0].axhline(y=-60, color='yellow', linestyle='--', label='Fair threshold')
        axes[1, 0].grid(True, alpha=0.3)
        axes[1, 0].legend()

        # Plot 4: Distance vs Path Loss
        axes[1, 1].scatter(df['distance'], df['path_loss'], c=df['time'], cmap='viridis', s=30)
        axes[1, 1].set_xlabel('Distance (m)')
        axes[1, 1].set_ylabel('Path Loss (dB)')
        axes[1, 1].set_title('Path Loss vs Distance')
        cbar = plt.colorbar(axes[1, 1].collections[0], ax=axes[1, 1])
        cbar.set_label('Time (s)')
        axes[1, 1].grid(True, alpha=0.3)

        plt.tight_layout()

        # Save plots
        output_dir = Path(output_dir)
        output_dir.mkdir(parents=True, exist_ok=True)
        plot_file = output_dir / f"{output_base}_all.png"
        plt.savefig(plot_file, dpi=150, bbox_inches='tight')
        print(f"\nSaved combined plot: {plot_file}")

        # Create individual plots
        fig2, ax2 = plt.subplots(figsize=(12, 6))
        ax2.plot(df['time'], df['path_loss'], 'b-', linewidth=2.5)
        ax2.fill_between(df['time'], df['path_loss'].min(), df['path_loss'], alpha=0.3)
        ax2.set_xlabel('Time (seconds)', fontsize=12)
        ax2.set_ylabel('Path Loss (dB)', fontsize=12)
        ax2.set_title('Free Space Path Loss Over Time (2.4 GHz)', fontsize=14)
        ax2.grid(True, alpha=0.3)
        plot_file2 = output_dir / f"{output_base}_path_loss.png"
        plt.savefig(plot_file2, dpi=150, bbox_inches='tight')
        print(f"✓ Saved path loss plot: {plot_file2}")

        plt.close('all')
        return True

    except Exception as e:
        print(f"Error creating plots: {e}")
        return False

def _default_output_name(log_file):
    log_path = Path(log_file)
    stem = log_path.stem
    return f"{stem}_path_loss.csv"


def _export_one(log_file, output_file, make_plots, plot_dir):
    if not export_path_loss_csv(str(log_file), output_file):
        return False

    if make_plots:
        print("\n" + "=" * 70)
        print("Creating visualization plots...")
        print("=" * 70)
        plot_base = Path(output_file).with_suffix("").name
        if create_matplotlib_plots(output_file, plot_dir, plot_base):
            print("\nPlots created successfully.")
        else:
            print("\nPlots not created; install matplotlib/pandas if needed.")
    return True


def main():
    parser = argparse.ArgumentParser(description="Analyze .tlog files and export path loss CSV/plots")
    parser.add_argument("file", nargs="?", default=None, help="Specific .tlog file (name or path) to analyze")
    parser.add_argument("--results", default="results", help="Results directory (default: results)")
    parser.add_argument("--config-prefix", dest="config_prefix", default="Communication", help="Result file prefix (default: Communication)")
    parser.add_argument("--all", action="store_true", help="Export all .tlog files in results directory")
    parser.add_argument("--plots", action="store_true", help="Generate plots")
    parser.add_argument("--no-plots", action="store_true", help="Skip plot generation")
    parser.add_argument("--csv-dir", default=None, help="Directory for CSV outputs (default: same folder as input .tlog)")
    parser.add_argument("--plot-dir", default=None, help="Directory for plot outputs (default: same folder as input .tlog)")
    args = parser.parse_args()

    result_dir = Path(args.results)
    if not result_dir.exists():
        print(f"Error: Results directory not found: {result_dir}")
        sys.exit(1)

    default_output_dir = result_dir
    csv_dir = Path(args.csv_dir) if args.csv_dir else default_output_dir
    plot_dir = Path(args.plot_dir) if args.plot_dir else default_output_dir
    csv_dir.mkdir(parents=True, exist_ok=True)
    plot_dir.mkdir(parents=True, exist_ok=True)

    print("=" * 70)
    print("Path Loss Analysis and Export Tool")
    print("=" * 70)

    if args.file:
        # Support both full paths and just filenames
        log_file = Path(args.file)
        if not log_file.exists():
            # Try to find it in results directory
            alt_path = result_dir / args.file
            if alt_path.exists():
                log_file = alt_path
            else:
                print(f"Error: .tlog file not found: {args.file}")
                sys.exit(1)
        if args.csv_dir is None:
            csv_dir = log_file.parent
            csv_dir.mkdir(parents=True, exist_ok=True)
        if args.plot_dir is None:
            plot_dir = log_file.parent
            plot_dir.mkdir(parents=True, exist_ok=True)
        output_file = csv_dir / _default_output_name(log_file)
        make_plots = args.plots and not args.no_plots
        if not _export_one(log_file, output_file, make_plots, plot_dir):
            sys.exit(1)
        return

    if args.all:
        log_files = sorted(result_dir.glob("*.tlog"))
        if not log_files:
            print("Error: No .tlog files found in results directory")
            sys.exit(1)
        for log_file in log_files:
            output_file = csv_dir / _default_output_name(log_file)
            print("\n" + "=" * 70)
            print(f"Exporting: {log_file.name}")
            print("=" * 70)
            if not _export_one(log_file, output_file, args.plots and not args.no_plots, plot_dir):
                print(f"Skipping: {log_file.name}")
        return

    # Default: pick first matching prefix
    log_files = sorted(result_dir.glob(f"{args.config_prefix}-*.tlog"))
    if not log_files:
        print(f"Error: No .tlog files found for prefix '{args.config_prefix}' in {result_dir}")
        print("Hint: enable recordReceptionLog/recordTransmissionLog to generate .tlog files.")
        sys.exit(1)

    log_file = log_files[0]
    output_file = csv_dir / _default_output_name(log_file)
    make_plots = args.plots and not args.no_plots
    if not _export_one(log_file, output_file, make_plots, plot_dir):
        sys.exit(1)

if __name__ == "__main__":
    main()
