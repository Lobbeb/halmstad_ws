#!/usr/bin/env python3
"""
Analyze OMNeT++ .vec/.sca results and generate network metric plots (SVG) without
external plotting dependencies.
"""

import argparse
import csv
import math
import re
from pathlib import Path
from typing import Dict, List, Optional, Tuple


def _write_svg_line_plot(
    output_file: Path,
    xs: List[float],
    ys: List[float],
    title: str,
    x_label: str,
    y_label: str,
    line_color: str = "#1f77b4",
) -> bool:
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

    def map_x(x: float) -> float:
        return margin_left + (x - x_min) / (x_max - x_min) * plot_w

    def map_y(y: float) -> float:
        return margin_top + (1.0 - (y - y_min) / (y_max - y_min)) * plot_h

    points = " ".join(f"{map_x(x):.2f},{map_y(y):.2f}" for x, y in zip(xs, ys))

    grid_lines = []
    ticks = 10
    for i in range(ticks + 1):
        gx = margin_left + i * plot_w / ticks
        gy = margin_top + i * plot_h / ticks
        xv = x_min + i * (x_max - x_min) / ticks
        yv = y_max - i * (y_max - y_min) / ticks
        grid_lines.append(
            f'<line x1="{gx:.2f}" y1="{margin_top}" x2="{gx:.2f}" y2="{margin_top + plot_h}" stroke="#e6e6e6" stroke-width="1"/>'
        )
        grid_lines.append(
            f'<line x1="{margin_left}" y1="{gy:.2f}" x2="{margin_left + plot_w}" y2="{gy:.2f}" stroke="#e6e6e6" stroke-width="1"/>'
        )
        grid_lines.append(
            f'<text x="{gx:.2f}" y="{margin_top + plot_h + 24}" text-anchor="middle" font-size="12" fill="#333">{xv:.2f}</text>'
        )
        grid_lines.append(
            f'<text x="{margin_left - 12}" y="{gy + 4:.2f}" text-anchor="end" font-size="12" fill="#333">{yv:.2f}</text>'
        )

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

    output_file.write_text(svg, encoding="utf-8")
    return True


def _latest_matching(directory: Path, pattern: str) -> Optional[Path]:
    files = sorted(directory.glob(pattern), key=lambda p: p.stat().st_mtime, reverse=True)
    return files[0] if files else None


def _parse_vector_definitions(vec_file: Path) -> Dict[int, Tuple[str, str]]:
    definitions: Dict[int, Tuple[str, str]] = {}
    with vec_file.open("r", encoding="utf-8", errors="ignore") as f:
        for line in f:
            if not line.startswith("vector "):
                continue
            parts = line.strip().split()
            if len(parts) < 4:
                continue
            try:
                vec_id = int(parts[1])
            except ValueError:
                continue
            module = parts[2]
            name = parts[3]
            definitions[vec_id] = (module, name)
    return definitions


def _metric_key(module: str, name: str) -> Optional[str]:
    if module.endswith("ugv.wlan[0].radio") and name.startswith("minSnir:vector"):
        return "snir_ugv"
    if module.endswith("uav.wlan[0].radio") and name.startswith("minSnir:vector"):
        return "snir_uav"
    if module.endswith("ugv.wlan[0].radio") and name.startswith("packetErrorRate:vector"):
        return "per_ugv"
    if module.endswith("uav.wlan[0].radio") and name.startswith("packetErrorRate:vector"):
        return "per_uav"
    if module.endswith("ugv.app[0]") and name.startswith("endToEndDelay:vector"):
        return "e2e_delay_ugv"
    if module.endswith("ugv.app[0]") and name.startswith("throughput:vector"):
        return "throughput_ugv"
    if module.endswith("uav.app[0]") and name.startswith("throughput:vector"):
        return "throughput_uav"
    if module.endswith("ugv.wlan[0].mac.dcf.channelAccess.pendingQueue") and name.startswith("queueLength:vector"):
        return "queue_len_ugv"
    if module.endswith("uav.wlan[0].mac.dcf.channelAccess.pendingQueue") and name.startswith("queueLength:vector"):
        return "queue_len_uav"
    if module.endswith("ugv.wlan[0].mac.dcf.channelAccess.pendingQueue") and name.startswith("incomingDataRate:vector"):
        return "in_data_rate_ugv"
    if module.endswith("ugv.wlan[0].mac.dcf.channelAccess.pendingQueue") and name.startswith("outgoingDataRate:vector"):
        return "out_data_rate_ugv"
    if module.endswith("uav.wlan[0].mac.dcf.channelAccess.pendingQueue") and name.startswith("incomingDataRate:vector"):
        return "in_data_rate_uav"
    if module.endswith("uav.wlan[0].mac.dcf.channelAccess.pendingQueue") and name.startswith("outgoingDataRate:vector"):
        return "out_data_rate_uav"
    return None


def _parse_metric_series(vec_file: Path, id_to_metric: Dict[int, str]) -> Dict[str, List[Tuple[float, float]]]:
    series: Dict[str, List[Tuple[float, float]]] = {m: [] for m in sorted(set(id_to_metric.values()))}
    with vec_file.open("r", encoding="utf-8", errors="ignore") as f:
        for line in f:
            if not line:
                continue
            c0 = line[0]
            if c0 < "0" or c0 > "9":
                continue
            parts = line.strip().split()
            if len(parts) < 4:
                continue
            try:
                vec_id = int(parts[0])
            except ValueError:
                continue
            metric = id_to_metric.get(vec_id)
            if not metric:
                continue
            try:
                t = float(parts[2])
                v = float(parts[3])
            except ValueError:
                continue
            series[metric].append((t, v))

    # Transform units for readability.
    for key in ("snir_ugv", "snir_uav"):
        if key in series:
            converted = []
            for t, v in series[key]:
                if v > 0:
                    converted.append((t, 10.0 * math.log10(v)))
            series[key] = converted

    if "e2e_delay_ugv" in series:
        series["e2e_delay_ugv"] = [(t, v * 1000.0) for t, v in series["e2e_delay_ugv"]]

    return series


def _read_pathloss_rssi(pathloss_csv: Optional[Path]) -> List[Tuple[float, float]]:
    if pathloss_csv is None or not pathloss_csv.exists():
        return []
    points = []
    with pathloss_csv.open("r", newline="", encoding="utf-8", errors="ignore") as f:
        reader = csv.DictReader(f)
        for row in reader:
            try:
                points.append((float(row["time"]), float(row["rssi"])))
            except Exception:
                continue
    return points


def _parse_sca_summary(sca_file: Optional[Path]) -> Dict[str, float]:
    summary: Dict[str, float] = {}
    if sca_file is None or not sca_file.exists():
        return summary

    sent_re = re.compile(r"^scalar\s+\S*uav\.app\[0\]\s+packetSent:count\s+([\deE+\-.]+)")
    recv_re = re.compile(r"^scalar\s+\S*ugv\.app\[0\]\s+packetReceived:count\s+([\deE+\-.]+)")

    with sca_file.open("r", encoding="utf-8", errors="ignore") as f:
        for line in f:
            m = sent_re.match(line)
            if m:
                summary["packets_sent_uav"] = float(m.group(1))
                continue
            m = recv_re.match(line)
            if m:
                summary["packets_received_ugv"] = float(m.group(1))

    sent = summary.get("packets_sent_uav", 0.0)
    recv = summary.get("packets_received_ugv", 0.0)
    if sent > 0:
        summary["pdr_percent"] = 100.0 * recv / sent
        summary["loss_percent"] = 100.0 - summary["pdr_percent"]
    return summary


def _write_summary_txt(output_file: Path, series: Dict[str, List[Tuple[float, float]]], summary: Dict[str, float]) -> None:
    lines = ["Network Metrics Summary", "=" * 40]

    for metric, data in sorted(series.items()):
        if not data:
            continue
        vals = [v for _, v in data]
        lines.append(f"{metric}: count={len(vals)}, min={min(vals):.6g}, max={max(vals):.6g}, avg={sum(vals)/len(vals):.6g}")

    if summary:
        lines.append("")
        lines.append("Packet Summary")
        lines.append("-" * 40)
        for k in sorted(summary.keys()):
            lines.append(f"{k}: {summary[k]:.6g}")

    output_file.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> None:
    parser = argparse.ArgumentParser(description="Plot OMNeT network metrics from .vec/.sca")
    parser.add_argument("--results", default="results", help="Results directory")
    parser.add_argument("--vec", default=None, help="Specific .vec file")
    parser.add_argument("--sca", default=None, help="Specific .sca file")
    parser.add_argument("--pathloss-csv", default=None, help="Path loss CSV (for RSSI plot)")
    parser.add_argument("--config-prefix", default="Communication-GazeboBridge", help="File prefix to auto-pick latest run")
    parser.add_argument("--csv-dir", default=None, help="CSV output directory (default: same folder as input .vec)")
    parser.add_argument("--plot-dir", default=None, help="Plot output directory (default: same folder as input .vec)")
    parser.add_argument("--write-csv", action="store_true", help="Write combined metric CSV (disabled by default to save space)")
    args = parser.parse_args()

    result_dir = Path(args.results)

    vec_file = Path(args.vec) if args.vec else _latest_matching(result_dir, f"{args.config_prefix}-#*.vec")
    if vec_file is None or not vec_file.exists():
        raise SystemExit(f"No .vec file found for prefix '{args.config_prefix}' in {result_dir}")

    sca_file = Path(args.sca) if args.sca else _latest_matching(result_dir, f"{args.config_prefix}-#*.sca")
    csv_dir = Path(args.csv_dir) if args.csv_dir else vec_file.parent
    plot_dir = Path(args.plot_dir) if args.plot_dir else vec_file.parent
    csv_dir.mkdir(parents=True, exist_ok=True)
    plot_dir.mkdir(parents=True, exist_ok=True)

    base = vec_file.stem
    out_csv = csv_dir / f"{base}_network_metrics.csv"
    summary_txt = csv_dir / f"{base}_network_summary.txt"

    defs = _parse_vector_definitions(vec_file)
    id_to_metric: Dict[int, str] = {}
    for vec_id, (module, name) in defs.items():
        mk = _metric_key(module, name)
        if mk is not None:
            id_to_metric[vec_id] = mk

    series = _parse_metric_series(vec_file, id_to_metric)

    pathloss_csv = None
    if args.pathloss_csv:
        pathloss_csv = Path(args.pathloss_csv)
    else:
        guessed = _latest_matching(csv_dir, f"{args.config_prefix}-*_path_loss.csv")
        pathloss_csv = guessed
    rssi_points = _read_pathloss_rssi(pathloss_csv)
    if rssi_points:
        series["rssi_estimated"] = rssi_points

    # Export combined CSV only when explicitly requested (can be very large).
    if args.write_csv:
        with out_csv.open("w", newline="", encoding="utf-8") as f:
            writer = csv.writer(f)
            writer.writerow(["metric", "time", "value"])
            for metric, data in sorted(series.items()):
                for t, v in data:
                    writer.writerow([metric, f"{t:.9f}", f"{v:.9f}"])

    # Plot selected metrics
    plot_specs = {
        "rssi_estimated": ("Estimated RSSI Over Time", "Time (s)", "RSSI (dBm)", "#c92a2a"),
        "snir_ugv": ("UGV Min SNIR Over Time", "Time (s)", "SNIR (dB)", "#0057b8"),
        "snir_uav": ("UAV Min SNIR Over Time", "Time (s)", "SNIR (dB)", "#7b2cbf"),
        "per_ugv": ("UGV Packet Error Rate Over Time", "Time (s)", "Packet Error Rate", "#2f9e44"),
        "per_uav": ("UAV Packet Error Rate Over Time", "Time (s)", "Packet Error Rate", "#1c7c54"),
        "e2e_delay_ugv": ("UGV End-to-End Delay Over Time", "Time (s)", "Delay (ms)", "#d9480f"),
        "throughput_ugv": ("UGV Throughput Over Time", "Time (s)", "Throughput (bps)", "#0b7285"),
        "throughput_uav": ("UAV Throughput Over Time", "Time (s)", "Throughput (bps)", "#1864ab"),
        "in_data_rate_ugv": ("UGV MAC Queue Incoming Data Rate", "Time (s)", "Data Rate (bps)", "#2b8a3e"),
        "out_data_rate_uav": ("UAV MAC Queue Outgoing Data Rate", "Time (s)", "Data Rate (bps)", "#364fc7"),
        "queue_len_ugv": ("UGV MAC Pending Queue Length", "Time (s)", "Queue Length (pkts)", "#495057"),
        "queue_len_uav": ("UAV MAC Pending Queue Length", "Time (s)", "Queue Length (pkts)", "#343a40"),
    }

    made = 0
    for key, (title, xl, yl, color) in plot_specs.items():
        data = series.get(key, [])
        if not data:
            continue
        xs = [t for t, _ in data]
        ys = [v for _, v in data]
        out = plot_dir / f"{base}_{key}.svg"
        if _write_svg_line_plot(out, xs, ys, title, xl, yl, color):
            print(f"✓ {out}")
            made += 1

    summary = _parse_sca_summary(sca_file)
    _write_summary_txt(summary_txt, series, summary)

    print(f"\nParsed vec: {vec_file}")
    if sca_file:
        print(f"Parsed sca: {sca_file}")
    if pathloss_csv and pathloss_csv.exists():
        print(f"RSSI source: {pathloss_csv}")
    else:
        print("RSSI source: not found (run path_loss_analysis.py first for RSSI plot)")
    if args.write_csv:
        print(f"Combined metric CSV: {out_csv}")
    else:
        print("Combined metric CSV: skipped (use --write-csv to enable)")
    print(f"Summary: {summary_txt}")
    print(f"Plots generated: {made}")


if __name__ == "__main__":
    main()
