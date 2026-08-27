#!/usr/bin/env python3
"""Summarize RSSI and PER ranges from offline OMNeT network_metrics.csv files."""

from __future__ import annotations

import argparse
import csv
import math
import re
from dataclasses import dataclass, field
from pathlib import Path
from typing import Iterable


WORKSPACE_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_RESULTS_GLOB = "bags/results_baylands_lora_c*"
C2_REP_LABELS = {
    "rep01": "Simplex",
    "rep02": "Duplex",
    "rep03": "Distance Sweep",
}
C2_REP_SET_LABELS = {
    "rep01": "rep01-simplex-set",
    "rep02": "rep02-duplex-set",
    "rep03": "rep03-distance-sweep-set",
}


@dataclass
class RunningStats:
    count: int = 0
    total: float = 0.0
    min_value: float = math.nan
    max_value: float = math.nan

    def add(self, value: float) -> None:
        if not math.isfinite(value):
            return
        self.count += 1
        self.total += value
        self.min_value = value if self.count == 1 else min(self.min_value, value)
        self.max_value = value if self.count == 1 else max(self.max_value, value)

    def add_weighted_mean(self, value: float, count: int) -> None:
        if not math.isfinite(value) or count <= 0:
            return
        was_empty = self.count == 0
        self.count += count
        self.total += value * count
        self.min_value = value if was_empty else min(self.min_value, value)
        self.max_value = value if was_empty else max(self.max_value, value)

    @property
    def mean(self) -> float:
        return self.total / self.count if self.count else math.nan


@dataclass
class GroupStats:
    campaign: str
    mode: str
    condition: str = "all"
    csv_paths: set[Path] = field(default_factory=set)
    routes: set[str] = field(default_factory=set)
    reps: set[str] = field(default_factory=set)
    distance_m: RunningStats = field(default_factory=RunningStats)
    rssi: RunningStats = field(default_factory=RunningStats)
    per_pct: RunningStats = field(default_factory=RunningStats)
    distance_runs: list[list[tuple[float, float]]] = field(default_factory=list)
    rssi_runs: list[list[tuple[float, float]]] = field(default_factory=list)
    per_pct_runs: list[list[tuple[float, float]]] = field(default_factory=list)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Compute min/mean/max RSSI and PER from offline OMNeT network_metrics.csv files.",
    )
    parser.add_argument(
        "roots",
        nargs="*",
        help=f"Campaign result roots. Default: {DEFAULT_RESULTS_GLOB}",
    )
    parser.add_argument(
        "--out",
        default="bags/network_metric_rssi_per_summary.csv",
        help="Output CSV path. Markdown is written next to it with .md suffix.",
    )
    parser.add_argument(
        "--warmup",
        type=float,
        default=30.0,
        help="Ignore samples with sim_time_s below this value. Default follows thesis analysis warmup.",
    )
    parser.add_argument(
        "--by-rep",
        action="store_true",
        help="Alias for --grouping condition-mode.",
    )
    parser.add_argument(
        "--grouping",
        choices=("campaign-mode", "condition-mode", "condition"),
        default="campaign-mode",
        help=(
            "campaign-mode splits by campaign and network mode; condition-mode also splits by rep/condition; "
            "condition matches rep-average plot grouping and combines included network modes."
        ),
    )
    parser.add_argument(
        "--modes",
        default="simplex,duplex",
        help="Comma-separated modes to include. Default: simplex,duplex.",
    )
    parser.add_argument(
        "--stats-source",
        choices=("raw", "plot-line"),
        default="raw",
        help="raw uses all CSV samples; plot-line uses the same binned mean line as --rep-average-plots.",
    )
    parser.add_argument(
        "--average-bin-s",
        type=float,
        default=5.0,
        help="Time bin size for --stats-source plot-line. Matches plot_network_metrics.py default.",
    )
    return parser.parse_args()


def default_roots() -> list[Path]:
    return sorted(path for path in WORKSPACE_ROOT.glob(DEFAULT_RESULTS_GLOB) if path.is_dir())


def resolve_input_path(value: str) -> Path:
    path = Path(value).expanduser()
    if not path.is_absolute():
        path = WORKSPACE_ROOT / path
    return path.resolve()


def resolve_output_path(value: str) -> Path:
    path = Path(value).expanduser()
    if not path.is_absolute():
        path = WORKSPACE_ROOT / path
    return path.resolve()


def selected_modes(value: str) -> set[str]:
    return {part.strip().lower() for part in value.split(",") if part.strip()}


def finite_float(value: str | None) -> float:
    if value is None or value == "":
        return math.nan
    try:
        parsed = float(value)
    except ValueError:
        return math.nan
    return parsed if math.isfinite(parsed) else math.nan


def infer_campaign(path: Path) -> str:
    for part in path.parts:
        match = re.fullmatch(r"results_baylands_lora_c(\d+)", part)
        if match:
            return f"C{int(match.group(1))}"
        if re.fullmatch(r"C\d+", part):
            return part
    return path.name


def infer_rep(path: Path) -> str:
    for part in path.parts:
        if re.fullmatch(r"rep\d+", part):
            return part
    for parent in path.parents:
        match = re.search(r"_(r\d+)__", parent.name)
        if match:
            return match.group(1)
    return "unknown"


def infer_route(path: Path) -> str:
    for parent in path.parents:
        if re.fullmatch(r"R\d+_.+", parent.name):
            return parent.name
        match = re.match(r"^C\d+_Route([A-Z])_(.+?)_r\d+__", parent.name)
        if match:
            return f"Route{match.group(1)}_{match.group(2)}"
    return "unknown"


def infer_summary_route(path: Path) -> str:
    route = infer_route(path)
    route = re.sub(r"^R\d+_", "", route)
    return route.replace("_", " ").strip().lower()


def infer_mode(metrics_csv: Path) -> str:
    folder = metrics_csv.parent.name.lower()
    if "duplex" in folder:
        return "duplex"
    if folder.startswith("lora"):
        return "simplex"
    return "unknown"


def condition_label(campaign: str, rep: str, *, combine_modes: bool = False) -> str:
    if campaign == "C2":
        labels = C2_REP_LABELS if combine_modes else C2_REP_SET_LABELS
        return labels.get(rep.lower(), rep)
    return rep


def discover_csvs(root: Path) -> Iterable[Path]:
    yield from sorted(root.glob("**/offline_omnet/*/network_metrics.csv"))


def load_summary_distances(root: Path) -> dict[tuple[str, str, str], tuple[float, int]]:
    summary_csv = root / "plots" / "network" / "combined" / "network_metrics_summary.csv"
    distances: dict[tuple[str, str, str], tuple[float, int]] = {}
    if not summary_csv.is_file():
        return distances
    with summary_csv.open("r", encoding="utf-8", newline="") as handle:
        reader = csv.DictReader(handle)
        for row in reader:
            rep = (row.get("rep") or "").strip().lower()
            route = (row.get("route") or "").strip().lower()
            mode = (row.get("lora_mode") or "").strip().lower()
            mean_distance = finite_float(row.get("true_distance_m"))
            sample_value = finite_float(row.get("samples_distance"))
            samples = int(sample_value) if math.isfinite(sample_value) else 0
            if rep and route and mode and math.isfinite(mean_distance):
                distances[(rep, route, mode)] = (mean_distance, samples)
    return distances


def distance_mean_from_csv(path: Path, warmup_s: float) -> tuple[float, int] | None:
    stats = RunningStats()
    if not path.is_file():
        return None
    with path.open("r", encoding="utf-8", newline="") as handle:
        reader = csv.DictReader(handle)
        for row in reader:
            sim_time = finite_float(row.get("sim_time_s") or row.get("wall_time_s"))
            if math.isfinite(sim_time) and sim_time < warmup_s:
                continue
            stats.add(finite_float(row.get("link_distance_m")))
    if not stats.count:
        return None
    return stats.mean, stats.count


def sibling_duplex_distance(metrics_csv: Path, warmup_s: float) -> tuple[float, int] | None:
    folder = metrics_csv.parent.name
    if not folder.startswith("lora_"):
        return None
    sibling = metrics_csv.parent.parent / folder.replace("lora_", "lora-duplex_", 1) / "network_metrics.csv"
    return distance_mean_from_csv(sibling, warmup_s)


def group_key(campaign: str, mode: str, rep: str, grouping: str) -> tuple[str, str, str]:
    if grouping == "condition":
        return campaign, condition_label(campaign, rep, combine_modes=True), "all"
    if grouping == "condition-mode":
        return campaign, condition_label(campaign, rep), mode
    return campaign, "all", mode


def read_csv_into_group(
    path: Path,
    group: GroupStats,
    warmup_s: float,
    fallback_distance: tuple[float, int] | None = None,
) -> None:
    distance_run: list[tuple[float, float]] = []
    rssi_run: list[tuple[float, float]] = []
    per_run: list[tuple[float, float]] = []
    with path.open("r", encoding="utf-8", newline="") as handle:
        reader = csv.DictReader(handle)
        for row in reader:
            sim_time = finite_float(row.get("sim_time_s") or row.get("wall_time_s"))
            if math.isfinite(sim_time) and sim_time < warmup_s:
                continue
            distance = finite_float(row.get("link_distance_m"))
            group.distance_m.add(distance)
            if math.isfinite(sim_time) and math.isfinite(distance):
                distance_run.append((sim_time, distance))
            rssi = finite_float(row.get("rssi_dbm"))
            group.rssi.add(rssi)
            if math.isfinite(sim_time) and math.isfinite(rssi):
                rssi_run.append((sim_time, rssi))
            per = finite_float(row.get("packet_error_rate"))
            if math.isfinite(per):
                per_pct = per * 100.0
                group.per_pct.add(per_pct)
                if math.isfinite(sim_time):
                    per_run.append((sim_time, per_pct))
    if distance_run:
        group.distance_runs.append(distance_run)
    elif fallback_distance is not None:
        distance, samples = fallback_distance
        group.distance_m.add_weighted_mean(distance, samples)
        group.distance_runs.append([(0.0, distance)])
    if rssi_run:
        group.rssi_runs.append(rssi_run)
    if per_run:
        group.per_pct_runs.append(per_run)


def binned_mean_line(runs: list[list[tuple[float, float]]], bin_s: float) -> list[float]:
    per_run_bins: list[dict[int, float]] = []
    for run in runs:
        buckets: dict[int, list[float]] = {}
        for t, value in run:
            if not (math.isfinite(t) and math.isfinite(value)):
                continue
            buckets.setdefault(int(t // bin_s), []).append(value)
        if buckets:
            per_run_bins.append({idx: sum(values) / len(values) for idx, values in buckets.items() if values})
    if not per_run_bins:
        return []
    indices = sorted(set().union(*(run_bins.keys() for run_bins in per_run_bins)))
    values: list[float] = []
    for idx in indices:
        bin_values = [run_bins[idx] for run_bins in per_run_bins if idx in run_bins]
        if bin_values:
            values.append(sum(bin_values) / len(bin_values))
    return values


def stats_from_values(values: list[float]) -> RunningStats:
    stats = RunningStats()
    for value in values:
        stats.add(value)
    return stats


def apply_plot_line_stats(groups: list[GroupStats], bin_s: float) -> None:
    for group in groups:
        group.distance_m = stats_from_values(binned_mean_line(group.distance_runs, bin_s))
        group.rssi = stats_from_values(binned_mean_line(group.rssi_runs, bin_s))
        group.per_pct = stats_from_values(binned_mean_line(group.per_pct_runs, bin_s))


def collect_stats(roots: list[Path], modes: set[str], warmup_s: float, grouping: str) -> list[GroupStats]:
    groups: dict[tuple[str, str, str], GroupStats] = {}
    for root in roots:
        summary_distances = load_summary_distances(root)
        for metrics_csv in discover_csvs(root):
            mode = infer_mode(metrics_csv)
            if mode not in modes:
                continue
            campaign = infer_campaign(metrics_csv)
            rep = infer_rep(metrics_csv)
            key = group_key(campaign, mode, rep, grouping)
            group = groups.setdefault(key, GroupStats(campaign=key[0], condition=key[1], mode=key[2]))
            group.csv_paths.add(metrics_csv)
            group.routes.add(infer_route(metrics_csv))
            group.reps.add(rep)
            fallback_distance = summary_distances.get((rep.lower(), infer_summary_route(metrics_csv), mode))
            if fallback_distance is None:
                fallback_distance = sibling_duplex_distance(metrics_csv, warmup_s)
            read_csv_into_group(metrics_csv, group, warmup_s, fallback_distance)
    return sorted(groups.values(), key=lambda item: (item.campaign, item.condition, item.mode))


def fmt_float(value: float, digits: int) -> str:
    if not math.isfinite(value):
        return ""
    return f"{value:.{digits}f}"


def write_csv(groups: list[GroupStats], out_path: Path, warmup_s: float, stats_source: str) -> None:
    out_path.parent.mkdir(parents=True, exist_ok=True)
    with out_path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(
            [
                "campaign",
                "condition",
                "mode",
                "warmup_s",
                "stats_source",
                "runs",
                "routes",
                "reps",
                "distance_samples",
                "distance_mean_m",
                "rssi_samples",
                "rssi_min_dbm",
                "rssi_mean_dbm",
                "rssi_max_dbm",
                "per_samples",
                "per_min_pct",
                "per_mean_pct",
                "per_max_pct",
            ]
        )
        for group in groups:
            writer.writerow(
                [
                    group.campaign,
                    group.condition,
                    group.mode,
                    fmt_float(warmup_s, 1),
                    stats_source,
                    len(group.csv_paths),
                    len(group.routes - {"unknown"}),
                    ",".join(sorted(group.reps)),
                    group.distance_m.count,
                    fmt_float(group.distance_m.mean, 2),
                    group.rssi.count,
                    fmt_float(group.rssi.min_value, 2),
                    fmt_float(group.rssi.mean, 2),
                    fmt_float(group.rssi.max_value, 2),
                    group.per_pct.count,
                    fmt_float(group.per_pct.min_value, 3),
                    fmt_float(group.per_pct.mean, 3),
                    fmt_float(group.per_pct.max_value, 3),
                ]
            )


def write_markdown(groups: list[GroupStats], md_path: Path, warmup_s: float, stats_source: str) -> None:
    md_path.parent.mkdir(parents=True, exist_ok=True)
    with md_path.open("w", encoding="utf-8") as handle:
        handle.write("# RSSI and PER Summary\n\n")
        handle.write(f"Samples before {warmup_s:.1f} s simulation time are excluded.\n\n")
        handle.write(f"Statistics source: `{stats_source}`.\n\n")
        handle.write("| Campaign | Condition | Mode | Runs | Routes | Distance mean | RSSI min | RSSI mean | RSSI max | PER min | PER mean | PER max |\n")
        handle.write("| --- | --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |\n")
        for group in groups:
            handle.write(
                "| "
                f"{group.campaign} | {group.condition} | {group.mode} | "
                f"{len(group.csv_paths)} | {len(group.routes - {'unknown'})} | "
                f"{fmt_float(group.distance_m.mean, 2)} | "
                f"{fmt_float(group.rssi.min_value, 2)} | "
                f"{fmt_float(group.rssi.mean, 2)} | "
                f"{fmt_float(group.rssi.max_value, 2)} | "
                f"{fmt_float(group.per_pct.min_value, 3)} | "
                f"{fmt_float(group.per_pct.mean, 3)} | "
                f"{fmt_float(group.per_pct.max_value, 3)} |\n"
            )


def print_table(groups: list[GroupStats]) -> None:
    headers = ["Campaign", "Condition", "Mode", "Runs", "Mean distance m", "RSSI min/mean/max dBm", "PER min/mean/max %"]
    rows = []
    for group in groups:
        rows.append(
            [
                group.campaign,
                group.condition,
                group.mode,
                str(len(group.csv_paths)),
                fmt_float(group.distance_m.mean, 2),
                f"{fmt_float(group.rssi.min_value, 2)} / {fmt_float(group.rssi.mean, 2)} / {fmt_float(group.rssi.max_value, 2)}",
                f"{fmt_float(group.per_pct.min_value, 3)} / {fmt_float(group.per_pct.mean, 3)} / {fmt_float(group.per_pct.max_value, 3)}",
            ]
        )
    widths = [len(header) for header in headers]
    for row in rows:
        widths = [max(width, len(cell)) for width, cell in zip(widths, row)]
    print("  ".join(header.ljust(width) for header, width in zip(headers, widths)))
    print("  ".join("-" * width for width in widths))
    for row in rows:
        print("  ".join(cell.ljust(width) for cell, width in zip(row, widths)))


def main() -> None:
    args = parse_args()
    grouping = "condition-mode" if args.by_rep else args.grouping
    roots = [resolve_input_path(root) for root in args.roots] if args.roots else default_roots()
    roots = [root for root in roots if root.is_dir()]
    if not roots:
        raise SystemExit("No result roots found.")

    groups = collect_stats(roots, selected_modes(args.modes), args.warmup, grouping)
    if not groups:
        raise SystemExit("No matching network_metrics.csv files found.")
    if args.stats_source == "plot-line":
        apply_plot_line_stats(groups, max(args.average_bin_s, 0.1))

    out_path = resolve_output_path(args.out)
    md_path = out_path.with_suffix(".md")
    write_csv(groups, out_path, args.warmup, args.stats_source)
    write_markdown(groups, md_path, args.warmup, args.stats_source)
    print_table(groups)
    print(out_path)
    print(md_path)


if __name__ == "__main__":
    main()
