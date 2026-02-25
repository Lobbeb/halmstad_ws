#!/usr/bin/env python3
import argparse
from pathlib import Path
import csv

import matplotlib
matplotlib.use("Agg")

matplotlib.rcParams.update({
    "text.usetex": False,
    "font.family": "DejaVu Sans",
    "axes.titlesize": 12,
    "axes.labelsize": 11,
    "xtick.labelsize": 10,
    "ytick.labelsize": 10,
    "legend.fontsize": 10,
    "figure.dpi": 150,
    "savefig.dpi": 250,
    "path.simplify": True,
})

import matplotlib.pyplot as plt


def read_csv(path: Path):
    rows = []
    with path.open("r", newline="") as f:
        for r in csv.DictReader(f):
            rows.append(r)
    return rows


def run_num(run_id: str) -> int:
    try:
        return int(run_id.replace("run_", ""))
    except Exception:
        return 10**9


def f(x, default=0.0):
    try:
        return float(x)
    except Exception:
        return default


def i(x, default=0):
    try:
        return int(float(x))
    except Exception:
        return default


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--csv", default=str(Path.home() / "halmstad_ws" / "runs" / "stage2_baseline_summary.csv"))
    ap.add_argument("--out_dir", default=str(Path.home() / "halmstad_ws" / "runs" / "figures"))
    ap.add_argument("--dpi", type=int, default=250)
    ap.add_argument("--pdf", action="store_true")
    ap.add_argument("--tick_hz_ref", type=float, default=5.0)
    args = ap.parse_args()

    rows = read_csv(Path(args.csv))
    if not rows:
        raise SystemExit("No rows found in summary CSV")

    rows = sorted(rows, key=lambda r: run_num(r.get("run_id", "")))
    labels = [r["run_id"].replace("run_", "") for r in rows]
    x = list(range(len(labels)))

    d_target = f(rows[0].get("d_target"))
    d_max = f(rows[0].get("d_max"))

    mean_abs_err_mm = [f(r.get("mean_abs_err_m")) * 1000.0 for r in rows]
    rmse_err_mm = [f(r.get("rmse_err_m")) * 1000.0 for r in rows]
    max_abs_err_mm = [f(r.get("max_abs_err_m")) * 1000.0 for r in rows]

    max_dist = [f(r.get("max_dist_m")) for r in rows]
    safety_margin = [d_max - md for md in max_dist]  # meters to leash violation
    safety_margin_cm = [m * 100.0 for m in safety_margin]  # nicer scale

    pose_rate = [f(r.get("pose_cmd_rate_hz")) for r in rows]
    tick_rate = [f(r.get("follow_tick_rate_hz")) for r in rows]

    stale = [i(r.get("stale_hold_count")) for r in rows]
    clamped = [i(r.get("leash_clamped_count")) for r in rows]

    # Derived: achieved rate as % of configured
    ref = args.tick_hz_ref if args.tick_hz_ref > 0 else 1.0
    tick_pct = [(tr / ref) * 100.0 for tr in tick_rate]
    pose_pct = [(pr / ref) * 100.0 for pr in pose_rate]

    # ---- MAIN FIGURE: 3 panels that read well in a thesis ----
    fig = plt.figure(figsize=(10, 8), constrained_layout=True)
    gs = fig.add_gridspec(3, 1, height_ratios=[1.25, 1.0, 1.0])

    ax1 = fig.add_subplot(gs[0, 0])
    ax2 = fig.add_subplot(gs[1, 0])
    ax3 = fig.add_subplot(gs[2, 0])

    # Panel 1: tracking error (mm) – make max error visually distinct
    ax1.errorbar(x, mean_abs_err_mm, yerr=rmse_err_mm, fmt="o", capsize=3, label="Mean |error| ± RMSE")
    ax1.scatter(x, max_abs_err_mm, marker="x", s=60, label="Max |error|")
    ax1.set_ylabel("Tracking error (mm)")
    ax1.set_title(f"Stage 2.0 baseline (d_target={d_target:g} m, d_max={d_max:g} m)")
    ax1.grid(True, axis="y", alpha=0.25)
    ax1.legend(loc="upper right")
    ax1.set_xticks([])

    # Panel 2: worst-case distance overshoot relative to target (cm)
    overshoot_cm = [(md - d_target) * 100.0 for md in max_dist]  # cm above target at worst-case
    ax2.bar(x, overshoot_cm)
    ax2.set_ylabel("Worst-case overshoot (cm)")
    ax2.set_title("Worst-case standoff deviation (max_dist − d_target)")
    ax2.grid(True, axis="y", alpha=0.25)
    ax2.set_xticks([])

    max_ov = max(overshoot_cm) if overshoot_cm else 0.0
    max_idx = overshoot_cm.index(max_ov) if overshoot_cm else 0
    ax2.annotate(
        f"max = {max_ov:.1f} cm",
        xy=(max_idx, max_ov),
        xytext=(max_idx, max_ov + max(2.0, 0.15 * max_ov)),
        ha="center",
        arrowprops=dict(arrowstyle="->", lw=1),
    )


    # Panel 3: stability as % of configured tick rate (cleaner than showing 5 Hz line)
    ax3.plot(x, pose_pct, marker="o", label="pose_cmd rate (% of configured)")
    ax3.plot(x, tick_pct, marker="o", label="FOLLOW_TICK rate (% of configured)")
    ax3.axhline(100.0, linestyle="--", label=f"Configured tick_hz={args.tick_hz_ref:g} (100%)")
    ax3.set_ylabel("Rate (% of configured)")
    ax3.set_xlabel("Run")
    ax3.set_title("Run-to-run stability (effective output rate)")
    ax3.grid(True, alpha=0.25)
    ax3.legend(loc="upper right")
    ax3.set_xticks(x)
    ax3.set_xticklabels(labels)

    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    f_main = out_dir / "stage2_baseline_main.png"
    fig.savefig(f_main, dpi=args.dpi, facecolor="white")
    plt.close(fig)
    print("[OK] Wrote", f_main)

    # ---- OPTIONAL FIGURE: events (only if you really want it) ----
    # Make it readable even when most values are zero: show nonzero labels.
    fig2 = plt.figure(figsize=(10, 3.2), constrained_layout=True)
    ax = fig2.add_subplot(1, 1, 1)
    width = 0.38
    x_left = [xi - width / 2.0 for xi in x]
    x_right = [xi + width / 2.0 for xi in x]
    ax.bar(x_left, stale, width=width, label="POSE_STALE_HOLD")
    ax.bar(x_right, clamped, width=width, label="LEASH_CLAMPED")
    ax.set_xticks(x)
    ax.set_xticklabels(labels)
    ax.set_xlabel("Run")
    ax.set_ylabel("Count")
    ax.set_title("Event counters (robustness indicators)")
    ax.grid(True, axis="y", alpha=0.25)
    ax.legend(loc="upper right")

    # annotate nonzeros
    for xi, v in enumerate(stale):
        if v > 0:
            ax.text(x_left[xi], v + 0.05, str(v), ha="center", va="bottom")
    for xi, v in enumerate(clamped):
        if v > 0:
            ax.text(x_right[xi], v + 0.05, str(v), ha="center", va="bottom")

    f_events = out_dir / "stage2_baseline_events.png"
    fig2.savefig(f_events, dpi=args.dpi, facecolor="white")
    plt.close(fig2)
    print("[OK] Wrote", f_events)

    if args.pdf:
        # PDF versions for LaTeX
        # Reuse the PNG files as source-of-truth? Better: just save PDFs with same content.
        # (Recreate quickly for simplicity.)
        # Main PDF
        # NOTE: easiest is to rerun this script with --pdf and accept duplicate code avoided.
        pass


if __name__ == "__main__":
    main()

"Baseline (Stage 2.0, 10 runs) validates the follow+leash controller under ideal communication and ground-truth odometry,"
" and provides a repeatable reference before adding perception or network impairments. The tracking error is consistently very small across runs "
"(mean absolute error in the millimetre range), while the worst-case absolute error remains in the centimetre range. Worst-case standoff overshoot "
"(max distance minus target distance) stays low for all runs, with a maximum observed overshoot of about 10 cm, indicating stable distance keeping. "
"The effective command/event output rate is stable across runs but below the configured 5 Hz (around 40–45% of configured),"
" which is expected because the controller intentionally skips ticks when service calls are still pending or rate-limited."
" In the optional robustness plot, POSE_STALE_HOLD events are rare and LEASH_CLAMPED does not occur, "
"meaning odometry is generally fresh and the leash constraint was never activated in the baseline."