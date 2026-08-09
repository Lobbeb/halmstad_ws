#!/usr/bin/env python3
from __future__ import annotations

import argparse
import fnmatch
import json
import re
from pathlib import Path
from typing import Iterable

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from matplotlib.patches import Patch
from PIL import Image, ImageOps

STATUS_ORDER = [
    "matched",
    "correct_empty",
    "no_prediction",
    "multi_prediction",
    "false_positive_empty",
    "under_prediction",
]
STATUS_COLORS = {
    "matched": "#4c78a8",
    "correct_empty": "#72b7b2",
    "no_prediction": "#f58518",
    "multi_prediction": "#e45756",
    "false_positive_empty": "#b279a2",
    "under_prediction": "#54a24b",
}
ISSUE_STATUS_ORDER = [
    "no_prediction",
    "multi_prediction",
    "false_positive_empty",
    "under_prediction",
]
DATASET_LABELS = {
    "baylands_sam3_obb_super_75_15_10": "Main",
    "baylands_sam3_obb_generalization_art": "ART",
    "baylands_sam3_obb_generalization_playground": "PG",
    "baylands_sam3_obb_generalization_road_to_art": "R2A",
    "baylands_sam3_obb_generalization_rotundan": "ROT",
    "baylands_super_75_15_10": "Main",
    "baylands_generalization_art": "ART",
    "baylands_generalization_playground": "PG",
    "baylands_generalization_road_to_art": "R2A",
    "baylands_generalization_rotundan": "ROT",
}
HEATMAP_ROUTE_ORDER = [
    "rotundan",
    "road_to_west",
    "parkinglot_west",
    "road_to_spawn",
    "spawn",
    "road_to_east",
    "parkinglot_east",
    "road_to_strip",
    "strip",
]
HEATMAP_ROUTE_LABELS = {
    "rotundan": "R1",
    "road_to_west": "R2",
    "parkinglot_west": "R3",
    "road_to_spawn": "R4",
    "spawn": "R5",
    "road_to_east": "R6",
    "parkinglot_east": "R7",
    "road_to_strip": "R8",
    "strip": "R9",
}
HELDOUT_LABELS = {
    "art": "ART",
    "playground": "PG",
    "road_to_art": "R2A",
    "rotundan": "ROT",
}
MODELS_TO_HIDE = {
    "v3",
    "baylands-leader-v3",
}
PREDICTION_SCATTER_LABEL_MODELS = {
    "-yolo26n",
    "-yolo26s",
    "v4",
    "v9-2-full",
    "v9-1-default",
    "v10-tuned-no-road-to-art",
    "v7d-color-robust",
    "v8c-mildgeometric",
}
GENERALIZATION_SCATTER_LABEL_MODELS = {
    "-yolo26n",
    "-yolo26s",
    "v5-3",
    "v6",
    "v8c-mildgeometric",
    "v9-1-default",
    "v9-2-full",
    "v9a-tuned-defaults",
}
SCATTER_LABEL_MODELS = PREDICTION_SCATTER_LABEL_MODELS | GENERALIZATION_SCATTER_LABEL_MODELS
GENERALIZATION_V6PLUS_HIDE_LABELS = {
}

PREDICTION_V6PLUS_HIDE_LABELS = {
    "baylands-sam3-yolo26s": "y26s-SAM3",
    "v10-tuned-from-v9-tight-3": "v10-ultra3",
}

MODEL_LABELS = {
    "v10-tuned-no-road-to-art": "v10-R2A",
    "v10-tuned-no-playground": "v10-PG",
    "v10-tuned-no-rotundan": "v10-ROT",
    "v10-tuned-no-art": "v10-ART",
    "v10-tuned-from-v9-tight-5": "v10-ultra5",
    "v10-tuned-from-v9-tight-3": "v10-ultra3",
    "v10-tuned-full-2": "v10c-tndfull",
    "v10-tuned-again": "v10b-full",
    "v10-tuned-full": "v10-tuned",
    "v10-1-defaults": "v10a-defaults",
    "v9-fuller": "v9.3",
    "v9-tuned-full": "v9-tuned",
    "v9-2-full": "v9.2-full",
    "v9-1-default": "v9.1",
    "v9a-tuned-defaults": "v9a-td",
    "v9b-patient-saveper": "v9b-ptn",
    "v8-tune-from-v7a-best": "v8.0",
    "v8a-noearlystop": "v8a-nostp",
    "v8b-tinymosaic": "v8b-msc",
    "v8c-mildgeometric": "v8c-geo",
    "v7a-ft-nomosaic-lr0002": "v7a-nomsc",
    "v7b-ft-mosaic010": "v7b-msc.01",
    "v7c-ft-mosaic025-lowerase": "v7c-msc25",
    "v7d-color-robust": "v7d-clr",
    "v7e-imgsz768-nomosaic": "v7e-imgz768",
    "v6-nomosaic": "v6a-nomsc",
    "v6-025mosaic": "v6b-msc25",
    "v6": "v6.0",
    "v5-4": "v5.4",
    "v5-3": "v5.3",
    "v5-2": "v5.2",
    "v5-1": "v5.1",
    "v5": "v5.0",
    "v4-1": "v4.1",
    "v4": "v4.0",
    "v3": "v3.0",
    "baylands-sam3-yolo26s": "y26sS3",
    "baylands-sam3-yolo26n": "y26nS3",
    "baylands-original-yolo26s": "y26s1",
    "baylands-original-yolo26n": "y26n1",
    "-yolo26s": "y26s0",
    "-yolo26n": "y26n0",
    "baylands-leader-v3": "v3.0",
    "baylands-leader-v4-2": "v4.2",
    "baylands-leader-v5": "v5.0",
    "baylands-leader-v6": "v6.0",
}


def style() -> None:
    plt.rcParams.update(
        {
            "font.family": "DejaVu Serif",
            "font.size": 11,
            "axes.labelsize": 11,
            "xtick.labelsize": 8,
            "ytick.labelsize": 8,
            "legend.fontsize": 9,
            "axes.linewidth": 0.8,
            "lines.linewidth": 1.8,
            "savefig.dpi": 300,
            "figure.dpi": 120,
        }
    )


def save(fig: plt.Figure, path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(path, bbox_inches="tight", pad_inches=0.03)
    plt.close(fig)
    print(path)


def nice_model(name: str) -> str:
    text = str(name)
    if text in MODEL_LABELS:
        return MODEL_LABELS[text]
    text = text.replace("baylands-leader-", "")
    text = text.replace("baylands-generalized-", "gen-")
    text = text.replace("v10-tuned-", "v10-")
    text = text.replace("v10_tuned_", "v10-")
    text = text.replace("-tuned-", "-")
    return MODEL_LABELS.get(text, text)


def model_major_version(name: str) -> int | None:
    match = re.search(r"(?:^|-)(?:leader-)?v(\d+)", str(name))
    if not match:
        return None
    return int(match.group(1))


def filter_min_model_version(df: pd.DataFrame, min_version: int, model_col: str = "model_version") -> pd.DataFrame:
    if model_col not in df.columns:
        return df.copy()
    keep = df[model_col].astype(str).map(lambda name: (model_major_version(name) or -1) >= min_version)
    return df[keep].copy()


def compressed_axis_values(values: pd.Series | np.ndarray, threshold: float, low_span: float = 0.28) -> np.ndarray:
    arr = np.asarray(values, dtype=float)
    clipped = np.clip(arr, 0.0, 1.0)
    low = low_span * (1.0 - np.square(1.0 - np.clip(clipped / threshold, 0.0, 1.0)))
    high = low_span + ((clipped - threshold) / (1.0 - threshold)) * (1.0 - low_span)
    return np.where(clipped <= threshold, low, high)


def broken_axis_values(values: pd.Series | np.ndarray, threshold: float, first_square: float = 0.22) -> np.ndarray:
    arr = np.asarray(values, dtype=float)
    clipped = np.clip(arr, 0.0, 1.0)
    low = (clipped / threshold) * first_square
    high = first_square + ((clipped - threshold) / (1.0 - threshold)) * (1.0 - first_square)
    return np.where(clipped <= threshold, low, high)


def focused_axis_values(
    values: pd.Series | np.ndarray,
    low: float,
    high: float,
    low_span: float = 0.08,
    focus_span: float = 0.84,
) -> np.ndarray:
    arr = np.asarray(values, dtype=float)
    clipped = np.clip(arr, 0.0, 1.0)
    low_part = (clipped / low) * low_span
    mid_part = low_span + ((clipped - low) / (high - low)) * focus_span
    high_part = low_span + focus_span + ((clipped - high) / (1.0 - high)) * (1.0 - low_span - focus_span)
    return np.where(clipped <= low, low_part, np.where(clipped <= high, mid_part, high_part))


def normalized_score_axis_values(
    values: pd.Series | np.ndarray,
    center: float = 0.75,
    steepness: float = 8.0,
) -> np.ndarray:
    clipped = np.clip(np.asarray(values, dtype=float), 0.0, 1.0)
    lo = 1.0 / (1.0 + np.exp(-steepness * (0.0 - center)))
    hi = 1.0 / (1.0 + np.exp(-steepness * (1.0 - center)))
    scaled = 1.0 / (1.0 + np.exp(-steepness * (clipped - center)))
    return (scaled - lo) / (hi - lo)


def valid_label_windows(
    label_left_windows: list[tuple[tuple[float, float], tuple[float, float]]] | None,
) -> list[tuple[tuple[float, float], tuple[float, float]]]:
    windows = []
    for window in label_left_windows or []:
        try:
            (x1, y1), (x2, y2) = window
            x1, y1, x2, y2 = float(x1), float(y1), float(x2), float(y2)
        except (TypeError, ValueError):
            continue
        windows.append(((min(x1, x2), min(y1, y2)), (max(x1, x2), max(y1, y2))))
    return windows


def transform_label_windows(
    label_left_windows: list[tuple[tuple[float, float], tuple[float, float]]] | None,
    x_range: tuple[float, float],
    y_range: tuple[float, float],
) -> list[tuple[tuple[float, float], tuple[float, float]]]:
    windows = []
    for (x1, y1), (x2, y2) in valid_label_windows(label_left_windows):
        tx1 = float(focused_axis_values(np.array([x1]), x_range[0], x_range[1])[0])
        ty1 = float(focused_axis_values(np.array([y1]), y_range[0], y_range[1])[0])
        tx2 = float(focused_axis_values(np.array([x2]), x_range[0], x_range[1])[0])
        ty2 = float(focused_axis_values(np.array([y2]), y_range[0], y_range[1])[0])
        windows.append(((min(tx1, tx2), min(ty1, ty2)), (max(tx1, tx2), max(ty1, ty2))))
    return windows


def focused_axis_ticks(axis_range: tuple[float, float]) -> list[float]:
    low, high = axis_range
    ticks = [
        0.0,
        low / 2.0,
        low,
        *np.linspace(low, high, 7)[1:-1],
        high,
        (high + 1.0) / 2.0,
        1.0,
    ]
    return sorted({round(float(tick), 4) for tick in ticks if 0.0 <= tick <= 1.0})


def focused_axis_minor_ticks(axis_range: tuple[float, float], subdivisions: int = 4) -> list[float]:
    major_ticks = focused_axis_ticks(axis_range)
    minor_ticks = []
    for start, end in zip(major_ticks[:-1], major_ticks[1:]):
        minor_ticks.extend(np.linspace(start, end, subdivisions + 1)[1:-1])
    major_set = {round(tick, 4) for tick in major_ticks}
    return sorted(
        {
            round(float(tick), 4)
            for tick in minor_ticks
            if 0.0 <= tick <= 1.0 and round(float(tick), 4) not in major_set
        }
    )


def focused_axis_tick_labels(axis_range: tuple[float, float], ticks: list[float], hide_compressed_midpoints: bool = False) -> list[str]:
    low, high = axis_range
    hidden = set()
    if hide_compressed_midpoints:
        hidden = {round(low / 2.0, 4), round((high + 1.0) / 2.0, 4)}
    labels = []
    for tick in ticks:
        rounded = round(float(tick), 4)
        if rounded in hidden:
            labels.append("")
        elif rounded in {0.0, 1.0}:
            labels.append(str(int(rounded)))
        else:
            labels.append(f"{tick:.4f}")
    return labels


def compressed_axis_config(config: float | tuple[float, float] | None) -> tuple[float, float] | None:
    if config is None:
        return None
    if isinstance(config, tuple):
        return config
    return (config, config)


def set_broken_axes(ax: plt.Axes, x_threshold: float, y_threshold: float, first_square: float = 0.22) -> None:
    x_ticks = [0.0, x_threshold, 0.8, 0.9, 1.0]
    y_ticks = [0.0, y_threshold, 0.8, 0.9, 1.0]
    ax.set_xlim(0.0, 1.0)
    ax.set_ylim(0.0, 1.0)
    ax.set_xticks(broken_axis_values(np.array(x_ticks), x_threshold, first_square))
    ax.set_xticklabels([f"{tick:.2f}".rstrip("0").rstrip(".") for tick in x_ticks])
    ax.set_yticks(broken_axis_values(np.array(y_ticks), y_threshold, first_square))
    ax.set_yticklabels([f"{tick:.2f}".rstrip("0").rstrip(".") for tick in y_ticks])
    ax.axvline(first_square, color="0.75", linewidth=0.8, linestyle=":")
    ax.axhline(first_square, color="0.75", linewidth=0.8, linestyle=":")


def set_focused_axes(
    ax: plt.Axes,
    x_range: tuple[float, float],
    y_range: tuple[float, float],
    low_span: float = 0.08,
    focus_span: float = 0.84,
) -> None:
    x_ticks = focused_axis_ticks(x_range)
    y_ticks = focused_axis_ticks(y_range)
    x_minor_ticks = focused_axis_minor_ticks(x_range)
    y_minor_ticks = focused_axis_minor_ticks(y_range)
    ax.set_xlim(0.0, 1.0)
    ax.set_ylim(0.0, 1.0)
    ax.set_xticks(focused_axis_values(np.array(x_ticks), x_range[0], x_range[1], low_span, focus_span))
    ax.set_xticklabels(focused_axis_tick_labels(x_range, x_ticks, hide_compressed_midpoints=True))
    ax.set_yticks(focused_axis_values(np.array(y_ticks), y_range[0], y_range[1], low_span, focus_span))
    ax.set_yticklabels(focused_axis_tick_labels(y_range, y_ticks))
    ax.set_xticks(
        focused_axis_values(np.array(x_minor_ticks), x_range[0], x_range[1], low_span, focus_span),
        minor=True,
    )
    ax.set_yticks(
        focused_axis_values(np.array(y_minor_ticks), y_range[0], y_range[1], low_span, focus_span),
        minor=True,
    )
    ax.tick_params(axis="both", which="major", length=4.0, width=0.8)
    ax.tick_params(axis="both", which="minor", length=2.0, width=0.6)
    ax.axvline(low_span, color="0.75", linewidth=0.8, linestyle=":")
    ax.axhline(low_span, color="0.75", linewidth=0.8, linestyle=":")
    ax.axvline(low_span + focus_span, color="0.75", linewidth=0.8, linestyle=":")
    ax.axhline(low_span + focus_span, color="0.75", linewidth=0.8, linestyle=":")


def set_focused_score_xaxis(ax: plt.Axes, low: float = 0.7, high: float = 0.9) -> None:
    ticks = [0.0, 0.5, low, 0.75, 0.8, 0.85, high, 0.95, 1.0]
    positions = normalized_score_axis_values(np.array(ticks), center=(low + high) / 2.0)
    labels = [str(int(tick)) if tick in {0.0, 1.0} else f"{tick:.2f}".rstrip("0").rstrip(".") for tick in ticks]
    ax.set_xlim(0.0, 1.0)
    ax.set_xticks(positions)
    ax.set_xticklabels(labels)
    ax.axvline(float(normalized_score_axis_values(np.array([low]), center=(low + high) / 2.0)[0]), color="0.75", linewidth=0.8, linestyle=":")
    ax.axvline(float(normalized_score_axis_values(np.array([high]), center=(low + high) / 2.0)[0]), color="0.75", linewidth=0.8, linestyle=":")


def set_compressed_axes(ax: plt.Axes, threshold: float) -> None:
    ticks = [0.0, threshold / 3.0, 2.0 * threshold / 3.0, threshold, 0.85, 0.925, 1.0]
    ticks = [tick for tick in ticks if tick <= 1.0]
    positions = compressed_axis_values(np.array(ticks), threshold)
    labels = [f"{tick:.2f}".rstrip("0").rstrip(".") for tick in ticks]
    ax.set_xlim(0.0, 1.0)
    ax.set_ylim(0.0, 1.0)
    ax.set_xticks(positions)
    ax.set_xticklabels(labels)
    ax.set_yticks(positions)
    ax.set_yticklabels(labels)


def filter_models(df: pd.DataFrame, max_rank: int, exclude_globs: str, model_col: str = "model_version") -> pd.DataFrame:
    out = df.copy()
    if model_col in out.columns:
        out = out[~out[model_col].astype(str).isin(MODELS_TO_HIDE)]
    if max_rank > 0 and "rank" in out.columns:
        out = out[out["rank"].fillna(10**9).astype(float) <= max_rank]
    patterns = [p.strip() for p in exclude_globs.split(",") if p.strip()]
    if patterns and model_col in out.columns:
        keep = []
        for name in out[model_col].astype(str):
            keep.append(not any(fnmatch.fnmatch(name, pattern) for pattern in patterns))
        out = out[keep]
    return out.copy()


def label_for_dataset(path_or_name: str) -> str:
    name = Path(str(path_or_name)).name
    return DATASET_LABELS.get(name, name.replace("baylands_sam3_obb_generalization_", "").replace("_", " "))


def annotate_extreme_middle(ax: plt.Axes, df: pd.DataFrame, score_col: str, x_col: str, y_col: str) -> None:
    if df.empty or score_col not in df:
        return
    ranked = df.dropna(subset=[score_col, x_col, y_col]).sort_values(score_col)
    if ranked.empty:
        return
    selected = [ranked.iloc[0], ranked.iloc[len(ranked) // 2], ranked.iloc[-1]]
    seen = set()
    for row in selected:
        model = str(row["model_version"])
        if model in seen:
            continue
        seen.add(model)
        label_left = float(row[x_col]) > 0.5
        ax.annotate(
            nice_model(model),
            (row[x_col], row[y_col]),
            xytext=(-6, 0) if label_left else (6, 0),
            textcoords="offset points",
            fontsize=8,
            va="center",
            ha="right" if label_left else "left",
        )


def annotate_all_models(
    ax: plt.Axes,
    df: pd.DataFrame,
    x_col: str,
    y_col: str,
    skip_models: set[str] | None = None,
    label_left_windows: list[tuple[tuple[float, float], tuple[float, float]]] | None = None,
) -> set[str]:
    skip_models = skip_models or set()
    label_left_windows = valid_label_windows(label_left_windows)
    rows = df.dropna(subset=[x_col, y_col])
    annotated = set()
    for _, row in rows.iterrows():
        model = str(row["model_version"])
        if model in skip_models:
            continue
        annotated.add(model)
        x = float(row[x_col])
        y = float(row[y_col])
        label_left = any(x1 <= x <= x2 and y1 <= y <= y2 for (x1, y1), (x2, y2) in label_left_windows)
        ax.annotate(
            nice_model(model),
            (row[x_col], row[y_col]),
            xytext=(-6, 0) if label_left else (6, 0),
            textcoords="offset points",
            fontsize=6.5,
            va="center",
            ha="right" if label_left else "left",
        )
    return annotated


def annotate_selected_models(
    ax: plt.Axes,
    df: pd.DataFrame,
    x_col: str,
    y_col: str,
    models: set[str],
    label_left_threshold: float = 0.5,
) -> set[str]:
    rows = df[df["model_version"].astype(str).isin(models)].dropna(subset=[x_col, y_col])
    annotated = set()
    for _, row in rows.iterrows():
        model = str(row["model_version"])
        annotated.add(model)
        label_left = float(row[x_col]) >= label_left_threshold
        ax.annotate(
            nice_model(model),
            (row[x_col], row[y_col]),
            xytext=(-6, 0) if label_left else (6, 0),
            textcoords="offset points",
            fontsize=8,
            va="center",
            ha="right" if label_left else "left",
        )
    return annotated


def annotate_score_span(
    ax: plt.Axes,
    df: pd.DataFrame,
    x_col: str,
    y_col: str,
    min_score: float,
    max_score: float,
    skip_models: set[str] | None = None,
    label_left_threshold: float = 0.5,
) -> None:
    skip_models = skip_models or set()
    rows = df[(df[x_col] > min_score) & (df[x_col] < max_score)].dropna(subset=[x_col, y_col])
    for _, row in rows.iterrows():
        model = str(row["model_version"])
        if model in skip_models:
            continue
        label_left = float(row[x_col]) >= label_left_threshold
        ax.annotate(
            nice_model(model),
            (row[x_col], row[y_col]),
            xytext=(-6, 0) if label_left else (6, 0),
            textcoords="offset points",
            fontsize=8,
            va="center",
            ha="right" if label_left else "left",
        )


def annotate_clump_label(
    ax: plt.Axes,
    df: pd.DataFrame,
    x_col: str,
    y_col: str,
    anchor_x: float | None = None,
    label: str = "YOLO26n+s,\nv5 ↓",
    offset: tuple[float, float] = (8, 0),
) -> None:
    rows = df.dropna(subset=[x_col, y_col])
    if rows.empty:
        return
    ax.annotate(
        label,
        (float(anchor_x) if anchor_x is not None else float(rows[x_col].max()), float(rows[y_col].median())),
        xytext=offset,
        textcoords="offset points",
        fontsize=8,
        va="center",
        ha="left",
        arrowprops={"arrowstyle": "->", "linewidth": 0.8, "color": "0.25"},
    )


def plot_model_ranking(full: pd.DataFrame, out: Path, top_n: int) -> Path:
    cols = {
        "eval_score": ("Combined score", "#4c78a8"),
        "main_test_mAP50-95": ("Main test mAP50-95", "#f58518"),
        "gen_avg_mAP50-95": ("Generalization mAP50-95", "#54a24b"),
    }
    df = full.sort_values("eval_score", ascending=False).head(min(top_n, 10)).copy()
    df["label"] = df["model_version"].map(nice_model)
    y = np.arange(len(df))
    h = 0.3
    offsets = np.linspace(-h, h, len(cols))
    fig, ax = plt.subplots(figsize=(6.8, max(3.2, 0.42 * len(df))))
    score_low, score_high = 0.7, 0.9
    for offset, (col, (_, color)) in zip(offsets, cols.items()):
        for yi, (_, row) in zip(y, df.iterrows()):
            if pd.isna(row[col]):
                continue
            value = float(row[col])
            plot_value = float(normalized_score_axis_values(np.array([value]), center=(score_low + score_high) / 2.0)[0])
            ax.barh(yi + offset, plot_value, left=0.0, height=h, color=color, edgecolor="white", linewidth=0.25)
    ax.set_yticks(y)
    ax.set_yticklabels(df["label"])
    ax.invert_yaxis()
    ax.set_xlabel("Score")
    set_focused_score_xaxis(ax, score_low, score_high)
    ax.grid(axis="x", alpha=0.25)
    handles = [Patch(facecolor=color, label=label) for label, color in cols.values()]
    ax.legend(handles=handles, loc="lower center", bbox_to_anchor=(0.5, 1.01), ncol=3, frameon=False)
    path = out / "01_model_ranking_top_models.png"
    save(fig, path)
    return path


def plot_main_vs_generalization(
    full: pd.DataFrame,
    out: Path,
    filename: str = "02_main_vs_generalization_scatter.png",
    label_left_threshold: float = 0.5,
    show_all_names: bool = False,
    skip_label_models: set[str] | None = None,
    compressed_axis_threshold: float | tuple[float, float] | None = None,
    focused_axis_ranges: tuple[tuple[float, float], tuple[float, float]] | None = None,
    label_min_model_version: int | None = None,
    hide_focused_low_clump_labels: bool = False,
    label_left_windows: list[tuple[tuple[float, float], tuple[float, float]]] | None = None,
    clump_label: str = "YOLO26n+s,\nv5 ↓",
    clump_label_offset: tuple[float, float] = (8, 0),
    color_vmin: float | None = None,
) -> Path:
    df = full.dropna(subset=["main_test_mAP50-95", "gen_avg_mAP50-95", "pred_avg_score"]).copy()
    fig, ax = plt.subplots(figsize=(4.8, 4.2))
    x_col = "main_test_mAP50-95"
    y_col = "gen_avg_mAP50-95"
    raw_x_col = x_col
    raw_y_col = y_col
    label_threshold = label_left_threshold
    clump_df = pd.DataFrame()
    clump_mask = None
    axis_config = compressed_axis_config(compressed_axis_threshold)
    if focused_axis_ranges is not None:
        x_range, y_range = focused_axis_ranges
        if hide_focused_low_clump_labels:
            clump_mask = (df[raw_x_col] < x_range[0]) & (df[raw_y_col] < y_range[0])
        x_col = "_main_test_mAP50-95_plot"
        y_col = "_gen_avg_mAP50-95_plot"
        df[x_col] = focused_axis_values(df["main_test_mAP50-95"], x_range[0], x_range[1])
        df[y_col] = focused_axis_values(df["gen_avg_mAP50-95"], y_range[0], y_range[1])
        label_threshold = float(focused_axis_values(np.array([label_left_threshold]), x_range[0], x_range[1])[0])
        if label_left_windows is not None:
            label_left_windows = transform_label_windows(label_left_windows, x_range, y_range)
    elif axis_config is not None:
        x_threshold, y_threshold = axis_config
        x_col = "_main_test_mAP50-95_plot"
        y_col = "_gen_avg_mAP50-95_plot"
        df[x_col] = broken_axis_values(df["main_test_mAP50-95"], x_threshold)
        df[y_col] = broken_axis_values(df["gen_avg_mAP50-95"], y_threshold)
        label_threshold = float(broken_axis_values(np.array([label_left_threshold]), x_threshold)[0])
    color_min = float(color_vmin) if color_vmin is not None else float(df["pred_avg_score"].min())
    color_max = float(df["pred_avg_score"].max())
    sc = ax.scatter(
        df[x_col],
        df[y_col],
        c=df["pred_avg_score"],
        cmap="viridis",
        vmin=color_min,
        vmax=color_max,
        s=48,
        edgecolor="black",
        linewidth=0.4,
    )
    if clump_mask is not None:
        clump_df = df[clump_mask].copy()
    if show_all_names:
        label_df = filter_min_model_version(df, label_min_model_version) if label_min_model_version else df
        effective_skip = set(skip_label_models or set())
        effective_skip.update(clump_df["model_version"].astype(str).to_list())
        annotate_all_models(ax, label_df, x_col, y_col, effective_skip, label_left_windows)
        if not clump_df.empty:
            anchor_x = None
            if focused_axis_ranges is not None:
                anchor_x = float(focused_axis_values(np.array([0.69]), focused_axis_ranges[0][0], focused_axis_ranges[0][1])[0])
            annotate_clump_label(ax, clump_df, x_col, y_col, anchor_x, clump_label, clump_label_offset)
    else:
        annotate_selected_models(ax, df, x_col, y_col, SCATTER_LABEL_MODELS, label_threshold)
    if focused_axis_ranges is not None:
        set_focused_axes(ax, focused_axis_ranges[0], focused_axis_ranges[1])
    elif axis_config is not None:
        set_broken_axes(ax, x_threshold, y_threshold)
    ax.set_xlabel("Main test mAP50-95")
    ax.set_ylabel("Generalization mAP50-95")
    ax.grid(alpha=0.25)
    cbar = fig.colorbar(sc, ax=ax, fraction=0.046, pad=0.04)
    cbar.set_label("Prediction score")
    path = out / filename
    save(fig, path)
    return path


def plot_prediction_scatter(
    full: pd.DataFrame,
    out: Path,
    filename: str = "03_prediction_score_scatter.png",
    label_left_threshold: float = 0.5,
    show_all_names: bool = False,
    skip_label_models: set[str] | None = None,
    compressed_axis_threshold: float | tuple[float, float] | None = None,
    focused_axis_ranges: tuple[tuple[float, float], tuple[float, float]] | None = None,
    label_min_model_version: int | None = None,
    hide_focused_low_clump_labels: bool = False,
    label_left_windows: list[tuple[tuple[float, float], tuple[float, float]]] | None = None,
    clump_label: str = "YOLO26n+s,\nv5 ↓",
    clump_label_offset: tuple[float, float] = (8, 0),
    color_vmin: float | None = None,
) -> Path:
    df = full.dropna(subset=["main_pred_score", "gen_avg_pred_score", "eval_score"]).copy()
    fig, ax = plt.subplots(figsize=(4.8, 4.2))
    x_col = "main_pred_score"
    y_col = "gen_avg_pred_score"
    raw_x_col = x_col
    raw_y_col = y_col
    label_threshold = label_left_threshold
    clump_df = pd.DataFrame()
    clump_mask = None
    axis_config = compressed_axis_config(compressed_axis_threshold)
    if focused_axis_ranges is not None:
        x_range, y_range = focused_axis_ranges
        if hide_focused_low_clump_labels:
            clump_mask = (df[raw_x_col] < x_range[0]) & (df[raw_y_col] < y_range[0])
        x_col = "_main_pred_score_plot"
        y_col = "_gen_avg_pred_score_plot"
        df[x_col] = focused_axis_values(df["main_pred_score"], x_range[0], x_range[1])
        df[y_col] = focused_axis_values(df["gen_avg_pred_score"], y_range[0], y_range[1])
        label_threshold = float(focused_axis_values(np.array([label_left_threshold]), x_range[0], x_range[1])[0])
        if label_left_windows is not None:
            label_left_windows = transform_label_windows(label_left_windows, x_range, y_range)
    elif axis_config is not None:
        x_threshold, y_threshold = axis_config
        x_col = "_main_pred_score_plot"
        y_col = "_gen_avg_pred_score_plot"
        df[x_col] = broken_axis_values(df["main_pred_score"], x_threshold)
        df[y_col] = broken_axis_values(df["gen_avg_pred_score"], y_threshold)
        label_threshold = float(broken_axis_values(np.array([label_left_threshold]), x_threshold)[0])
    color_min = float(color_vmin) if color_vmin is not None else float(df["eval_score"].min())
    color_max = float(df["eval_score"].max())
    sc = ax.scatter(
        df[x_col],
        df[y_col],
        c=df["eval_score"],
        cmap="plasma",
        vmin=color_min,
        vmax=color_max,
        s=48,
        edgecolor="black",
        linewidth=0.4,
    )
    if clump_mask is not None:
        clump_df = df[clump_mask].copy()
    if show_all_names:
        label_df = filter_min_model_version(df, label_min_model_version) if label_min_model_version else df
        effective_skip = set(skip_label_models or set())
        effective_skip.update(clump_df["model_version"].astype(str).to_list())
        annotate_all_models(ax, label_df, x_col, y_col, effective_skip, label_left_windows)
        if not clump_df.empty:
            anchor_x = None
            if focused_axis_ranges is not None:
                anchor_x = float(focused_axis_values(np.array([0.69]), focused_axis_ranges[0][0], focused_axis_ranges[0][1])[0])
            annotate_clump_label(ax, clump_df, x_col, y_col, anchor_x, clump_label, clump_label_offset)
    else:
        annotated = annotate_selected_models(ax, df, x_col, y_col, SCATTER_LABEL_MODELS, label_threshold)
        annotate_score_span(ax, df, x_col, y_col, 0.6, 0.7, annotated, label_threshold)
    if focused_axis_ranges is not None:
        set_focused_axes(ax, focused_axis_ranges[0], focused_axis_ranges[1])
    elif axis_config is not None:
        set_broken_axes(ax, x_threshold, y_threshold)
    ax.set_xlabel("Main prediction score")
    ax.set_ylabel("Generalization prediction score")
    ax.grid(alpha=0.25)
    cbar = fig.colorbar(sc, ax=ax, fraction=0.046, pad=0.04)
    cbar.set_label("Evaluation score")
    path = out / filename
    save(fig, path)
    return path


def plot_heldout(heldout: pd.DataFrame, out: Path) -> Path:
    if heldout.empty:
        return None
    df = heldout.sort_values("rank").copy()
    labels = [HELDOUT_LABELS.get(str(v), str(v).replace("_", " ").title()) for v in df["heldout"]]
    x = np.arange(len(df))
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(7.0, 3.3), gridspec_kw={"width_ratios": [2.2, 1.0]})
    w = 0.26
    ax1.bar(x - w, df["heldout_mAP50-95"], width=w, label="mAP50-95")
    ax1.bar(x, df["pred_score"], width=w, label="Prediction score")
    ax1.bar(x + w, df["mean_iou"], width=w, label="Mean IoU")
    ax1.set_xticks(x)
    ax1.set_xticklabels(labels, rotation=25, ha="right")
    ax1.set_ylim(0, 1.02)
    ax1.set_ylabel("Score")
    ax1.grid(axis="y", alpha=0.25)
    ax1.legend(loc="lower center", bbox_to_anchor=(0.5, 1.02), ncol=2, frameon=False)
    ax2.bar(x, df["issue_rate"], color="#e45756")
    ax2.set_xticks(x)
    ax2.set_xticklabels(labels, rotation=25, ha="right")
    ax2.set_ylim(0, max(0.05, float(df["issue_rate"].max()) * 1.25))
    ax2.set_ylabel("Issue rate")
    ax2.grid(axis="y", alpha=0.25)
    path = out / "04_heldout_generalization_bars.png"
    save(fig, path)
    return path


def canonical_summaries(analysis_root: Path, model: str) -> list[Path]:
    root = analysis_root / model
    if not root.exists():
        return []
    preferred = [
        "baylands_sam3_obb_super_75_15_10",
        "baylands_sam3_obb_generalization_art",
        "baylands_sam3_obb_generalization_playground",
        "baylands_sam3_obb_generalization_road_to_art",
        "baylands_sam3_obb_generalization_rotundan",
    ]
    paths = []
    for name in preferred:
        p = root / name / "summary.json"
        if p.exists():
            paths.append(p)
    if paths:
        return paths
    return sorted(root.glob("*/summary.json"))


def load_summary_table(paths: Iterable[Path]) -> pd.DataFrame:
    rows = []
    for p in paths:
        data = json.loads(p.read_text())
        counts = data.get("status_counts", {}) or {}
        images = int(data.get("images", 0) or 0)
        row = {
            "dataset": p.parent.name,
            "label": label_for_dataset(p.parent.name),
            "images": images,
            "mean_iou": float(data.get("mean_best_iou_non_empty", np.nan)),
        }
        for status in STATUS_ORDER:
            row[status] = int(counts.get(status, 0) or 0)
            row[f"{status}_rate"] = row[status] / images if images > 0 else np.nan
        rows.append(row)
    return pd.DataFrame(rows)


def plot_issue_breakdown(summary_df: pd.DataFrame, out: Path) -> Path | None:
    if summary_df.empty:
        return None
    df = summary_df.copy()
    x = np.arange(len(df))
    fig, ax = plt.subplots(figsize=(6.4, 3.5))
    width = 0.18
    offsets = np.linspace(-1.5 * width, 1.5 * width, len(ISSUE_STATUS_ORDER))
    for offset, status in zip(offsets, ISSUE_STATUS_ORDER):
        vals = df[f"{status}_rate"].fillna(0).to_numpy()
        vals = np.where(vals > 0, vals, np.nan)
        ax.bar(x + offset, vals, width=width, label=status.replace("_", " "), color=STATUS_COLORS.get(status))
    ax.set_xticks(x)
    ax.set_xticklabels(df["label"], rotation=25, ha="right")
    ax.set_ylabel("Issue fraction")
    ax.set_yscale("log")
    ax.set_ylim(1e-3, 1.0)
    ax.grid(axis="y", which="both", alpha=0.25)
    ax.legend(loc="lower center", bbox_to_anchor=(0.5, 1.02), ncol=2, frameon=False)
    path = out / "05_prediction_issue_breakdown.png"
    save(fig, path)
    return path


def plot_iou_distribution(analysis_root: Path, model: str, summary_paths: list[Path], out: Path) -> Path | None:
    data = []
    labels = []
    for summary in summary_paths:
        comp = summary.parent / "comparison.csv"
        if not comp.exists():
            continue
        df = pd.read_csv(comp)
        vals = df.loc[df["best_iou"].notna() & (df["best_iou"] >= 0), "best_iou"].astype(float).to_numpy()
        if len(vals):
            data.append(vals)
            labels.append(label_for_dataset(summary.parent.name))
    if not data:
        return None
    fig, ax = plt.subplots(figsize=(6.4, 3.5))
    ax.boxplot(data, labels=labels, showfliers=False, patch_artist=True, boxprops={"facecolor": "#72b7b2", "alpha": 0.75})
    ax.set_ylabel("Best OBB IoU")
    ax.set_ylim(0, 1.02)
    ax.tick_params(axis="x", rotation=25)
    ax.grid(axis="y", alpha=0.25)
    path = out / "06_iou_distribution_by_dataset.png"
    save(fig, path)
    return path


def plot_validation_heatmap(csv_path: Path, out: Path) -> Path | None:
    if not csv_path.exists():
        return None
    df = pd.read_csv(csv_path)
    if df.empty:
        return None
    df = df[~df["model"].astype(str).str.endswith("-v1")].copy()
    df = df[~df["model"].astype(str).isin(MODELS_TO_HIDE)].copy()
    if df.empty:
        return None
    model_order = df.groupby("model")["map50_95"].mean().sort_values(ascending=False).head(8).index.tolist()
    sub = df[df["model"].isin(model_order)].copy()
    pivot = sub.pivot_table(index="model", columns="scenario", values="map50_95", aggfunc="mean")
    route_cols = [route for route in HEATMAP_ROUTE_ORDER if route in pivot.columns]
    extra_cols = [col for col in pivot.columns if col not in route_cols]
    pivot = pivot[route_cols + extra_cols]
    pivot = pivot.loc[model_order]
    fig, ax = plt.subplots(figsize=(6.8, 3.6))
    im = ax.imshow(pivot.to_numpy(), aspect="auto", cmap="viridis", vmin=0, vmax=1)
    ax.set_yticks(np.arange(len(pivot.index)))
    ax.set_yticklabels([nice_model(m) for m in pivot.index])
    ax.set_xticks(np.arange(len(pivot.columns)))
    ax.set_xticklabels([HEATMAP_ROUTE_LABELS.get(str(c), str(c).replace("_", " ")) for c in pivot.columns], rotation=0)
    for i in range(pivot.shape[0]):
        for j in range(pivot.shape[1]):
            value = pivot.iat[i, j]
            if np.isfinite(value):
                ax.text(j, i, f"{value:.2f}", ha="center", va="center", color="white" if value < 0.65 else "black", fontsize=8)
    cbar = fig.colorbar(im, ax=ax, fraction=0.046, pad=0.04)
    cbar.set_label("mAP50-95")
    path = out / "07_route_validation_heatmap.png"
    save(fig, path)
    return path


def plot_tuning_fitness(tune_root: Path, out: Path) -> Path | None:
    paths = sorted(p for p in tune_root.glob("*/tune_results.ndjson") if p.parent.name not in MODELS_TO_HIDE)
    series = []
    for p in paths:
        rows = []
        for line in p.read_text().splitlines():
            if not line.strip():
                continue
            try:
                d = json.loads(line)
            except Exception:
                continue
            rows.append((int(d.get("iteration", len(rows) + 1)), float(d.get("fitness", np.nan))))
        if rows:
            df = pd.DataFrame(rows, columns=["iteration", "fitness"]).dropna()
            if not df.empty:
                series.append((p.parent.name, df))
    if not series:
        return None
    series = sorted(series, key=lambda item: item[1]["fitness"].max(), reverse=True)[:6]
    fig, ax = plt.subplots(figsize=(6.4, 3.5))
    colors = plt.cm.tab10(np.linspace(0, 1, len(series)))
    linestyles = ["-", "--", "-.", ":", (0, (5, 2)), (0, (3, 1, 1, 1))]
    markers = ["o", "s", "^", "D", "v", "P"]
    for idx, (name, df) in enumerate(series):
        ax.plot(
            df["iteration"],
            df["fitness"].cummax(),
            label=nice_model(name),
            color=colors[idx],
            linestyle=linestyles[idx % len(linestyles)],
            marker=markers[idx % len(markers)],
            markersize=3,
            markevery=max(1, len(df) // 10),
            linewidth=2.0,
        )
    ax.set_xlabel("Tuning iteration")
    ax.set_ylabel("Best fitness")
    ax.set_ylim(0.82, 0.845)
    ax.grid(alpha=0.25)
    ax.legend(loc="lower center", bbox_to_anchor=(0.5, 1.02), ncol=2, frameon=False)
    path = out / "08_tuning_fitness_curves.png"
    save(fig, path)
    return path


def make_chosen_montage(
    chosen_dir: Path,
    out: Path,
    glob_pattern: str,
    limit: int,
    output_name: str,
    cols: int = 3,
) -> Path | None:
    files = sorted([p for p in chosen_dir.glob(glob_pattern) if p.suffix.lower() in {".jpg", ".jpeg", ".png"}])
    if limit > 0:
        files = files[:limit]
    if not files:
        return None
    tiles = []
    tile_w, tile_h = 360, 210
    for p in files:
        img = Image.open(p).convert("RGB")
        img.thumbnail((tile_w, tile_h), Image.Resampling.LANCZOS)
        canvas = Image.new("RGB", (tile_w, tile_h), (28, 28, 28))
        canvas.paste(img, ((tile_w - img.width) // 2, (tile_h - img.height) // 2))
        tiles.append(canvas)
    rows = int(np.ceil(len(tiles) / cols))
    gutter = 8
    montage = Image.new("RGB", (cols * tile_w + (cols - 1) * gutter, rows * tile_h + (rows - 1) * gutter), (28, 28, 28))
    for idx, tile in enumerate(tiles):
        x = (idx % cols) * (tile_w + gutter)
        y = (idx // cols) * (tile_h + gutter)
        montage.paste(tile, (x, y))
    path = out / output_name
    path.parent.mkdir(parents=True, exist_ok=True)
    montage.save(path, quality=95)
    print(path)
    return path


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--experiment-root", type=Path, default=Path("models/obb/experiments/sam3_hybrid_full"))
    parser.add_argument("--legacy-analysis-root", type=Path, default=Path("models/obb/experiments/results/analysis"))
    parser.add_argument("--chosen-dir", type=Path, default=Path("datasets/review/chosen"))
    parser.add_argument("--chosen-glob", default="*", help="Image glob used inside --chosen-dir for the chosen montage.")
    parser.add_argument("--chosen-limit", type=int, default=9, help="Max chosen montage images. 0 = all matching images.")
    parser.add_argument("--chosen-name", default="09_chosen_amcl_vs_sam3_montage.jpg")
    parser.add_argument("--chosen-cols", type=int, default=3)
    parser.add_argument("--out-dir", type=Path, default=Path("models/obb/experiments/thesis_plots/plots"))
    parser.add_argument("--top-n", type=int, default=12)
    parser.add_argument("--max-rank", type=int, default=0, help="Keep only models up to this rank. 0 = no rank filter.")
    parser.add_argument("--exclude-model-glob", default="", help="Comma-separated model globs to remove from thesis plots.")
    args = parser.parse_args()

    root = Path.cwd()
    exp = (root / args.experiment_root).resolve()
    out = (root / args.out_dir).resolve()
    legacy = (root / args.legacy_analysis_root).resolve()
    chosen = (root / args.chosen_dir).resolve()
    out.mkdir(parents=True, exist_ok=True)
    style()

    full_raw = pd.read_csv(exp / "scoreboards" / "full_scoreboard.csv")
    heldout_raw = pd.read_csv(exp / "scoreboards" / "heldout" / "heldout_scoreboard.csv")
    full = filter_models(full_raw, args.max_rank, args.exclude_model_glob, "model_version")
    heldout = filter_models(heldout_raw, args.max_rank, args.exclude_model_glob, "model_version")
    if full.empty:
        raise SystemExit("No full-scoreboard models left after filtering.")
    best_model = str(full.sort_values("rank").iloc[0]["model_version"])
    summary_paths = canonical_summaries(exp / "runs" / "prediction_analysis", best_model)
    summary_df = load_summary_table(summary_paths)

    generated: list[Path] = []
    generated.append(plot_model_ranking(full, out, args.top_n))
    generated.append(plot_main_vs_generalization(full, out))
    if not filter_min_model_version(full, 6).empty:
        generated.append(
            plot_main_vs_generalization(
                full,
                out,
                "02b_main_vs_generalization_scatter_v6plus.png",
                show_all_names=True,
                skip_label_models=GENERALIZATION_V6PLUS_HIDE_LABELS,
                focused_axis_ranges=((0.7995,0.8915),(0.6615,0.78)),
                label_min_model_version=None,
                hide_focused_low_clump_labels=True,
                clump_label="YOLO26n+s,\nv5.3 ↓",
                clump_label_offset=(8, -3),
                color_vmin=0.6,
                label_left_windows=[
                    ((0.83,0.74),(0.835,0.77)), # window for v10.1a
                    ((0.865,0.77),(0.88,0.8)), # window for top right clump
                    ((0.87,0.735),(0.883,0.745)), # window for v10.5
                    ((0.8840,0.7310),(0.8845,0.7320)), # v9a.tune,0.8841,0.7316
                    ((0.81,0.6975),(0.8275,0.7075)), # window for v7b & v8a
                    ((0.8230,0.675),(0.83,0.695)), # window for v7e & v6b
                    ((0.8240,0.6625),(0.8250,0.6650)), # v7d.clr,0.8246,0.6630
                    ((0.7220,0.8195),(0.7225,0.8200)), # v9.1,0.7223,0.8197
                    ((0.7710,0.6940),(0.7720,0.6950)),

#                   y26n.0,0.3552,0.3768
#                   y26s.0,0.3805,0.4209
#                   y26nS3,0.7091,0.6349
#                   y26sS3,0.7713,0.6946
#                   v10.1a,0.8313,0.7474
#                   v10.2,0.8457,0.6679
#                   v10.3,0.8502,0.6893
#                   v10.5,0.8805,0.7410
#                   v10.0,0.8271,0.6730
#                   v10.1b,0.8460,0.7041
#                   v10.ART,0.8812,0.7750
#                   v10.PG,0.8799,0.7738
#                   v10.R2A,0.8726,0.7791
#                   v10.ROT,0.8841,0.7800
#                   v4.0,0.3505,0.4001
#                   v4.1,0.3552,0.4060
#                   v5.0,0.3520,0.4008
#                   v5.1,0.3807,0.4255
#                   v5.2,0.3585,0.4080
#                   v5.3,0.6518,0.6452
#                   v5.4,0.3847,0.4228
#                   v6.0,0.7672,0.7157
#                   v6b.msc25,0.8291,0.6861
#                   v6a.nomsc,0.8286,0.6961
#                   v7a.nomsc,0.8647,0.7530
#                   v7b.msc01,0.8217,0.7048
#                   v7c.msc25,0.8227,0.6908
#                   v7d.clr,0.8246,0.6630
#                   v7e.768,0.8245,0.6946
#                   v8.0,0.8871,0.7296
#                   v8a.nostp,0.8272,0.6985
#                   v8b.msc,0.8277,0.7032
#                   v8c.geo,0.8242,0.6553
#                   v9.1,0.7223,0.8197
#                   v9.2,0.8002,0.7899
#                   v9.3,0.8399,0.7512
#                   v9.0,0.8530,0.7080
#                   v9a.tune,0.8841,0.7316
#                   v9b.ptn,0.8828,0.7325
                ],
            )
        )
    generated.append(plot_prediction_scatter(full, out))
    if not filter_min_model_version(full, 6).empty:
        generated.append(
            plot_prediction_scatter(
                full,
                out,
                "03b_prediction_score_scatter_v6plus.png",
                show_all_names=True,
                skip_label_models=PREDICTION_V6PLUS_HIDE_LABELS,
                focused_axis_ranges=((0.7655, 0.822), (0.5600, 0.7625)),
                label_min_model_version=None,
                hide_focused_low_clump_labels=True,
                clump_label="YOLO26n+s,\nv5.2 ↓",
                clump_label_offset=(8, 0),
                color_vmin=0.6,
                label_left_windows=[
                    ((0.8040,0.635),(0.8145,0.6575)), # v9.0, v8.0
                    ((0.795,0.61),(0.8025,0.6525)), # v10, v7c
                    ((0.7950,0.6460),(0.8040,0.696)), # v6a, v7e, v8c, v8b
                    ((0.7865,0.6985),(0.7870,0.6990)), # v10.1a
                    ((0.8085,0.655),(0.8175,0.6670)), # v10.3, v10.ART
                    ((0.8210,0.6510),(0.8215,0.6520)), # v9a.tune,0.8212,0.6518

#                   y26n.0,0.4156,0.3226
#                   y26s.0,0.4883,0.3905
#                   y26nS3,0.6920,0.5966
#                   y26sS3,0.7481,0.5958
#                   v10.1a,0.7869,0.6988
#                   v10.2,0.8133,0.5326
#                   v10.3,0.8087,0.6656
#                   v10.5,0.8081,0.6641
#                   v10.0,0.7979,0.6504
#                   v10.1b,0.8147,0.6412
#                   v10.ART,0.8163,0.6558
#                   v10.PG,0.8090,0.6918
#                   v10.R2A,0.8114,0.7573
#                   v10.ROT,0.8143,0.6097
#                   v4.0,0.5266,0.5469
#                   v4.1,0.5149,0.5272
#                   v5.0,0.4774,0.4819
#                   v5.1,0.5100,0.4941
#                   v5.2,0.4989,0.4991
#                   v5.3,0.7125,0.6606
#                   v5.4,0.5176,0.4224
#                   v6.0,0.7624,0.7163
#                   v6b.msc25,0.8048,0.6275
#                   v6a.nomsc,0.8012,0.6649
#                   v7a.nomsc,0.8005,0.6969
#                   v7b.msc01,0.7929,0.7051
#                   v7c.msc25,0.8015,0.6264
#                   v7d.clr,0.7885,0.5611
#                   v7e.768,0.7997,0.6750
#                   v8.0,0.8123,0.6445
#                   v8a.nostp,0.7745,0.5977
#                   v8b.msc,0.7970,0.6956
#                   v8c.geo,0.7975,0.6861
#                   v9.1,0.7123,0.7587
#                   v9.2,0.7682,0.7356
#                   v9.3,0.8098,0.7167
#                   v9.0,0.8143,0.6371
#                   v9a.tune,0.8212,0.6518
#                   v9b.ptn,0.8074,0.6509
                ],
            )
        )
    p = plot_heldout(heldout, out)
    if p:
        generated.append(p)
    p = plot_issue_breakdown(summary_df, out)
    if p:
        generated.append(p)
    p = plot_iou_distribution(exp / "runs" / "prediction_analysis", best_model, summary_paths, out)
    if p:
        generated.append(p)
    p = plot_validation_heatmap(legacy / "all_validation_runs.csv", out)
    if p:
        generated.append(p)
    p = plot_tuning_fitness(exp / "runs" / "tune", out)
    if p:
        generated.append(p)
    p = make_chosen_montage(chosen, out, args.chosen_glob, args.chosen_limit, args.chosen_name, args.chosen_cols)
    if p:
        generated.append(p)

    manifest = out.parent / "plots_manifest.csv"
    pd.DataFrame({"plot": [str(p.relative_to(out.parent)) for p in generated]}).to_csv(manifest, index=False)
    print(manifest)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
