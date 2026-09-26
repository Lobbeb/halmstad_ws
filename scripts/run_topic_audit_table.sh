#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

MODE="follow"
OUT="$WS_ROOT/debug_topics_table.md"
IGNORE="$WS_ROOT/debug_topics_ignore.txt"
VISITED=""
SAMPLE_S="5"
SAMPLES="10"
DISCOVERY_S="2"
MAX_TOPICS=""
INCLUDE_IGNORED="false"
APPEND_VALUED_TO_IGNORE="false"
MARK_SILENT_VISITED="false"
VALUED_ONLY="false"
REFRESH_ONLY="false"
WATCH_NEW="false"
WATCH_INTERVAL_S="0.25"
RUN_CMD=()

usage() {
  cat <<EOF
Usage:
  ./run.sh topic_audit_table [mode:=follow|yolo|yolo_tracker|yolo_visual_bridge|yolo_full|support_yolo] [sample_s:=5] [samples:=10]

Simple ROS topic audit:
  1. subscribe to each listed topic
  2. collect up to samples messages until sample_s timeout
  3. calculate hz and bw
  4. write publisher/subscriber counts and message type

Options:
  mode:=follow|yolo|yolo_tracker|yolo_visual_bridge|yolo_full|support_yolo
                                Column to update. Default: follow.
  out:=PATH                     Output markdown table. Default: debug_topics_table.md.
  ignore:=PATH                  Regex ignore file. Default: debug_topics_ignore.txt.
  include_ignored:=true         Ignore the ignore file. Default: false.
  valued_only:=true             Only sample live topics already valued in out:= table. Default: false.
  refresh_only:=true            Rewrite md/csv from existing table without sampling. Default: false.
  watch_new:=true               Discover and subscribe to topics that appear during sample_s. Default: false.
  watch_interval_s:=SECONDS     Dynamic discovery interval. Default: 0.25.
  sample_s:=SECONDS             Sampling timeout. Default: 5.
  samples:=N                    Stop early after N messages on every subscribed topic. Default: 10.
  discovery_s:=SECONDS          Wait for DDS topic type discovery. Default: 2.
  batch_samples:=N              Alias for samples:=N.
  max_topics:=N                 Limit topics for quick tests.
  visited:=PATH                 Topics with observed traffic. Default: <out>.visited.
  append_valued_to_ignore:=true Append topics with measured hz/bw to ignore on exit.
  append_visited_to_ignore:=true Compatibility alias for append_valued_to_ignore.
  mark_silent_visited:=true     Also mark silent topics with publishers as visited. Default: false.

Optional command:
  ./run.sh topic_audit_table watch_new:=true sample_s:=30 -- ./run.sh tmux_1to1 baylands mode:=yolo
EOF
}

while [ "$#" -gt 0 ]; do
  arg="$1"
  shift
  case "$arg" in
    --)
      RUN_CMD=("$@")
      break
      ;;
    help|-h|--help)
      usage
      exit 0
      ;;
    mode:=*) MODE="${arg#mode:=}" ;;
    out:=*) OUT="${arg#out:=}" ;;
    ignore:=*) IGNORE="${arg#ignore:=}" ;;
    include_ignored:=*) INCLUDE_IGNORED="${arg#include_ignored:=}" ;;
    valued_only:=*) VALUED_ONLY="${arg#valued_only:=}" ;;
    refresh_only:=*) REFRESH_ONLY="${arg#refresh_only:=}" ;;
    watch_new:=*) WATCH_NEW="${arg#watch_new:=}" ;;
    watch_interval_s:=*) WATCH_INTERVAL_S="${arg#watch_interval_s:=}" ;;
    sample_s:=*) SAMPLE_S="${arg#sample_s:=}" ;;
    samples:=*) SAMPLES="${arg#samples:=}" ;;
    discovery_s:=*) DISCOVERY_S="${arg#discovery_s:=}" ;;
    batch_samples:=*) SAMPLES="${arg#batch_samples:=}" ;;
    max_topics:=*) MAX_TOPICS="${arg#max_topics:=}" ;;
    visited:=*) VISITED="${arg#visited:=}" ;;
    append_valued_to_ignore:=*) APPEND_VALUED_TO_IGNORE="${arg#append_valued_to_ignore:=}" ;;
    append_visited_to_ignore:=*) APPEND_VALUED_TO_IGNORE="${arg#append_visited_to_ignore:=}" ;;
    mark_silent_visited:=*) MARK_SILENT_VISITED="${arg#mark_silent_visited:=}" ;;
    *)
      echo "Unknown argument: $arg" >&2
      usage >&2
      exit 2
      ;;
  esac
done

case "$MODE" in
  follow|yolo|yolo_tracker|yolo_visual_bridge|yolo_full|support_yolo) ;;
  *) echo "Invalid mode: $MODE" >&2; usage >&2; exit 2 ;;
esac
case "$SAMPLES" in
  ''|*[!0-9]*|0) echo "Invalid samples: $SAMPLES" >&2; exit 2 ;;
esac

case "$OUT" in /*) ;; *) OUT="$WS_ROOT/$OUT" ;; esac
case "$IGNORE" in /*) ;; *) IGNORE="$WS_ROOT/$IGNORE" ;; esac
if [ -z "$VISITED" ]; then
  VISITED="${OUT}.visited"
fi
case "$VISITED" in /*) ;; *) VISITED="$WS_ROOT/$VISITED" ;; esac

set +u
source /opt/ros/jazzy/setup.bash >/dev/null 2>&1
if [ -f "$WS_ROOT/install/setup.bash" ]; then
  source "$WS_ROOT/install/setup.bash" >/dev/null 2>&1
fi
set -u

tmp_dir="$(mktemp -d)"
topic_list="$tmp_dir/topics.txt"
rows_tsv="$tmp_dir/rows.tsv"
trap 'rm -rf "$tmp_dir"' EXIT

ros2 topic list --no-daemon > "$topic_list.raw" 2>/dev/null || true
grep '^/' "$topic_list.raw" | sort > "$topic_list" || true
if [ "$REFRESH_ONLY" = "true" ]; then
  : > "$topic_list"
elif [ "$VALUED_ONLY" = "true" ]; then
  python3 - "$OUT" "$tmp_dir/valued_topics.txt" <<'PY'
import re
import sys
from pathlib import Path
import csv

out_path = Path(sys.argv[1])
valued_path = Path(sys.argv[2])
csv_path = out_path.with_suffix(".csv")

def valued(value):
    return value.strip() not in ("", "-", "silent", "unknown", "?")

topics = []
if csv_path.exists():
    with csv_path.open("r", encoding="utf-8", newline="") as handle:
        reader = csv.DictReader(handle)
        for row in reader:
            topic = str(row.get("topic", "")).strip()
            if topic and (valued(str(row.get("hz", ""))) or valued(str(row.get("bw", "")))):
                topics.append(topic)
elif out_path.exists():
    for line in out_path.read_text(encoding="utf-8").splitlines():
        if not line.startswith("| /"):
            continue
        parts = [part.strip() for part in line.strip().strip("|").split("|")]
        if len(parts) >= 3 and (valued(parts[1]) or valued(parts[2])):
            topics.append(parts[0])

valued_path.write_text("\n".join(sorted(set(topics))) + ("\n" if topics else ""), encoding="utf-8")
PY
  grep -Fx -f "$tmp_dir/valued_topics.txt" "$topic_list" > "$topic_list.valued" || true
  mv "$topic_list.valued" "$topic_list"
elif [ "$INCLUDE_IGNORED" != "true" ] && [ -s "$IGNORE" ]; then
  grep -Ev -f "$IGNORE" "$topic_list" > "$topic_list.filtered" || true
  mv "$topic_list.filtered" "$topic_list"
fi
if [ -n "$MAX_TOPICS" ]; then
  head -n "$MAX_TOPICS" "$topic_list" > "$topic_list.limited"
  mv "$topic_list.limited" "$topic_list"
fi

mkdir -p "$(dirname "$OUT")" "$(dirname "$VISITED")"

if [ "${#RUN_CMD[@]}" -gt 0 ]; then
  echo "[topic_audit_table] starting command: ${RUN_CMD[*]}"
  "${RUN_CMD[@]}" &
  run_cmd_pid="$!"
else
  run_cmd_pid=""
fi

python3 - "$MODE" "$OUT" "$VISITED" "$SAMPLE_S" "$SAMPLES" "$DISCOVERY_S" "$MARK_SILENT_VISITED" "$topic_list" "$rows_tsv" "$WATCH_NEW" "$WATCH_INTERVAL_S" "$IGNORE" "$INCLUDE_IGNORED" "$MAX_TOPICS" <<'PY'
import csv
import re
import sys
import time
from collections import defaultdict
from pathlib import Path

import pandas as pd
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rosidl_runtime_py.utilities import get_message

(
    mode,
    out_path,
    visited_path,
    sample_s_raw,
    samples_raw,
    discovery_s_raw,
    mark_silent_raw,
    topic_list_path,
    rows_path,
    watch_new_raw,
    watch_interval_s_raw,
    ignore_path_raw,
    include_ignored_raw,
    max_topics_raw,
) = sys.argv[1:15]
out = Path(out_path)
csv_out = out.with_suffix(".csv")
visited_path = Path(visited_path)
sample_s = max(0.1, float(sample_s_raw))
samples = max(1, int(samples_raw))
discovery_s = max(0.0, float(discovery_s_raw))
mark_silent = mark_silent_raw.lower() in ("1", "true", "yes", "on")
watch_new = watch_new_raw.lower() in ("1", "true", "yes", "on")
watch_interval_s = max(0.05, float(watch_interval_s_raw))
include_ignored = include_ignored_raw.lower() in ("1", "true", "yes", "on")
max_topics = int(max_topics_raw) if max_topics_raw.strip().isdigit() else 0
topics = [line.strip() for line in Path(topic_list_path).read_text(encoding="utf-8").splitlines() if line.strip()]
ignore_patterns = []
ignore_path = Path(ignore_path_raw)
if ignore_path.exists() and not include_ignored:
    ignore_patterns = [
        re.compile(line.strip())
        for line in ignore_path.read_text(encoding="utf-8").splitlines()
        if line.strip() and not line.strip().startswith("#")
    ]
mode_columns = ["follow", "yolo", "yolo_tracker", "yolo_visual_bridge", "yolo_full", "support_yolo"]
mode_labels = {
    "follow": "FOLLOW",
    "yolo": "YOLO",
    "yolo_tracker": "TRACKER",
    "yolo_visual_bridge": "VISUAL BRIDGE",
    "yolo_full": "FULL",
    "support_yolo": "SUPPORT YOLO",
}

def human_bw(bytes_per_s):
    units = ["B/s", "KB/s", "MB/s", "GB/s"]
    value = float(bytes_per_s)
    for unit in units:
        if abs(value) < 1024.0 or unit == units[-1]:
            return f"{value:.0f} {unit}" if unit == "B/s" else f"{value:.2f} {unit}"
        value /= 1024.0
    return f"{value:.2f} GB/s"

def bw_score(value):
    match = re.search(r"([0-9.]+)\s*([KMGT]?B)/s", value)
    if not match:
        return -1.0
    scale = {"B": 1.0, "KB": 1024.0, "MB": 1024.0 ** 2, "GB": 1024.0 ** 3}
    return float(match.group(1)) * scale.get(match.group(2), 1.0)

def hz_score(value):
    try:
        return float(value)
    except ValueError:
        return -1.0

def short_nodes(endpoint_infos):
    names = []
    for endpoint in endpoint_infos:
        node_name = getattr(endpoint, "node_name", "")
        node_ns = getattr(endpoint, "node_namespace", "")
        if not node_name:
            continue
        if node_ns and node_ns != "/":
            full_name = f"{node_ns.rstrip('/')}/{node_name}"
        else:
            full_name = node_name
        if full_name not in names:
            names.append(full_name)
    if not names:
        return "-"
    if len(names) > 3:
        return ", ".join(names[:3]) + f", +{len(names) - 3}"
    return ", ".join(names)

rows = []
elapsed = 0.0
node = None

if topics or watch_new:
    try:
        rclpy.init()
        node = Node("topic_audit_table_sampler")

        types_by_topic = {}
        discovery_deadline = time.monotonic() + discovery_s
        while True:
            types_by_topic = {name: types for name, types in node.get_topic_names_and_types()}
            if watch_new or not topics or all(topic in types_by_topic for topic in topics):
                break
            if time.monotonic() >= discovery_deadline:
                break
            rclpy.spin_once(node, timeout_sec=0.1)
        stats = defaultdict(lambda: {"count": 0, "bytes": 0})
        subscriptions = {}

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        def make_callback(topic):
            def callback(raw):
                stats[topic]["count"] += 1
                stats[topic]["bytes"] += len(raw) if hasattr(raw, "__len__") else 0
            return callback

        def ignored_topic(topic):
            return any(pattern.search(topic) for pattern in ignore_patterns)

        def subscribe_topic(topic):
            if topic in subscriptions or ignored_topic(topic):
                return
            if max_topics and len(subscriptions) >= max_topics:
                return
            type_names = types_by_topic.get(topic, [])
            if not type_names:
                return
            try:
                msg_type = get_message(type_names[0])
                subscriptions[topic] = node.create_subscription(
                    msg_type,
                    topic,
                    make_callback(topic),
                    qos,
                    raw=True,
                )
            except Exception as exc:
                stats[topic]["error"] = type(exc).__name__

        for topic in topics:
            subscribe_topic(topic)

        start = time.monotonic()
        deadline = start + sample_s
        next_watch = start
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
            now = time.monotonic()
            if watch_new and now >= next_watch:
                types_by_topic = {name: types for name, types in node.get_topic_names_and_types()}
                for topic in sorted(types_by_topic):
                    if not topic.startswith("/"):
                        continue
                    subscribe_topic(topic)
                next_watch = now + watch_interval_s
            if not watch_new and subscriptions and all(stats[topic]["count"] >= samples for topic in subscriptions):
                break
        elapsed = max(1e-6, time.monotonic() - start)

        visited = set()
        if visited_path.exists():
            visited = {line.strip() for line in visited_path.read_text(encoding="utf-8").splitlines() if line.strip()}

        sampled_topics = sorted(set(topics) | set(subscriptions))
        for topic in sampled_topics:
            type_names = types_by_topic.get(topic, [])
            topic_type = type_names[0] if type_names else "unknown"
            count = int(stats[topic]["count"])
            total_bytes = int(stats[topic]["bytes"])
            hz = f"{count / elapsed:.3f}" if count > 0 else "silent"
            bw = human_bw(total_bytes / elapsed) if count > 0 else "silent"
            pub_count = node.count_publishers(topic)
            sub_count = node.count_subscribers(topic)
            pub_nodes = short_nodes(node.get_publishers_info_by_topic(topic))
            sub_nodes_all = short_nodes(node.get_subscriptions_info_by_topic(topic))
            if topic in subscriptions and sub_count > 0:
                sub_count -= 1
            sub_nodes = ", ".join(
                name
                for name in sub_nodes_all.split(", ")
                if name not in ("-", "topic_audit_table_sampler")
            ) or "-"
            if count > 0 or (mark_silent and count == 0):
                visited.add(topic)
            rows.append(
                {
                    "topic": topic,
                    "hz": hz,
                    "bw": bw,
                    "pub": str(pub_count),
                    "sub": str(sub_count),
                    "pub_nodes": pub_nodes,
                    "sub_nodes": sub_nodes,
                    "info": topic_type,
                }
            )

        visited_path.write_text("\n".join(sorted(visited)) + ("\n" if visited else ""), encoding="utf-8")
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

with open(rows_path, "w", encoding="utf-8", newline="") as handle:
    writer = csv.DictWriter(
        handle,
        fieldnames=["topic", "hz", "bw", "pub", "sub", "pub_nodes", "sub_nodes", "info"],
        delimiter="\t",
    )
    writer.writeheader()
    writer.writerows(rows)

def empty_record(topic):
    return {
        "topic": topic,
        "hz": "-",
        "bw": "-",
        "peak_hz": "-",
        "peak_bw": "-",
        "info": "",
        **{column: "-" for column in mode_columns},
    }

def endpoint_text(pub_count="-", pub_nodes="-", sub_count="-", sub_nodes="-"):
    def clean(value):
        text = str(value).strip()
        if text in ("", "nan", "None"):
            return "-"
        if re.fullmatch(r"\d+\.0", text):
            return text[:-2]
        return text
    return f"pub {clean(pub_count)} [{clean(pub_nodes)}], sub {clean(sub_count)} [{clean(sub_nodes)}]"

def split_endpoint_text(value):
    text = str(value).strip()
    match = re.fullmatch(r"pub\s+([^\[]+)\s+\[(.*?)\],\s+sub\s+([^\[]+)\s+\[(.*?)\]", text)
    if not match:
        return "-", "-", "-", "-"
    return tuple(part.strip() or "-" for part in match.groups())

def load_existing_from_csv(path):
    loaded = {}
    if not path.exists():
        return loaded
    df = pd.read_csv(path, keep_default_na=False)
    for _, row in df.iterrows():
        topic = str(row.get("topic", "")).strip()
        if not topic:
            continue
        record = empty_record(topic)
        record["hz"] = str(row.get("hz", "-")).strip() or "-"
        record["bw"] = str(row.get("bw", "-")).strip() or "-"
        record["peak_hz"] = str(row.get("peak_hz", "-")).strip() or "-"
        record["peak_bw"] = str(row.get("peak_bw", "-")).strip() or "-"
        note = str(row.get("note", "")).strip()
        if note and note != "-":
            hz_match = re.search(r"peak hz ([^:]+): ([^;]+?)(?: >|$)", note)
            bw_match = re.search(r"peak bw ([^:]+): ([^;]+?)(?: >|$)", note)
            if hz_match and record["peak_hz"] == "-":
                record["peak_hz"] = f"{hz_match.group(2).strip()} ({mode_labels.get(hz_match.group(1).strip(), hz_match.group(1).strip().upper())})"
            if bw_match and record["peak_bw"] == "-":
                record["peak_bw"] = f"{bw_match.group(2).strip()} ({mode_labels.get(bw_match.group(1).strip(), bw_match.group(1).strip().upper())})"
        record["info"] = str(row.get("info", "")).strip()
        for column in mode_columns:
            if column in row and str(row.get(column, "")).strip():
                record[column] = str(row.get(column)).strip()
            else:
                pub_count = row.get(f"{column}_pub_count", "-")
                pub_nodes = row.get(f"{column}_pub_nodes", "-")
                sub_count = row.get(f"{column}_sub_count", "-")
                sub_nodes = row.get(f"{column}_sub_nodes", "-")
                if any(str(value).strip() not in ("", "-", "nan") for value in (pub_count, pub_nodes, sub_count, sub_nodes)):
                    record[column] = endpoint_text(pub_count, pub_nodes, sub_count, sub_nodes)
        loaded[topic] = record
    return loaded

def load_existing_from_markdown(path):
    loaded = {}
    if not path.exists():
        return loaded
    for line in out.read_text(encoding="utf-8").splitlines():
        if not line.startswith("| /"):
            continue
        parts = [part.strip() for part in line.strip().strip("|").split("|")]
        if len(parts) < 6:
            continue
        record = empty_record(parts[0])
        record["hz"] = parts[1]
        record["bw"] = parts[2]
        record["info"] = parts[-1]
        if len(parts) >= 3 + len(mode_columns) + 2:
            record["peak_hz"] = parts[-3]
            record["peak_bw"] = parts[-2]
            for index, column in enumerate(mode_columns, start=3):
                record[column] = parts[index]
        elif len(parts) >= 3 + len(mode_columns) + 1:
            for index, column in enumerate(mode_columns, start=3):
                record[column] = parts[index]
        else:
            record["follow"] = parts[3]
            record["yolo"] = parts[4]
        loaded[parts[0]] = record
    return loaded

existing = load_existing_from_csv(csv_out) or load_existing_from_markdown(out)

def valued(value):
    return value.strip() not in ("", "-", "silent", "unknown", "?")

for row in rows:
    record = existing.get(
        row["topic"],
        empty_record(row["topic"]),
    )
    if valued(row["hz"]) and hz_score(row["hz"]) > hz_score(record["hz"]):
        record["peak_hz"] = f"{row['hz']} ({mode_labels.get(mode, mode.upper())})"
        record["hz"] = row["hz"]
    elif not valued(record["hz"]):
        record["hz"] = row["hz"]
        if valued(row["hz"]) and not valued(record.get("peak_hz", "-")):
            record["peak_hz"] = f"{row['hz']} ({mode_labels.get(mode, mode.upper())})"
    if valued(row["bw"]) and bw_score(row["bw"]) > bw_score(record["bw"]):
        record["peak_bw"] = f"{row['bw']} ({mode_labels.get(mode, mode.upper())})"
        record["bw"] = row["bw"]
    elif not valued(record["bw"]):
        record["bw"] = row["bw"]
        if valued(row["bw"]) and not valued(record.get("peak_bw", "-")):
            record["peak_bw"] = f"{row['bw']} ({mode_labels.get(mode, mode.upper())})"
    count_text = f"pub {row['pub']} [{row.get('pub_nodes', '-')}], sub {row['sub']} [{row.get('sub_nodes', '-')}]"
    record[mode] = count_text
    if row["info"] != "unknown" or not valued(record["info"]):
        record["info"] = row["info"]
    existing[row["topic"]] = record

records = sorted(existing.values(), key=lambda item: (bw_score(item["bw"]), hz_score(item["hz"]), item["topic"]), reverse=True)

def has_mode(value):
    return value.strip() not in ("", "-")

def active_modes(item):
    return [column for column in mode_columns if has_mode(item[column])]

def is_silent(item):
    return str(item.get("hz", "")).strip() == "silent" and str(item.get("bw", "")).strip() == "silent"

active_records = [item for item in records if not is_silent(item)]
silent_records = [item for item in records if is_silent(item)]

groups = [
    ("Follow and YOLO family", [item for item in active_records if "follow" in active_modes(item) and any(column.startswith("yolo") or column == "support_yolo" for column in active_modes(item))]),
    ("Follow only", [item for item in active_records if active_modes(item) == ["follow"]]),
    ("YOLO family only", [item for item in active_records if active_modes(item) and "follow" not in active_modes(item)]),
    ("No mode column", [item for item in active_records if not active_modes(item)]),
    ("Silent topics", silent_records),
]

csv_rows = []
csv_columns = ["group", "topic", "hz", "bw", "peak_hz", "peak_bw", "modes", "info"]
for column in mode_columns:
    csv_columns.extend([
        f"{column}_pub_count",
        f"{column}_pub_nodes",
        f"{column}_sub_count",
        f"{column}_sub_nodes",
    ])
for item in records:
    active = active_modes(item)
    csv_row = {
            "group": next((name for name, items in groups if item in items), ""),
        "topic": item["topic"],
        "hz": item["hz"],
        "bw": item["bw"],
        "peak_hz": item.get("peak_hz", "-"),
        "peak_bw": item.get("peak_bw", "-"),
        "modes": ", ".join(mode_labels[column] for column in active) if active else "-",
        "info": item["info"],
    }
    for column in mode_columns:
        pub_count, pub_nodes, sub_count, sub_nodes = split_endpoint_text(item[column])
        csv_row[f"{column}_pub_count"] = pub_count
        csv_row[f"{column}_pub_nodes"] = pub_nodes
        csv_row[f"{column}_sub_count"] = sub_count
        csv_row[f"{column}_sub_nodes"] = sub_nodes
    csv_rows.append(csv_row)
csv_df = pd.DataFrame(csv_rows, columns=csv_columns)
csv_df.to_csv(csv_out, index=False)

lines = [
    "<!-- Generated by ./run.sh topic_audit_table. Full detail table: "
    + csv_out.name
    + " -->",
]

def markdown_table(df):
    columns = ["topic", "hz", "bw", "peak_hz", "peak_bw", "modes", "info"]
    aligns = {
        "hz": "---:",
        "bw": "---:",
    }
    lines_out = [
        "| " + " | ".join(columns) + " |",
        "| " + " | ".join(aligns.get(column, "---") for column in columns) + " |",
    ]
    for _, row in df.iterrows():
        values = []
        for column in columns:
            value = str(row.get(column, "-")).replace("|", "\\|").replace("\n", " ")
            values.append(value if value else "-")
        lines_out.append("| " + " | ".join(values) + " |")
    return lines_out

summary = ", ".join(f"{name.lower().replace(' ', '_')}={len(items)}" for name, items in groups if items)
lines.append(f"Summary: {summary}")
for name, _items in groups:
    group_df = csv_df[csv_df["group"] == name]
    if group_df.empty:
        continue
    lines.extend(
        [
            "",
            f"## {name}",
        ]
    )
    lines.extend(markdown_table(group_df))
out.write_text("\n".join(lines) + "\n", encoding="utf-8")

topics_with_messages = sum(1 for row in rows if row["hz"] != "silent")
silent_topics = sum(1 for row in rows if row["hz"] == "silent")
print(
    f"[topic_audit_table] sampled {len(topics)} topics for {elapsed:.2f}s, "
    f"topics_with_messages={topics_with_messages}, silent_topics={silent_topics}, target={samples} messages/topic"
)
print(f"[topic_audit_table] wrote {out}")
print(f"[topic_audit_table] wrote {csv_out}")
PY

if [ "$APPEND_VALUED_TO_IGNORE" = "true" ] && [ -s "$VISITED" ]; then
  touch "$IGNORE"
  while IFS= read -r topic; do
    [ -n "$topic" ] || continue
    printf '^%s$\n' "$topic"
  done < "$VISITED" >> "$IGNORE"
  sort -u -o "$IGNORE" "$IGNORE"
  echo "[topic_audit_table] appended valued topics to $IGNORE"
fi
