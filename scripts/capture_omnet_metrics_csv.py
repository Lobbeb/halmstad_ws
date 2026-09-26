#!/usr/bin/env python3
"""Capture OMNeT metrics TCP lines to CSV."""

from __future__ import annotations

import argparse
import csv
import socket
import time
from pathlib import Path


HEADER = [
    "wall_time_s",
    "sim_time_s",
    "link_distance_m",
    "rssi_dbm",
    "snir_db",
    "packet_error_rate",
    "radio_distance_m",
    "packet_delivery_ratio",
    "latency_s",
    "jitter_s",
]


def parse_line(line: str) -> list[float] | None:
    parts = line.split()
    if len(parts) not in (6, 8, 9):
        return None
    try:
        values = [float(part) for part in parts]
    except ValueError:
        return None
    if len(values) == 9:
        return values
    if len(values) == 8:
        sim_time, rssi, snir, per, radio_dist, pdr, latency, jitter = values
        return [sim_time, float("nan"), rssi, snir, per, radio_dist, pdr, latency, jitter]
    sim_time, link_dist, rssi, snir, per, radio_dist = values
    return [sim_time, link_dist, rssi, snir, per, radio_dist, float("nan"), float("nan"), float("nan")]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=5556)
    parser.add_argument("--out", required=True)
    parser.add_argument("--connect-timeout-s", type=float, default=120.0)
    parser.add_argument("--retry-s", type=float, default=0.25)
    return parser.parse_args()


def connect_with_retry(host: str, port: int, timeout_s: float, retry_s: float) -> socket.socket:
    deadline = time.monotonic() + timeout_s
    last_error: Exception | None = None
    while time.monotonic() < deadline:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            sock.connect((host, port))
            sock.settimeout(5.0)
            return sock
        except OSError as exc:
            last_error = exc
            sock.close()
            time.sleep(retry_s)
    raise SystemExit(f"Could not connect to OMNeT metrics server at {host}:{port}: {last_error}")


def main() -> None:
    args = parse_args()
    out = Path(args.out).expanduser().resolve()
    out.parent.mkdir(parents=True, exist_ok=True)

    sock = connect_with_retry(args.host, args.port, args.connect_timeout_s, args.retry_s)
    start = time.monotonic()
    with sock, out.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(HEADER)
        handle.flush()
        buf = b""
        while True:
            chunk = sock.recv(512)
            if not chunk:
                break
            buf += chunk
            while b"\n" in buf:
                raw, buf = buf.split(b"\n", 1)
                line = raw.decode("ascii", errors="ignore").strip()
                values = parse_line(line)
                if values is None:
                    continue
                writer.writerow([f"{time.monotonic() - start:.6f}", *values])
                handle.flush()


if __name__ == "__main__":
    main()
