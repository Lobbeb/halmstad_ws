#!/usr/bin/env python3
from __future__ import annotations

import json
import sys
from pathlib import Path
from typing import List, Dict, Any

import pandas as pd


def main() -> int:
    if len(sys.argv) != 2:
        print("Usage: analyze_batch.py <runs_root_or_batch_dir>")
        return 2

    root = Path(sys.argv[1]).expanduser().resolve()
    if not root.exists():
        print(f"ERROR: missing {root}")
        return 2

    run_dirs: List[Path] = []
    if (root / "meta.yaml").exists():
        run_dirs = [root]
    else:
        run_dirs = [p for p in root.rglob("*") if (p / "meta.yaml").exists() and (p / "bag").exists()]

    rows: List[Dict[str, Any]] = []
    for rd in sorted(run_dirs):
        results_path = rd / "results.json"
        meta_path = rd / "meta.yaml"
        meta = {}
        try:
            import yaml
            meta = yaml.safe_load(meta_path.read_text()) or {}
        except Exception:
            pass

        if results_path.exists():
            data = json.loads(results_path.read_text())
        else:
            data = {"notes": "results.json missing; run analyze_run.py first"}

        row = {
            "run_dir": str(rd),
            "run_id": meta.get("run_id"),
            "timestamp": meta.get("timestamp"),
            "condition": meta.get("condition"),
            "world": meta.get("world"),
            "uav_name": meta.get("uav_name"),
        }
        row.update({f"res_{k}": v for k, v in data.items() if k not in row})
        rows.append(row)

    df = pd.DataFrame(rows)
    out_csv = root / "results.csv"
    df.to_csv(out_csv, index=False)

    summary = {
        "num_runs": len(df),
        "conditions": sorted(df["condition"].dropna().unique().tolist()) if "condition" in df else [],
    }
    out_json = root / "batch_summary.json"
    out_json.write_text(json.dumps(summary, indent=2))

    print(df.head(10).to_string(index=False))
    print(f"✅ wrote {out_csv}")
    print(f"✅ wrote {out_json}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
