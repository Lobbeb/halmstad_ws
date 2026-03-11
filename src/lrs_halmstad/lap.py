"""Minimal scipy-backed compatibility shim for Ultralytics tracker imports.

Ultralytics track() imports the external ``lap`` package and uses only
``lap.lapjv(...)``. Some development machines in this workspace do not have the
compiled ``lap`` wheel installed, but they do have SciPy. This shim provides
the small subset of the API that Ultralytics needs so tracker mode can run
offline.
"""

from __future__ import annotations

from typing import Tuple

import numpy as np
from scipy.optimize import linear_sum_assignment

__version__ = "0.5.12-scipy"


def _empty_assignment(rows: int, cols: int, return_cost: bool):
    x = np.full(rows, -1, dtype=int)
    y = np.full(cols, -1, dtype=int)
    if return_cost:
        return 0.0, x, y
    return x, y


def lapjv(
    cost_matrix,
    extend_cost: bool = True,
    cost_limit: float = np.inf,
    return_cost: bool = True,
):
    """Approximate ``lap.lapjv`` using SciPy's Hungarian solver.

    The Ultralytics tracker calls this with ``extend_cost=True`` and a finite
    ``cost_limit``. We model unmatched rows/columns by expanding the assignment
    matrix with dummy rows/columns carrying that limit as the unmatched cost.
    """

    cost = np.asarray(cost_matrix, dtype=np.float64)
    if cost.ndim != 2:
        raise ValueError("cost_matrix must be 2-dimensional")

    rows, cols = cost.shape
    if rows == 0 or cols == 0:
        return _empty_assignment(rows, cols, return_cost)

    finite = cost[np.isfinite(cost)]
    max_finite = float(np.max(finite)) if finite.size else 1.0

    if np.isfinite(cost_limit):
        unmatched_cost = float(cost_limit)
    else:
        unmatched_cost = max_finite + 1.0

    blocked_cost = max(max_finite, unmatched_cost, 1.0) * 1e6
    safe_cost = np.where(np.isfinite(cost), cost, blocked_cost)

    if not extend_cost and rows == cols:
        row_ind, col_ind = linear_sum_assignment(safe_cost)
        x = np.full(rows, -1, dtype=int)
        y = np.full(cols, -1, dtype=int)
        total = 0.0
        for row, col in zip(row_ind, col_ind):
            value = float(safe_cost[row, col])
            if value <= unmatched_cost:
                x[row] = col
                y[col] = row
                total += value
        if return_cost:
            return total, x, y
        return x, y

    total_size = rows + cols
    expanded = np.full((total_size, total_size), blocked_cost, dtype=np.float64)
    expanded[:rows, :cols] = safe_cost

    # Allow each real row to go unmatched against its own dummy column.
    for row in range(rows):
        expanded[row, cols + row] = unmatched_cost

    # Allow each real column to go unmatched against its own dummy row.
    for col in range(cols):
        expanded[rows + col, col] = unmatched_cost

    # Dummy-to-dummy assignments close the square system at zero cost.
    expanded[rows:, cols:] = 0.0

    row_ind, col_ind = linear_sum_assignment(expanded)
    x = np.full(rows, -1, dtype=int)
    y = np.full(cols, -1, dtype=int)
    total = 0.0

    for row, col in zip(row_ind, col_ind):
        if row < rows and col < cols:
            value = float(safe_cost[row, col])
            if value <= unmatched_cost:
                x[row] = col
                y[col] = row
                total += value

    if return_cost:
        return total, x, y
    return x, y
