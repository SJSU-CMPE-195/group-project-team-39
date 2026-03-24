"""
Polynomial-regression trajectory predictor.

Buffers the last N detected positions and fits a polynomial to estimate:
  - Current velocity (1st derivative of the fit at the latest time step)
  - Future positions (extrapolated curve)
  - First wall-collision point (walking along the curve)

Advantages over the original 2-point linear method
----------------------------------------------------
* Noise reduction — averages over many frames, not just p1-p0.
* Handles curved motion — a degree-2 polynomial captures spin or arc.
* Smoother direction arrow — velocity is a polynomial derivative, not a raw diff.

Falls back to linear prediction automatically when fewer than 3 points are
available (same behaviour as the original code).
"""

import numpy as np
from collections import deque


class TrajectoryPredictor:
    """
    Args:
        history_len: number of past positions to keep (ring buffer)
        poly_degree:  polynomial degree for the fit (2 = quadratic / parabola)
    """

    def __init__(self, history_len: int = 12, poly_degree: int = 2):
        self.poly_degree = poly_degree
        self._positions: deque[tuple[int, int]] = deque(maxlen=history_len)
        self._t = 0  # monotonically increasing frame index

    # ── Data ingestion ────────────────────────────────────────────────────────

    def update(self, cx: int, cy: int) -> None:
        """Record a new (smoothed) position."""
        self._positions.append((cx, cy))
        self._t += 1

    def reset(self) -> None:
        self._positions.clear()
        self._t = 0

    @property
    def n(self) -> int:
        return len(self._positions)

    # ── Polynomial helpers ────────────────────────────────────────────────────

    def _fit(self):
        """Return (ts, px_coeffs, py_coeffs) with ts normalised so last point = 0."""
        pos = list(self._positions)
        n = len(pos)
        ts = np.arange(n, dtype=float) - (n - 1)   # last point is t=0
        xs = np.array([p[0] for p in pos], dtype=float)
        ys = np.array([p[1] for p in pos], dtype=float)
        degree = min(self.poly_degree, n - 1)
        px = np.polyfit(ts, xs, degree)
        py = np.polyfit(ts, ys, degree)
        return ts, px, py

    # ── Public API ────────────────────────────────────────────────────────────

    def predicted_velocity(self) -> tuple[float, float]:
        """
        Smoothed velocity (vx, vy) at the most recent position.

        Uses the derivative of the polynomial fit evaluated at t=0 (current
        position). Much less noisy than the raw p1-p0 difference.
        """
        if self.n < 2:
            return (0.0, 0.0)

        _, px, py = self._fit()
        vx = float(np.polyval(np.polyder(px), 0.0))
        vy = float(np.polyval(np.polyder(py), 0.0))
        return (vx, vy)

    def predict_path(self, n_steps: int = 120) -> list[tuple[int, int]]:
        """
        Predict the next n_steps positions along the polynomial curve.
        Returns a list of (x, y) integer tuples.
        """
        if self.n < 2:
            return []

        _, px, py = self._fit()
        future_ts = np.arange(1, n_steps + 1, dtype=float)
        xs = np.polyval(px, future_ts).astype(int)
        ys = np.polyval(py, future_ts).astype(int)
        return list(zip(xs.tolist(), ys.tolist()))

    def predict_border_hit(
        self, w: int, h: int, max_steps: int = 400
    ) -> tuple[tuple[int, int] | None, str | None]:
        """
        Walk along the predicted polynomial path to find the first wall hit.

        Returns:
            (hit_point, wall_name)  where wall_name ∈ {"left","right","top","bottom"}
            (None, None)            if no hit within max_steps

        This is the AI-enhanced replacement for first_border_intersection().
        Unlike linear ray-casting, it follows the actual predicted curve.
        After the first hit, the caller can still apply the original linear
        reflect() logic for the bounce (history doesn't yet contain post-bounce
        data, so the polynomial can't predict it).
        """
        if self.n < 2:
            return None, None

        path = self.predict_path(n_steps=max_steps)
        if not path:
            return None, None

        prev_x = float(self._positions[-1][0])
        prev_y = float(self._positions[-1][1])

        for pt in path:
            fx, fy = float(pt[0]), float(pt[1])

            dx = fx - prev_x
            dy = fy - prev_y

            # Left wall
            if fx <= 0:
                t = (0 - prev_x) / dx if abs(dx) > 1e-9 else 0.0
                iy = int(np.clip(prev_y + t * dy, 0, h - 1))
                return (0, iy), "left"

            # Right wall
            if fx >= w - 1:
                t = ((w - 1) - prev_x) / dx if abs(dx) > 1e-9 else 0.0
                iy = int(np.clip(prev_y + t * dy, 0, h - 1))
                return (w - 1, iy), "right"

            # Top wall
            if fy <= 0:
                t = (0 - prev_y) / dy if abs(dy) > 1e-9 else 0.0
                ix = int(np.clip(prev_x + t * dx, 0, w - 1))
                return (ix, 0), "top"

            # Bottom wall
            if fy >= h - 1:
                t = ((h - 1) - prev_y) / dy if abs(dy) > 1e-9 else 0.0
                ix = int(np.clip(prev_x + t * dx, 0, w - 1))
                return (ix, h - 1), "bottom"

            prev_x, prev_y = fx, fy

        return None, None
