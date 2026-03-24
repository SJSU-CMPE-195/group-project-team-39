"""
Kalman Filter tracker for 2D puck position.

State vector: [x, y, vx, vy]
Observation:  [x, y]

Replaces the raw 2-point velocity estimate (p1 - p0) with a proper
state estimator that handles noisy detections and brief occlusions.
"""

import numpy as np


class KalmanPuckTracker:
    """
    4-state Kalman filter: position + velocity in 2D.

    Args:
        process_noise: Q scale — higher = trust detections more, less smooth
        meas_noise:    R scale — higher = trust model more, smoother but laggier
        dt:            time step (1.0 = one frame)
    """

    def __init__(self, process_noise: float = 1.0, meas_noise: float = 10.0, dt: float = 1.0):
        self.dt = dt

        # State transition: constant-velocity model
        self.F = np.array([
            [1, 0, dt,  0],
            [0, 1,  0, dt],
            [0, 0,  1,  0],
            [0, 0,  0,  1],
        ], dtype=float)

        # Observation matrix (we observe x, y only)
        self.H = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0],
        ], dtype=float)

        # Process noise covariance
        self.Q = np.eye(4, dtype=float) * process_noise

        # Measurement noise covariance
        self.R = np.eye(2, dtype=float) * meas_noise

        self._x: np.ndarray | None = None          # state [x, y, vx, vy]
        self._P: np.ndarray = np.eye(4) * 500.0    # covariance
        self.initialized = False

    # ── Public API ────────────────────────────────────────────────────────────

    def init(self, x: float, y: float) -> None:
        """Seed the filter with a known starting position."""
        self._x = np.array([x, y, 0.0, 0.0], dtype=float)
        self._P = np.eye(4, dtype=float) * 500.0
        self.initialized = True

    def update(self, x: float, y: float) -> tuple[int, int]:
        """
        Feed a new measurement.  If not yet initialized, initializes first.
        Returns the smoothed (x, y) position.
        """
        if not self.initialized:
            self.init(x, y)
            return self.position

        # Predict step
        self._x = self.F @ self._x
        self._P = self.F @ self._P @ self.F.T + self.Q

        # Update step
        z = np.array([x, y], dtype=float)
        y_inn = z - self.H @ self._x
        S = self.H @ self._P @ self.H.T + self.R
        K = self._P @ self.H.T @ np.linalg.inv(S)
        self._x = self._x + K @ y_inn
        self._P = (np.eye(4) - K @ self.H) @ self._P

        return self.position

    def predict(self) -> tuple[int, int] | None:
        """
        Advance one step with no measurement (handles brief occlusion).
        Returns predicted (x, y) or None if not initialized.
        """
        if not self.initialized:
            return None
        self._x = self.F @ self._x
        self._P = self.F @ self._P @ self.F.T + self.Q
        return self.position

    def predict_future(self, n_steps: int) -> tuple[int, int] | None:
        """Return predicted position n_steps ahead without mutating state."""
        if not self.initialized:
            return None
        x_pred = self._x.copy()
        for _ in range(n_steps):
            x_pred = self.F @ x_pred
        return (int(x_pred[0]), int(x_pred[1]))

    def reset(self) -> None:
        self._x = None
        self._P = np.eye(4, dtype=float) * 500.0
        self.initialized = False

    # ── Properties ────────────────────────────────────────────────────────────

    @property
    def position(self) -> tuple[int, int] | None:
        if not self.initialized or self._x is None:
            return None
        return (int(self._x[0]), int(self._x[1]))

    @property
    def velocity(self) -> tuple[float, float]:
        """Smoothed (vx, vy) from the Kalman state — far less noisy than p1-p0."""
        if not self.initialized or self._x is None:
            return (0.0, 0.0)
        return (float(self._x[2]), float(self._x[3]))
