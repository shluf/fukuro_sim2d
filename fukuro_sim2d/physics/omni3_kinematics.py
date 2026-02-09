"""
omni3_kinematics.py — Forward & Inverse kinematics untuk robot omniwheel 3 roda.

Semua kalkulasi dalam world frame ROS:
    vx  = kecepatan maju (+X robot)
    vy  = kecepatan lateral (+Y robot, ke kiri)
    ω   = kecepatan angular (CCW positif)

Konvensi sudut roda:
    Sudut roda diukur dari sumbu +X robot (CCW positif).
    Default 3 roda: 0°, 120°, 240°.
"""

import numpy as np


class Omni3Kinematics:
    """Kinematics model untuk 3-wheeled omnidirectional robot."""

    def __init__(self, R: float, r: float,
                 wheel_angles_deg: list[float] | None = None):
        """
        Parameters
        ----------
        R : float
            Jarak dari pusat robot ke roda (meter).
        r : float
            Radius roda (meter).
        wheel_angles_deg : list[float] | None
            Sudut masing-masing roda dalam derajat dari sumbu +X robot.
            Default: [0, 120, 240].
        """
        self.R = R
        self.r = r

        if wheel_angles_deg is None:
            wheel_angles_deg = [0.0, 120.0, 240.0]

        self.wheel_angles = np.radians(wheel_angles_deg)
        self._build_matrices()

    def _build_matrices(self):
        """Bangun matriks kinematika G (inverse) dan G_inv (forward)."""
        rows = []
        for alpha in self.wheel_angles:
            # ω_wheel = (-sin(α)*vx + cos(α)*vy + R*ω) / r
            rows.append([-np.sin(alpha), np.cos(alpha), self.R])
        self.G = np.array(rows) / self.r            # (3×3)
        self.G_inv = np.linalg.pinv(self.G)          # (3×3)

    # ------------------------------------------------------------------

    def inverse(self, vx: float, vy: float, omega: float) -> np.ndarray:
        """Hitung kecepatan angular tiap roda dari body velocity.

        Parameters
        ----------
        vx : float   — kecepatan maju (m/s)
        vy : float   — kecepatan lateral (m/s)
        omega : float — kecepatan angular (rad/s)

        Returns
        -------
        np.ndarray shape (3,) — kecepatan angular tiap roda (rad/s)
        """
        v = np.array([vx, vy, omega])
        return self.G @ v

    def forward(self, wheel_speeds: np.ndarray) -> tuple[float, float, float]:
        """Hitung body velocity dari kecepatan angular tiap roda.

        Parameters
        ----------
        wheel_speeds : array-like shape (3,) — ω tiap roda (rad/s)

        Returns
        -------
        (vx, vy, omega) dalam frame robot.
        """
        w = np.asarray(wheel_speeds)
        v = self.G_inv @ w
        return float(v[0]), float(v[1]), float(v[2])
