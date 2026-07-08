"""
robot_model.py — Model fisika robot (pure physics, tanpa Pygame/ROS).

Semua state dalam world frame ROS-compliant:
    x, y   : posisi (meter)
    theta   : heading (rad), 0 = +X, positif CCW
    vx, vy  : body velocity (m/s) — dalam frame robot
    omega   : angular velocity (rad/s)

Persamaan update (global displacement dari body velocity):
    dx = (vx·cosθ − vy·sinθ) · Ts
    dy = (vx·sinθ + vy·cosθ) · Ts
    dθ = ω · Ts
"""

import math

import numpy as np

from fukuro_sim2d.physics.omni3_kinematics import Omni3Kinematics


class RobotModel:
    """Physics model untuk satu robot omniwheel."""

    def __init__(
        self,
        x: float,
        y: float,
        theta: float,
        radius: float = 0.2,
        wheel_R: float = 0.5,
        wheel_r: float = 0.05,
        wheel_angles_deg: list[float] | None = None,
        mass: float = 20.0,
        friction: float = 0.98,
        max_linear_speed: float = 2.5,
        max_angular_speed: float = 4.0,
        max_linear_accel: float = 3.0,
        max_angular_accel: float = 8.0,
    ):
        """
        Parameters
        ----------
        x, y : float
            Posisi awal dalam world frame (meter).
        theta : float
            Heading awal (radian), 0 = menghadap +X.
        radius : float
            Radius badan robot (meter), untuk collision & rendering.
        wheel_R : float
            Jarak pusat robot ke roda (meter).
        wheel_r : float
            Radius roda (meter).
        wheel_angles_deg : list[float] | None
            Sudut mounting roda (derajat). Default [0, 120, 240].
        mass : float
            Massa robot (kg), untuk collision physics.
        friction : float
            Koefisien redam per frame (0–1). Setiap frame: velocity *= friction.
        """
        # --- State ---
        self.x = x
        self.y = y
        self.theta = theta

        # --- Body velocity (robot frame) ---
        self.vx = 0.0  # m/s  maju
        self.vy = 0.0  # m/s  lateral
        self.omega = 0.0  # rad/s

        # Target body velocity dari controller/strategy. Robot fisik tidak
        # meloncat instan ke target, jadi update() menerapkan limit akselerasi.
        self.target_vx = 0.0
        self.target_vy = 0.0
        self.target_omega = 0.0
        self.max_linear_speed = max_linear_speed
        self.max_angular_speed = max_angular_speed
        self.max_linear_accel = max_linear_accel
        self.max_angular_accel = max_angular_accel

        # --- Physical properties ---
        self.radius = radius
        self.mass = mass
        self.friction = friction

        # --- Kinematics ---
        self.kinematics = Omni3Kinematics(wheel_R, wheel_r, wheel_angles_deg)
        self.omega_wheels = np.zeros(3)

        # --- Dribbler & grip ---
        self.dribbler_active = False
        self.dribbler_pwm = 0
        self.is_gripped = False

        # --- Ready state ---
        self.is_ready = False

    # ------------------------------------------------------------------
    # Physics update
    # ------------------------------------------------------------------

    def set_velocity(self, vx: float, vy: float, omega: float):
        """Set target body velocity command (m/s, rad/s)."""
        linear_norm = math.hypot(vx, vy)
        if linear_norm > self.max_linear_speed and linear_norm > 1e-9:
            scale = self.max_linear_speed / linear_norm
            vx *= scale
            vy *= scale
        self.target_vx = vx
        self.target_vy = vy
        self.target_omega = max(
            -self.max_angular_speed, min(self.max_angular_speed, omega)
        )

    def reset_velocity(self):
        """Emergency stop: clear target and actual velocity immediately."""
        self.target_vx = self.target_vy = self.target_omega = 0.0
        self.vx = self.vy = self.omega = 0.0
        self.omega_wheels = np.zeros(3)

    @staticmethod
    def _approach(current: float, target: float, max_delta: float) -> float:
        if current < target:
            return min(current + max_delta, target)
        if current > target:
            return max(current - max_delta, target)
        return current

    def update(self, Ts: float):
        """Advance state satu timestep.

        Parameters
        ----------
        Ts : float   — periode sampling (detik)
        """
        # Acceleration-limited response toward target command.
        max_linear_delta = self.max_linear_accel * Ts
        max_angular_delta = self.max_angular_accel * Ts
        self.vx = self._approach(self.vx, self.target_vx, max_linear_delta)
        self.vy = self._approach(self.vy, self.target_vy, max_linear_delta)
        self.omega = self._approach(self.omega, self.target_omega, max_angular_delta)

        # Small floor-contact/rolling loss, like real omni wheels under load.
        self.vx *= self.friction
        self.vy *= self.friction
        self.omega *= self.friction

        cos_th = math.cos(self.theta)
        sin_th = math.sin(self.theta)

        # Global displacement
        self.x += (self.vx * cos_th - self.vy * sin_th) * Ts
        self.y += (self.vx * sin_th + self.vy * cos_th) * Ts
        self.theta += self.omega * Ts  # CCW positif (standar ROS)

        # Normalize theta ke [0, 2π)
        self.theta = self.theta % (2 * math.pi)

        # Update wheel speeds (for visualization / debug)
        self.omega_wheels = self.kinematics.inverse(self.vx, self.vy, self.omega)

    # ------------------------------------------------------------------
    # Dribbler helpers
    # ------------------------------------------------------------------

    def activate_dribbler(self, pwm: int):
        self.dribbler_active = True
        self.dribbler_pwm = pwm

    def deactivate_dribbler(self):
        self.dribbler_active = False
        self.dribbler_pwm = 0

    # ------------------------------------------------------------------
    # Getter untuk posisi "depan" robot (mulut dribbler)
    # ------------------------------------------------------------------

    def front_position(self) -> tuple[float, float]:
        """Posisi titik depan robot (world coords)."""
        fx = self.x + self.radius * math.cos(self.theta)
        fy = self.y + self.radius * math.sin(self.theta)
        return fx, fy
