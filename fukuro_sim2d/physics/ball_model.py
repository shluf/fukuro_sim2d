"""
ball_model.py — Model fisika bola (pure physics, tanpa Pygame/ROS).

Semua state dalam world frame ROS-compliant:
    x, y    : posisi (meter)
    vx, vy  : velocity global (m/s)
    speed   : magnitude kecepatan (m/s)
    angle   : arah gerak (radian, world frame)

Bola mengalami decelerasi akibat gesekan lantai.
"""

import math


class BallModel:
    """Physics model untuk bola."""

    def __init__(self, x: float, y: float, radius: float = 0.12,
                 friction: float = 0.98, min_speed: float = 0.01, mass: float = 0.5):
        """
        Parameters
        ----------
        x, y : float
            Posisi awal (meter, world frame).
        radius : float
            Radius bola (meter).
        friction : float
            Koefisien redam per frame (0–1). Setiap frame: speed *= friction.
        min_speed : float
            Kecepatan di bawah ini dianggap 0.
        mass : float
            Massa bola (kg), untuk collision physics.
        """
        self.x = x
        self.y = y
        self.radius = radius

        self.speed = 0.0
        self.angle = 0.0   # radian, world frame

        self.friction = friction
        self.min_speed = min_speed
        self.mass = mass

    # ------------------------------------------------------------------

    def kick(self, angle: float, speed: float):
        """Tendang bola ke arah `angle` (radian, world) dengan `speed` (m/s)."""
        self.angle = angle
        self.speed = speed

    def grip_to(self, robot):
        """Tempel bola ke depan robot (saat dribbler aktif).

        Parameters
        ----------
        robot : RobotModel
        """
        fx, fy = robot.front_position()
        self.x = fx
        self.y = fy
        self.speed = 0.0

    def update(self, Ts: float):
        """Advance posisi bola satu timestep.

        Parameters
        ----------
        Ts : float   — periode sampling (detik)
        """
        if self.speed > self.min_speed:
            self.x += math.cos(self.angle) * self.speed * Ts
            self.y += math.sin(self.angle) * self.speed * Ts
            self.speed *= self.friction
        else:
            self.speed = 0.0

    # ------------------------------------------------------------------
    # Preset positions (dalam world meter)
    # ------------------------------------------------------------------

    def set_position(self, x: float, y: float):
        """Pindahkan bola ke posisi tertentu dan hentikan."""
        self.x = x
        self.y = y
        self.speed = 0.0
