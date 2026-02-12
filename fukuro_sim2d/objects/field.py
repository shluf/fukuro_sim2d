"""
field.py — Definisi dimensi dan elemen lapangan dalam world frame (meter).

Mendukung dua mode kompetisi:
  - REGIONAL : setengah lapangan 8m × 6m, obstacle, gawang di bawah (Y=6)
  - NASIONAL : lapangan penuh 12m × 8m, gawang kiri & kanan, robot lawan

Semua koordinat world frame:
    REGIONAL : Origin (0, 0) = pojok kiri atas, X → kanan, Y → bawah
    NASIONAL : Origin (0, 0) = pojok kiri bawah, X → kanan, Y → atas
"""

from dataclasses import dataclass, field
from enum import Enum


class SimMode(Enum):
    REGIONAL = "regional"
    NASIONAL = "nasional"


# ======================================================================
# Regional Field — setengah lapangan 8×6 dengan obstacle
# ======================================================================

@dataclass
class RegionalFieldConfig:
    """Konfigurasi lapangan regional (8m × 6m, gawang di bawah → Y=6)."""

    length: float = 8.0
    width: float = 6.0

    # Area penalti (bawah): x=[2,6], y=[4.5,6.0]
    penalty_area_x: float = 2.0
    penalty_area_w: float = 4.0
    penalty_area_y: float = 0.0
    penalty_area_h: float = 1.5

    # Area tengah: x=[3,5], y=[0,4.5]
    center_area_x: float = 3.0
    center_area_w: float = 2.0
    center_area_y: float = 1.5
    center_area_h: float = 4.5

    # Goal line (bawah): x=[3,5], y=6.0
    goal_line_x1: float = 3.0
    goal_line_x2: float = 5.0
    goal_line_y: float = 0.0

    # Goal posts
    goal_post_left: tuple[float, float] = (4.4, 0.0)
    goal_post_right: tuple[float, float] = (3.6, 0.0)

    # Corner boxes
    corner_box_right_x: float = 0.0
    corner_box_right_y: float = 4.0
    corner_box_size: float = 0.75
    corner_box_left_x: float = 7.25
    corner_box_left_y: float = 4.0

    # Preset ball positions
    kickoff_position: tuple[float, float] = (4.0, 6.0)
    corner_left_position: tuple[float, float] = (7.88, 0.12)
    corner_right_position: tuple[float, float] = (0.12, 0.12)

    # Posisi awal robot (world meter)
    robot1_start: tuple[float, float, float] = (0.3, 4.375, 0.0)       # x, y, theta
    robot2_start: tuple[float, float, float] = (7.7, 4.375, 3.14159)


@dataclass
class RegionalObstacleConfig:
    """Obstacle untuk mode regional."""

    size: float = 0.4

    positions: list[tuple[float, float]] = field(default_factory=lambda: [
        (6.0, 2.2),
        (6.0, 3.4),
        (6.0, 4.6),
        (4.0, 2.2),
        (4.0, 3.4),
        (2.0, 2.2),
        (2.0, 3.4),
        (2.0, 4.6),
        (4.8, 0.0),
        (3.2, 0.0),
        (4.0, 0.0),
    ])


# ======================================================================
# Nasional Field — lapangan penuh 12×8 dengan gawang kiri-kanan
# ======================================================================

@dataclass
class NasionalFieldConfig:
    """Konfigurasi lapangan nasional (12m × 8m, gawang kiri & kanan)."""

    length: float = 12.0
    width: float = 8.0

    # ── Garis tengah ──
    center_x: float = 6.0
    center_circle_radius: float = 1.0

    # ── Goal area KIRI (x=0) ──
    goal_left_depth: float = 1.5
    goal_left_width: float = 3.0
    goal_left_area_x: float = 0.0
    goal_left_area_y: float = 2.5     # (8 - 3) / 2
    goal_left_line_y1: float = 3.0
    goal_left_line_y2: float = 5.0

    # ── Goal area KANAN (x=12) ──
    goal_right_depth: float = 1.5
    goal_right_width: float = 3.0
    goal_right_area_x: float = 10.5   # 12 - 1.5
    goal_right_area_y: float = 2.5
    goal_right_line_y1: float = 3.0
    goal_right_line_y2: float = 5.0

    # ── Penalty area KIRI ──
    penalty_left_x: float = 0.0
    penalty_left_y: float = 1.5
    penalty_left_w: float = 2.5
    penalty_left_h: float = 5.0

    # ── Penalty area KANAN ──
    penalty_right_x: float = 9.5
    penalty_right_y: float = 1.5
    penalty_right_w: float = 2.5
    penalty_right_h: float = 5.0

    # Penalty spots
    penalty_spot_left: tuple[float, float] = (2.0, 4.0)
    penalty_spot_right: tuple[float, float] = (10.0, 4.0)

    # Preset ball positions
    kickoff_position: tuple[float, float] = (6.0, 4.0)
    corner_bl: tuple[float, float] = (0.12, 0.12)
    corner_br: tuple[float, float] = (11.88, 0.12)
    corner_tl: tuple[float, float] = (0.12, 7.88)
    corner_tr: tuple[float, float] = (11.88, 7.88)

    # ── Posisi awal robot (world meter) ──
    robot1_start: tuple[float, float, float] = (1.0, 4.0, 0.0)
    robot2_start: tuple[float, float, float] = (3.0, 2.5, 0.0)

    # Tim lawan (kanan)
    enemy1_start: tuple[float, float, float] = (11.0, 4.0, 3.14159)
    enemy2_start: tuple[float, float, float] = (9.0, 5.5, 3.14159)
    enemy3_start: tuple[float, float, float] = (9.0, 2.5, 3.14159)


@dataclass
class NasionalObstacleConfig:
    """Obstacle untuk mode nasional (tidak ada obstacle statis)."""

    size: float = 0.0
    positions: list[tuple[float, float]] = field(default_factory=list)
