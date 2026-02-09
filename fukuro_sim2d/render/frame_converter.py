"""
frame_converter.py — Konversi antara World Frame (ROS) dan Screen Frame (Pygame).

World Frame:
    REGIONAL: Origin (0, 0) di pojok kiri atas, X → kanan, Y → bawah
    NASIONAL: Origin (0, 0) di pojok kiri bawah, X → kanan, Y → atas
    θ = 0 menghadap +X, positif CCW

Screen Frame (Pygame):
    Origin (0, 0) di pojok kiri atas layar
    X → kanan
    Y → bawah
    θ positif CW

render_width bisa lebih kecil dari screen_width jika ada sidebar.
"""

import numpy as np


class FrameConverter:
    """Mengonversi koordinat dan sudut antara world frame dan screen frame."""

    def __init__(self, screen_width: int, screen_height: int,
                 field_length: float, field_width: float,
                 margin_x: float = 0.5, margin_y: float = 0.5,
                 render_width: int | None = None,
                 origin_top_left: bool = False):
        """
        Parameters
        ----------
        screen_width : int
            Lebar total layar Pygame (termasuk sidebar).
        screen_height : int
            Tinggi layar Pygame dalam pixel.
        field_length : float
            Panjang lapangan dalam meter (sumbu X world).
        field_width : float
            Lebar lapangan dalam meter (sumbu Y world).
        margin_x : float
            Margin kiri-kanan di world meter.
        margin_y : float
            Margin atas-bawah di world meter.
        render_width : int | None
            Lebar area render lapangan (pixel). Jika None, sama dengan screen_width.
            Sidebar menempati screen_width - render_width di sisi kanan.
        origin_top_left : bool
            True untuk regional (origin di pojok kiri atas, Y ke bawah).
            False untuk nasional (origin di pojok kiri bawah, Y ke atas).
        """
        self.screen_width = screen_width
        self.screen_height = screen_height
        self.render_width = render_width if render_width is not None else screen_width
        self.field_length = field_length
        self.field_width = field_width
        self.margin_x = margin_x
        self.margin_y = margin_y
        self.origin_top_left = origin_top_left

        # Total world area yang di-render (lapangan + margin)
        total_world_x = field_length + 2 * margin_x
        total_world_y = field_width + 2 * margin_y

        # Scale: pixel per meter — gunakan render_width (bukan screen_width)
        self.scale = min(self.render_width / total_world_x,
                         screen_height / total_world_y)

        # Offset pixel supaya lapangan terpusat di area render
        rendered_px = total_world_x * self.scale
        rendered_py = total_world_y * self.scale
        self.offset_x = (self.render_width - rendered_px) / 2.0
        self.offset_y = (screen_height - rendered_py) / 2.0

    # ------------------------------------------------------------------
    # World → Screen
    # ------------------------------------------------------------------

    def world_to_screen(self, wx: float, wy: float) -> tuple[int, int]:
        """Konversi posisi world (meter) ke posisi screen (pixel).

        Regional: origin (0,0) = pojok kiri atas, Y ke bawah (sama dengan screen).
        Nasional: origin (0,0) = pojok kiri bawah, Y ke atas (perlu flip).
        Margin ditambahkan agar lapangan tidak menempel tepi layar.
        """
        px = (wx + self.margin_x) * self.scale + self.offset_x
        
        if self.origin_top_left:
            # Regional: Y world sudah ke bawah, sama dengan screen
            py = (wy + self.margin_y) * self.scale + self.offset_y
        else:
            # Nasional: Flip Y (world Y atas → screen Y bawah)
            py = self.screen_height - ((wy + self.margin_y) * self.scale + self.offset_y)
        
        return int(round(px)), int(round(py))

    def world_length_to_px(self, length_m: float) -> int:
        """Konversi panjang/jarak world (meter) ke pixel."""
        return int(round(abs(length_m) * self.scale))

    def theta_world_to_screen(self, theta: float) -> float:
        """Konversi sudut world (CCW positif) ke screen (CW positif untuk drawing).

        Pygame mengukur sudut CW dari +X screen, jadi cukup negate.
        """
        return -theta

    # ------------------------------------------------------------------
    # Screen → World
    # ------------------------------------------------------------------

    def screen_to_world(self, px: int, py: int) -> tuple[float, float]:
        """Konversi posisi screen (pixel) ke posisi world (meter)."""
        wx = (px - self.offset_x) / self.scale - self.margin_x
        
        if self.origin_top_left:
            # Regional: Y world ke bawah, sama dengan screen
            wy = (py - self.offset_y) / self.scale - self.margin_y
        else:
            # Nasional: Flip Y
            wy = (self.screen_height - py - self.offset_y) / self.scale - self.margin_y
        
        return wx, wy

    # ------------------------------------------------------------------
    # Helper: world rect → screen rect (untuk pygame.draw.rect)
    # ------------------------------------------------------------------

    def world_rect_to_screen(self, wx: float, wy: float,
                              w_meters: float, h_meters: float):
        """Konversi rectangle world ke Pygame rect (x_top_left, y_top_left, w_px, h_px).

        Regional: (wx, wy) = top-left corner (Y ke bawah).
        Nasional: (wx, wy) = bottom-left corner (Y ke atas), perlu konversi ke top-left.
        """
        if self.origin_top_left:
            # Regional: (wx, wy) sudah top-left
            sx, sy = self.world_to_screen(wx, wy)
        else:
            # Nasional: (wx, wy) = bottom-left, konversi ke top-left
            sx, sy = self.world_to_screen(wx, wy + h_meters)
        
        w_px = self.world_length_to_px(w_meters)
        h_px = self.world_length_to_px(h_meters)
        return (sx, sy, w_px, h_px)
