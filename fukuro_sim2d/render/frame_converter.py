"""
frame_converter.py — Konversi antara World Frame (ROS) dan Screen Frame (Pygame).

World Frame:
    REGIONAL: Origin (0, 0) di pojok kiri atas, X → bawah, Y → kanan
    NASIONAL: Origin (0, 0) di pojok kiri bawah, X → kanan, Y → atas
    θ = 0 menghadap arah positif sumbu pertama (regional: bawah, nasional: kanan), positif CCW

Screen Frame (Pygame):
    Origin (0, 0) di pojok kiri atas layar
    X → kanan
    Y → bawah

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
        self.origin_top_left = origin_top_left
        self.margin_x = margin_x
        self.margin_y = margin_y

        # Total world area yang di-render (lapangan + margin)
        if origin_top_left:
            # Regional: swap karena di world_to_screen, wy+margin_y->px dan wx+margin_x->py
            # Margin untuk screen horizontal (X) berasal dari world Y dimension
            # Margin untuk screen vertical (Y) berasal dari world X dimension  
            self.margin_screen_x = margin_y  # untuk screen horizontal
            self.margin_screen_y = margin_x  # untuk screen vertical
            total_world_x = field_length + 2 * margin_y  # horizontal screen dari Y world
            total_world_y = field_width + 2 * margin_x   # vertikal screen dari X world
        else:
            # Nasional: normal
            self.margin_screen_x = margin_x
            self.margin_screen_y = margin_y
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

        Regional: X→bawah, Y→kanan, swap ke screen X→kanan, Y→bawah.
        Nasional: X→kanan, Y→atas, flip Y ke screen Y→bawah.
        Margin ditambahkan agar lapangan tidak menempel tepi layar.
        """
        if self.origin_top_left:
            # Regional: swap X↔Y
            # world Y(horizontal, kanan) → screen X(horizontal, kanan): gunakan margin_x
            # world X(vertikal, bawah) → screen Y(vertikal, bawah): gunakan margin_y
            # Tapi karena world menggunakan koordinat (wx, wy) dimana wx=vertikal dan wy=horizontal,
            # kita perlu swap marginnya juga
            px = (wy + self.margin_y) * self.scale + self.offset_x  # wy horizontal, pakai margin_y karena field.width=vertikal
            py = (wx + self.margin_x) * self.scale + self.offset_y  # wx vertikal, pakai margin_x karena field.length=horizontal
        else:
            # Nasional: X sama, Y flip
            px = (wx + self.margin_x) * self.scale + self.offset_x
            py = self.screen_height - ((wy + self.margin_y) * self.scale + self.offset_y)
        
        return int(round(px)), int(round(py))

    def world_length_to_px(self, length_m: float) -> int:
        """Konversi panjang/jarak world (meter) ke pixel."""
        return int(round(abs(length_m) * self.scale))

    def theta_world_to_screen(self, theta: float) -> float:
        """Konversi sudut world ke screen untuk rendering.

        Regional: world X→bawah, Y→kanan. Theta 0 = world X+ (bawah).
                  Screen X→kanan, Y→bawah. Theta 0 = screen X+ (kanan).
                  World X+(bawah) → Screen Y+(bawah) = 90 deg dari Screen X+
                  Plus perlu negate karena rotasi terbalik akibat swap
        Nasional: world X→kanan, Y→atas. Theta 0 = world X+ (kanan).
                  Sama dengan screen X+ (kanan).
        """
        if self.origin_top_left:
            # Regional: negate untuk balik arah rotasi, lalu adjust -90 deg
            return -theta + 1.5708 
        else:
            # Nasional: orientasi negate
            return -theta

    # ------------------------------------------------------------------
    # Screen → World
    # ------------------------------------------------------------------

    def screen_to_world(self, px: int, py: int) -> tuple[float, float]:
        """Konversi posisi screen (pixel) ke posisi world (meter)."""
        if self.origin_top_left:
            # Regional: swap balik dengan margin yang sesuai
            # screen X → world Y(horizontal): margin_y
            # screen Y → world X(vertikal): margin_x
            wx = (py - self.offset_y) / self.scale - self.margin_x
            wy = (px - self.offset_x) / self.scale - self.margin_y
        else:
            # Nasional: X sama, Y flip
            wx = (px - self.offset_x) / self.scale - self.margin_x
            wy = (self.screen_height - py - self.offset_y) / self.scale - self.margin_y
        
        return wx, wy

    # ------------------------------------------------------------------
    # Helper: world rect → screen rect (untuk pygame.draw.rect)
    # ------------------------------------------------------------------

    def world_rect_to_screen(self, wx: float, wy: float,
                              w_meters: float, h_meters: float):
        """Konversi rectangle world ke Pygame rect (x_top_left, y_top_left, w_px, h_px).

        Regional: (wx, wy) = top-left corner, tapi karena swap X-Y, width dan height juga swap.
        Nasional: (wx, wy) = bottom-left corner (Y ke atas), perlu konversi ke top-left.
        """
        if self.origin_top_left:
            # Regional: (wx, wy) top-left, swap w-h untuk screen
            sx, sy = self.world_to_screen(wx, wy)
            w_px = self.world_length_to_px(h_meters)  # world height -> screen width
            h_px = self.world_length_to_px(w_meters)  # world width -> screen height
        else:
            # Nasional: (wx, wy) = bottom-left, konversi ke top-left
            sx, sy = self.world_to_screen(wx, wy + h_meters)
            w_px = self.world_length_to_px(w_meters)
            h_px = self.world_length_to_px(h_meters)
        
        return (sx, sy, w_px, h_px)
