"""
renderer.py — Visualisasi Pygame dengan sidebar informasi.

Mendukung dua mode:
  - REGIONAL : setengah lapangan 8×6 dengan obstacle
  - NASIONAL : lapangan penuh 12×8 dengan gawang kiri-kanan

Layout window:
  ┌─────────────────────┬──────────┐
  │                     │ SIDEBAR  │
  │    FIELD AREA       │  info    │
  │                     │  robots  │
  │                     │  mode sw │
  └─────────────────────┴──────────┘
"""

import math
import numpy as np
import pygame

from fukuro_sim2d.render.frame_converter import FrameConverter
from fukuro_sim2d.objects.field import (
    SimMode,
    RegionalFieldConfig, RegionalObstacleConfig,
    NasionalFieldConfig, NasionalObstacleConfig,
)
from fukuro_sim2d.physics.robot_model import RobotModel
from fukuro_sim2d.physics.ball_model import BallModel


# ──────────────────────────────────────────────────────────────────────
# Warna
# ──────────────────────────────────────────────────────────────────────
COLOR_FIELD       = (26, 125, 20)
COLOR_LINE        = (240, 240, 240)
COLOR_GOAL_LINE   = (255, 0, 0)
COLOR_GOAL_POST   = (255, 182, 193)
COLOR_PENALTY     = (255, 255, 0)
COLOR_BOX_ORANGE  = (255, 127, 80)
COLOR_BOX_BLUE    = (0, 0, 255)
COLOR_OBSTACLE    = (0, 0, 0)
COLOR_BALL        = (255, 127, 80)
COLOR_ROBOT_BODY  = (0, 255, 0)
COLOR_ENEMY_BODY  = (220, 50, 50)
COLOR_ROBOT_FRONT = (0, 0, 255)
COLOR_DRIBBLER_ON = (255, 255, 0)
COLOR_WHEEL       = (255, 0, 0)
COLOR_WHITE       = (255, 255, 255)
COLOR_RED         = (255, 0, 0)
COLOR_GREEN       = (34, 139, 34)
COLOR_SIDEBAR_BG  = (30, 30, 30)
COLOR_SIDEBAR_SEC = (50, 50, 50)
COLOR_CYAN        = (0, 200, 200)
COLOR_GRAY        = (150, 150, 150)
COLOR_GOAL_NET    = (255, 255, 255)
COLOR_CENTER_LINE = (240, 240, 240)
COLOR_BTN_ACTIVE  = (0, 180, 80)
COLOR_BTN_INACTIVE= (80, 80, 80)

SIDEBAR_WIDTH = 320


class Renderer:
    """Render seluruh scene simulasi + sidebar informasi."""

    def __init__(self, screen_width: int = 1400, screen_height: int = 900,
                 mode: SimMode = SimMode.REGIONAL):
        pygame.init()
        self.screen_width = screen_width
        self.screen_height = screen_height
        self.screen = pygame.display.set_mode((screen_width, screen_height))
        pygame.display.set_caption("Fukuro Omniwheel Simulation")
        self.clock = pygame.time.Clock()

        self.render_width = screen_width - SIDEBAR_WIDTH

        # Fonts
        self.font_title = pygame.font.SysFont("monospace", 22, bold=True)
        self.font = pygame.font.SysFont("monospace", 18)
        self.font_small = pygame.font.SysFont("monospace", 15)
        self.font_btn = pygame.font.SysFont("monospace", 20, bold=True)

        # Mode switch button rects (akan dihitung di _draw_sidebar)
        self.btn_regional_rect = pygame.Rect(0, 0, 0, 0)
        self.btn_nasional_rect = pygame.Rect(0, 0, 0, 0)
        self._obstacle_checkbox_rects = []

        # Set initial mode
        self.mode = mode
        self._apply_mode(mode)

    # ==================================================================
    # Mode switching
    # ==================================================================

    def _apply_mode(self, mode: SimMode):
        """Bangun FrameConverter dan field config sesuai mode."""
        self.mode = mode

        if mode == SimMode.REGIONAL:
            self.field = RegionalFieldConfig()
            self.obstacles = RegionalObstacleConfig()
            origin_top_left = True  # Regional: origin di pojok kiri atas
        else:
            self.field = NasionalFieldConfig()
            self.obstacles = NasionalObstacleConfig()
            origin_top_left = False  # Nasional: origin di pojok kiri bawah

        self.fc = FrameConverter(
            self.screen_width, self.screen_height,
            self.field.length, self.field.width,
            margin_x=0.6, margin_y=0.6,
            render_width=self.render_width,
            origin_top_left=origin_top_left,
        )

    def switch_mode(self, mode: SimMode):
        """Ganti mode dan rebuild FrameConverter."""
        if mode != self.mode:
            self._apply_mode(mode)

    # ==================================================================
    # Public draw API
    # ==================================================================

    def clear(self):
        self.screen.fill(COLOR_FIELD)

    def draw_field(self):
        """Gambar lapangan sesuai mode aktif (tanpa obstacle)."""
        if self.mode == SimMode.REGIONAL:
            self._draw_regional_field()
        else:
            self._draw_nasional_field()
        
        # Debug: tampilkan margin boundary
        # self.draw_margin_debug()

    def draw_robot(self, robot: RobotModel, label: str = "Robot",
                   color: tuple = COLOR_ROBOT_BODY,
                   wheel_angles_deg: list[float] | None = None):
        """Gambar satu robot."""
        cx, cy = self.fc.world_to_screen(robot.x, robot.y)
        r_px = self.fc.world_length_to_px(robot.radius)
        if r_px < 5:
            r_px = 5

        # Body
        pygame.draw.circle(self.screen, color, (cx, cy), r_px, 2)

        # Front indicator
        front_color = COLOR_DRIBBLER_ON if robot.dribbler_pwm > 100 else COLOR_ROBOT_FRONT
        theta_s = self.fc.theta_world_to_screen(robot.theta)
        fx = cx + r_px * math.cos(theta_s)
        fy = cy + r_px * math.sin(theta_s)
        pygame.draw.circle(self.screen, front_color, (int(fx), int(fy)), 4)

        # Label
        name_surf = self.font_small.render(label, True, COLOR_WHITE)
        self.screen.blit(name_surf, (cx - r_px, cy - r_px - 18))

        # Wheels
        if wheel_angles_deg is None:
            wheel_angles_deg = [180.0, 300.0, 60.0]
        for i, angle_deg in enumerate(wheel_angles_deg):
            angle_rad = math.radians(angle_deg)
            wx = cx + r_px * math.cos(theta_s + angle_rad)
            wy = cy + r_px * math.sin(theta_s + angle_rad)
            pygame.draw.circle(self.screen, COLOR_WHEEL, (int(wx), int(wy)), 7)

    def draw_ball(self, ball: BallModel):
        bx, by = self.fc.world_to_screen(ball.x, ball.y)
        r_px = self.fc.world_length_to_px(ball.radius)
        if r_px < 3:
            r_px = 3
        pygame.draw.circle(self.screen, COLOR_BALL, (bx, by), r_px)

    # ------------------------------------------------------------------
    # Sidebar
    # ------------------------------------------------------------------

    def draw_sidebar(self, mode: SimMode,
                     ally_robots: list[dict],
                     enemy_robots: list[dict],
                     ball_info: dict,
                     obstacles: list = None,
                     extra_lines: list[str] | None = None,
                     input_mode: bool = False,
                     input_text: str = ""):
        """Gambar sidebar informasi di sisi kanan.

        Parameters
        ----------
        mode : SimMode
        ally_robots : list[dict]
            Tiap dict: {name, x, y, theta, vx, vy, omega, strategy, state,
                        is_ready, is_gripped, is_stopped, dribbler_pwm}
        enemy_robots : list[dict]
            Tiap dict: {name, x, y, theta}
        ball_info : dict
            {x, y, speed}
        obstacles : list[DraggableObstacle]
            List obstacle dengan enabled checkbox.
        extra_lines : list[str]
            Baris teks tambahan (keybindings, dll).
        input_mode : bool
            True jika sedang input koordinat obstacle.
        input_text : str
            Teks input saat ini.
        """
        sb_x = self.render_width
        sb_w = SIDEBAR_WIDTH
        sb_h = self.screen_height

        # Background
        pygame.draw.rect(self.screen, COLOR_SIDEBAR_BG,
                         (sb_x, 0, sb_w, sb_h))
        pygame.draw.line(self.screen, COLOR_GRAY,
                         (sb_x, 0), (sb_x, sb_h), 2)

        y = 10
        pad = sb_x + 12

        # ── Title ──
        title = self.font_title.render("FUKURO SIM 2D", True, COLOR_CYAN)
        self.screen.blit(title, (pad, y))
        y += 30

        # ── Mode switch buttons ──
        y += 5
        btn_w = (sb_w - 36) // 2
        btn_h = 30

        self.btn_regional_rect = pygame.Rect(pad, y, btn_w, btn_h)
        self.btn_nasional_rect = pygame.Rect(pad + btn_w + 12, y, btn_w, btn_h)

        reg_color = COLOR_BTN_ACTIVE if mode == SimMode.REGIONAL else COLOR_BTN_INACTIVE
        nas_color = COLOR_BTN_ACTIVE if mode == SimMode.NASIONAL else COLOR_BTN_INACTIVE

        pygame.draw.rect(self.screen, reg_color, self.btn_regional_rect, border_radius=6)
        pygame.draw.rect(self.screen, nas_color, self.btn_nasional_rect, border_radius=6)

        reg_txt = self.font_btn.render("REGIONAL", True, COLOR_WHITE)
        nas_txt = self.font_btn.render("NASIONAL", True, COLOR_WHITE)
        self.screen.blit(reg_txt, (self.btn_regional_rect.x + 10, self.btn_regional_rect.y + 5))
        self.screen.blit(nas_txt, (self.btn_nasional_rect.x + 10, self.btn_nasional_rect.y + 5))
        y += btn_h + 15

        # ── Separator ──
        pygame.draw.line(self.screen, COLOR_GRAY,
                         (pad, y), (sb_x + sb_w - 12, y), 1)
        y += 10

        # ── Ball info ──
        y = self._sidebar_section(y, pad, "BOLA", [
            f"Pos : ({ball_info.get('x', 0):.2f}, {ball_info.get('y', 0):.2f})",
            f"Speed: {ball_info.get('speed', 0):.2f} m/s",
        ])

        # ── Ally robots ──
        for rinfo in ally_robots:
            ready_str = "YES" if rinfo.get("is_ready") else "NO"
            grip_str = "YES" if rinfo.get("is_gripped") else "NO"
            stop_str = "STOP" if rinfo.get("is_stopped") else "RUN"
            stop_color = "(RED)" if rinfo.get("is_stopped") else "(GRN)"
            lines = [
                f"Pos  : ({rinfo['x']:.2f}, {rinfo['y']:.2f})",
                f"Theta: {math.degrees(rinfo['theta']):.1f} deg",
                f"Vel  : ({rinfo['vx']:.2f}, {rinfo['vy']:.2f})",
                f"State: {stop_str} {stop_color}",
                f"Ready: {ready_str}  Grip: {grip_str}",
                f"Strat: {rinfo.get('strategy', '-')}",
            ]
            y = self._sidebar_section(y, pad, rinfo['name'], lines,
                                       title_color=COLOR_GREEN)

        # ── Enemy robots ──
        if enemy_robots:
            for rinfo in enemy_robots:
                lines = [
                    f"Pos  : ({rinfo['x']:.2f}, {rinfo['y']:.2f})",
                    f"Theta: {math.degrees(rinfo['theta']):.1f} deg",
                ]
                y = self._sidebar_section(y, pad, rinfo['name'], lines,
                                           title_color=COLOR_RED)

        # ── Obstacles ──
        if obstacles:
            pygame.draw.line(self.screen, COLOR_GRAY,
                             (pad, y), (sb_x + sb_w - 12, y), 1)
            y += 8
            title_surf = self.font.render(f"OBSTACLES ({len(obstacles)})", True, COLOR_CYAN)
            self.screen.blit(title_surf, (pad, y))
            y += 22
            
            # Store checkbox rects for click detection
            self._obstacle_checkbox_rects = []
            
            for i, obs in enumerate(obstacles):
                # Checkbox
                cb_rect = pygame.Rect(pad, y, 15, 15)
                self._obstacle_checkbox_rects.append(cb_rect)
                cb_color = COLOR_GREEN if obs.enabled else COLOR_GRAY
                pygame.draw.rect(self.screen, cb_color, cb_rect, 2)
                if obs.enabled:
                    pygame.draw.line(self.screen, COLOR_GREEN,
                                     (cb_rect.x+3, cb_rect.y+7),
                                     (cb_rect.x+6, cb_rect.y+11), 2)
                    pygame.draw.line(self.screen, COLOR_GREEN,
                                     (cb_rect.x+6, cb_rect.y+11),
                                     (cb_rect.x+12, cb_rect.y+3), 2)
                
                # Text
                text = f"({obs.x:.1f}, {obs.y:.1f})"
                surf = self.font_small.render(text, True, COLOR_WHITE)
                self.screen.blit(surf, (pad + 20, y))
                y += 18

            y += 6

        # ── Input mode ──
        if input_mode:
            pygame.draw.line(self.screen, COLOR_GRAY,
                             (pad, y), (sb_x + sb_w - 12, y), 1)
            y += 8
            prompt_surf = self.font.render("Input (x,y):", True, COLOR_CYAN)
            self.screen.blit(prompt_surf, (pad, y))
            y += 22
            input_surf = self.font_small.render(input_text + "_", True, COLOR_WHITE)
            self.screen.blit(input_surf, (pad + 4, y))
            y += 20

        # ── Extra lines (keybindings) ──
        if extra_lines:
            pygame.draw.line(self.screen, COLOR_GRAY,
                             (pad, y), (sb_x + sb_w - 12, y), 1)
            y += 8
            for line in extra_lines:
                surf = self.font_small.render(line, True, COLOR_GRAY)
                self.screen.blit(surf, (pad, y))
                y += 17

    def _sidebar_section(self, y: int, pad: int, title: str,
                          lines: list[str],
                          title_color: tuple = COLOR_CYAN) -> int:
        """Gambar satu section sidebar, return y baru."""
        title_surf = self.font.render(title, True, title_color)
        self.screen.blit(title_surf, (pad, y))
        y += 22
        for line in lines:
            surf = self.font_small.render(line, True, COLOR_WHITE)
            self.screen.blit(surf, (pad + 4, y))
            y += 17
        y += 6
        return y

    def flip(self, fps: int = 60):
        pygame.display.flip()
        self.clock.tick(fps)

    # ==================================================================
    # Click handling
    # ==================================================================

    def check_mode_button_click(self, pos: tuple[int, int]) -> SimMode | None:
        """Cek apakah klik mengenai tombol mode. Return mode baru atau None."""
        if self.btn_regional_rect.collidepoint(pos):
            return SimMode.REGIONAL
        if self.btn_nasional_rect.collidepoint(pos):
            return SimMode.NASIONAL
        return None

    def check_obstacle_checkbox_click(self, pos: tuple[int, int], obstacles: list) -> int | None:
        """Cek apakah klik mengenai checkbox obstacle. Return index atau None."""
        for i, rect in enumerate(self._obstacle_checkbox_rects):
            if rect.collidepoint(pos):
                return i
        return None

    def draw_obstacles(self, obstacles: list):
        """Gambar semua obstacle yang enabled."""
        fc = self.fc
        for obs in obstacles:
            if obs.enabled:
                half = obs.size / 2.0
                rect = fc.world_rect_to_screen(obs.x - half, obs.y - half, obs.size, obs.size)
                pygame.draw.rect(self.screen, COLOR_OBSTACLE, rect)

    # ==================================================================
    # Debug visualization
    # ==================================================================

    def draw_margin_debug(self):
        """Gambar garis batas margin untuk debugging."""
        fc = self.fc
        f = self.field
        
        # Warna untuk garis debug
        COLOR_MARGIN = (255, 0, 255)  # Magenta untuk margin
        
        if self.mode == SimMode.REGIONAL:  
            # Outer margin boundary - hitung dari koordinat screen langsung
            # Field dalam screen pixel
            field_tl_screen = fc.world_to_screen(0, 0)
            field_br_screen = fc.world_to_screen(f.width, f.length)
            
            # Margin dalam pixel
            margin_x_px = fc.margin_screen_x * fc.scale
            margin_y_px = fc.margin_screen_y * fc.scale
            
            # Outer rectangle dengan margin
            outer_x = field_tl_screen[0] - margin_x_px
            outer_y = field_tl_screen[1] - margin_y_px
            outer_w = (field_br_screen[0] - field_tl_screen[0]) + 2 * margin_x_px
            outer_h = (field_br_screen[1] - field_tl_screen[1]) + 2 * margin_y_px
            
            pygame.draw.rect(self.screen, COLOR_MARGIN, 
                           (int(outer_x), int(outer_y), int(outer_w), int(outer_h)), 4)
            
        else:
            # Outer margin - dari screen pixel langsung
            field_tl_screen = fc.world_to_screen(0, f.width)
            field_br_screen = fc.world_to_screen(f.length, 0)
            
            margin_x_px = fc.margin_screen_x * fc.scale
            margin_y_px = fc.margin_screen_y * fc.scale
            
            outer_x = field_tl_screen[0] - margin_x_px
            outer_y = field_tl_screen[1] - margin_y_px
            outer_w = (field_br_screen[0] - field_tl_screen[0]) + 2 * margin_x_px
            outer_h = (field_br_screen[1] - field_tl_screen[1]) + 2 * margin_y_px
            
            pygame.draw.rect(self.screen, COLOR_MARGIN,
                           (int(outer_x), int(outer_y), int(outer_w), int(outer_h)), 2)

    # ==================================================================
    # Regional field drawing
    # ==================================================================

    def _draw_regional_field(self):
        fc = self.fc
        f: RegionalFieldConfig = self.field

        # Border - swap width dan length karena world_rect_to_screen akan swap lagi
        rect = fc.world_rect_to_screen(0, 0, f.width, f.length)
        pygame.draw.rect(self.screen, COLOR_LINE, rect, 5)

        # Penalty area
        rect_p = fc.world_rect_to_screen(
            f.penalty_area_x, f.penalty_area_y,
            f.penalty_area_w, f.penalty_area_h)
        pygame.draw.rect(self.screen, COLOR_PENALTY, rect_p, 4)

        # Center area
        rect_c = fc.world_rect_to_screen(
            f.center_area_x, f.center_area_y,
            f.center_area_w, f.center_area_h)
        pygame.draw.rect(self.screen, COLOR_PENALTY, rect_c, 4)

        # Goal line - horizontal line di X=0, Y dari 3 sampai 5
        gl1 = fc.world_to_screen(f.goal_line_x, f.goal_line_y1)
        gl2 = fc.world_to_screen(f.goal_line_x, f.goal_line_y2)
        pygame.draw.line(self.screen, COLOR_GOAL_LINE, gl1, gl2, 12)

        # Goal posts
        pygame.draw.circle(self.screen, COLOR_GOAL_POST,
                           fc.world_to_screen(*f.goal_post_left), 5)
        pygame.draw.circle(self.screen, COLOR_GOAL_POST,
                           fc.world_to_screen(*f.goal_post_right), 5)

        # Corner boxes
        rect_cr = fc.world_rect_to_screen(
            f.corner_box_right_x, f.corner_box_right_y,
            f.corner_box_size, f.corner_box_size)
        pygame.draw.rect(self.screen, COLOR_BOX_ORANGE, rect_cr, 5)

        rect_cl = fc.world_rect_to_screen(
            f.corner_box_left_x, f.corner_box_left_y,
            f.corner_box_size, f.corner_box_size)
        pygame.draw.rect(self.screen, COLOR_BOX_BLUE, rect_cl, 5)

    # ==================================================================
    # Nasional field drawing
    # ==================================================================

    def _draw_nasional_field(self):
        fc = self.fc
        f: NasionalFieldConfig = self.field

        # Border
        rect = fc.world_rect_to_screen(0, 0, f.length, f.width)
        pygame.draw.rect(self.screen, COLOR_LINE, rect, 4)

        # ── Garis tengah ──
        mid_bot = fc.world_to_screen(f.center_x, 0)
        mid_top = fc.world_to_screen(f.center_x, f.width)
        pygame.draw.line(self.screen, COLOR_CENTER_LINE, mid_bot, mid_top, 3)

        # Lingkaran tengah
        center_px = fc.world_to_screen(f.center_x, f.width / 2)
        cr_px = fc.world_length_to_px(f.center_circle_radius)
        pygame.draw.circle(self.screen, COLOR_CENTER_LINE, center_px, cr_px, 3)

        # Titik tengah
        pygame.draw.circle(self.screen, COLOR_WHITE, center_px, 4)

        # ── Penalty area KIRI ──
        rect_pl = fc.world_rect_to_screen(
            f.penalty_left_x, f.penalty_left_y,
            f.penalty_left_w, f.penalty_left_h)
        pygame.draw.rect(self.screen, COLOR_LINE, rect_pl, 3)

        # ── Penalty area KANAN ──
        rect_pr = fc.world_rect_to_screen(
            f.penalty_right_x, f.penalty_right_y,
            f.penalty_right_w, f.penalty_right_h)
        pygame.draw.rect(self.screen, COLOR_LINE, rect_pr, 3)

        # ── Goal area KIRI ──
        rect_gl = fc.world_rect_to_screen(
            f.goal_left_area_x, f.goal_left_area_y,
            f.goal_left_depth, f.goal_left_width)
        pygame.draw.rect(self.screen, COLOR_LINE, rect_gl, 3)

        # ── Goal area KANAN ──
        rect_gr = fc.world_rect_to_screen(
            f.goal_right_area_x, f.goal_right_area_y,
            f.goal_right_depth, f.goal_right_width)
        pygame.draw.rect(self.screen, COLOR_LINE, rect_gr, 3)

        # ── Gawang KIRI (net di belakang garis, x < 0) ──
        goal_depth = 0.4  # kedalaman visual net
        g_bl = fc.world_to_screen(-goal_depth, f.goal_left_line_y1)
        g_tl = fc.world_to_screen(-goal_depth, f.goal_left_line_y2)
        g_br = fc.world_to_screen(0, f.goal_left_line_y1)
        g_tr = fc.world_to_screen(0, f.goal_left_line_y2)
        pygame.draw.line(self.screen, COLOR_GOAL_NET, g_bl, g_tl, 4)
        pygame.draw.line(self.screen, COLOR_GOAL_NET, g_bl, g_br, 4)
        pygame.draw.line(self.screen, COLOR_GOAL_NET, g_tl, g_tr, 4)
        pygame.draw.line(self.screen, COLOR_GOAL_LINE, g_br, g_tr, 5)

        # ── Gawang KANAN ──
        g_bl2 = fc.world_to_screen(f.length, f.goal_right_line_y1)
        g_tl2 = fc.world_to_screen(f.length, f.goal_right_line_y2)
        g_br2 = fc.world_to_screen(f.length + goal_depth, f.goal_right_line_y1)
        g_tr2 = fc.world_to_screen(f.length + goal_depth, f.goal_right_line_y2)
        pygame.draw.line(self.screen, COLOR_GOAL_NET, g_br2, g_tr2, 4)
        pygame.draw.line(self.screen, COLOR_GOAL_NET, g_bl2, g_br2, 4)
        pygame.draw.line(self.screen, COLOR_GOAL_NET, g_tl2, g_tr2, 4)
        pygame.draw.line(self.screen, COLOR_GOAL_LINE, g_bl2, g_tl2, 5)

        # ── Penalty spots ──
        pygame.draw.circle(self.screen, COLOR_WHITE,
                           fc.world_to_screen(*f.penalty_spot_left), 4)
        pygame.draw.circle(self.screen, COLOR_WHITE,
                           fc.world_to_screen(*f.penalty_spot_right), 4)
