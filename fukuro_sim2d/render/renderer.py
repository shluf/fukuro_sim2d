"""
renderer.py — Visualisasi Pygame dengan sidebar informasi.

Mendukung dua mode:
  - REGIONAL : setengah lapangan 8x6 dengan obstacle
  - NASIONAL : lapangan penuh 12x8 dengan gawang kiri-kanan

Layout window:
  ┌─────────────────────┬──────────┐
  │                     │ SIDEBAR  │
  │    FIELD AREA       │  info    │
  │                     │  robots  │
  │                     │  mode sw │
  └─────────────────────┴──────────┘
"""

import math
import pygame
import pygame.freetype

from fukuro_sim2d.render.frame_converter import FrameConverter
from fukuro_sim2d.render.scoreboard import Scoreboard
from fukuro_sim2d.objects.field import (
    SimMode,
    RegionalFieldConfig, RegionalObstacleConfig,
    NasionalFieldConfig, NasionalObstacleConfig,
)
from fukuro_sim2d.physics.robot_model import RobotModel
from fukuro_sim2d.physics.ball_model import BallModel


# ──────────────────────────────────────────────────────────────────────
# Field / robot colors (unchanged)
# ──────────────────────────────────────────────────────────────────────
COLOR_FIELD       = (26, 125, 20)
COLOR_LINE        = (240, 240, 240)
COLOR_GOAL_LINE   = (255, 0, 0)
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

# ──────────────────────────────────────────────────────────────────────
# Sidebar design palette
# ──────────────────────────────────────────────────────────────────────
SB_BG          = (18, 20, 26)       # near-black background
SB_SURFACE     = (26, 29, 38)       # elevated card surface
SB_BORDER      = (44, 48, 62)       # subtle divider / border
SB_ACCENT      = (0, 188, 212)      # teal accent (labels, headers)
SB_GREEN       = (39, 174, 96)      # status OK
SB_RED         = (231, 76, 60)      # status danger / enemy
SB_AMBER       = (243, 156, 18)     # status warning
SB_TEXT        = (220, 224, 235)    # primary text
SB_TEXT_DIM    = (110, 118, 140)    # secondary / dim text
SB_HEADER_BG   = (28, 32, 44)       # header strip fill
SB_BTN_REG_ACT = (17, 153, 83)     # regional active button
SB_BTN_NAS_ACT = (24, 114, 210)    # nasional active button
SB_BTN_INACT   = (38, 42, 56)      # inactive button

SIDEBAR_WIDTH = 320


class Renderer:
    """Render seluruh scene simulasi + sidebar informasi."""

    def __init__(self, screen_width: int = 1400, screen_height: int = 900,
                 mode: SimMode = SimMode.REGIONAL):
        pygame.init()
        self.screen_width  = screen_width
        self.screen_height = screen_height
        self.screen = pygame.display.set_mode((screen_width, screen_height))
        pygame.display.set_caption("Fukuro Omniwheel Simulation")
        self.clock = pygame.time.Clock()

        self.render_width = screen_width - SIDEBAR_WIDTH

        # Fonts using pygame.freetype
        pygame.freetype.init()
        self.font_title = pygame.freetype.SysFont("liberation mono, consolas, courier new", 20, bold=True)
        self.font_sec   = pygame.freetype.SysFont("liberation mono, consolas, courier new", 13, bold=True)
        self.font       = pygame.freetype.SysFont("liberation mono, consolas, courier new", 15)
        self.font_small = pygame.freetype.SysFont("liberation mono, consolas, courier new", 13)
        self.font_btn   = pygame.freetype.SysFont("liberation mono, consolas, courier new", 13, bold=True)
        self.font_label = pygame.freetype.SysFont("liberation mono, consolas, courier new", 11)
        self.font_value = pygame.freetype.SysFont("liberation mono, consolas, courier new", 14, bold=True)
        self.font_score = pygame.freetype.SysFont("liberation mono, consolas, courier new", 28, bold=True)

        # Click-target rects (populated during draw_sidebar)
        self.btn_regional_rect         = pygame.Rect(0, 0, 0, 0)
        self.btn_nasional_rect         = pygame.Rect(0, 0, 0, 0)
        self._obstacle_checkbox_rects  = []
        self._obstacle_delete_rects    = []   # delete buttons per obstacle
        self.btn_add_obstacle_rect     = pygame.Rect(0, 0, 0, 0)
        self.btn_wheel_speed_rect      = pygame.Rect(0, 0, 0, 0)
        self.btn_obs_coords_rect       = pygame.Rect(0, 0, 0, 0)
        self.btn_reset_score_rect      = pygame.Rect(0, 0, 0, 0)

        # Toggle states
        self.show_wheel_speeds         = False
        self.show_obstacle_coords      = False
        
        # Sidebar scroll
        self.sidebar_scroll_offset     = 0
        self.sidebar_content_height    = 0
        self.sidebar_viewport_rect     = pygame.Rect(0, 0, 0, 0)
        
        # Scoreboard
        self.team_score                = 0
        self.enemy_score               = 0

        self.mode = mode
        self._apply_mode(mode)
        
        # Initialize scoreboard widget
        self.scoreboard = Scoreboard()

    # ==================================================================
    # Mode switching
    # ==================================================================

    def _apply_mode(self, mode: SimMode):
        """Bangun FrameConverter dan field config sesuai mode."""
        self.mode = mode

        if mode == SimMode.REGIONAL:
            self.field         = RegionalFieldConfig()
            self.obstacles     = RegionalObstacleConfig()
            origin_top_left    = True
        else:
            self.field         = NasionalFieldConfig()
            self.obstacles     = NasionalObstacleConfig()
            origin_top_left    = False

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
        r_px   = self.fc.world_length_to_px(robot.radius)
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
        name_surf, name_rect = self.font_small.render(label, COLOR_WHITE)
        self.screen.blit(name_surf, (cx - r_px, cy - r_px - 28))

        # Wheels
        if wheel_angles_deg is None:
            wheel_angles_deg = [180.0, 300.0, 60.0]
        for i, angle_deg in enumerate(wheel_angles_deg):
            angle_rad = math.radians(angle_deg)
            wx = cx + r_px * math.cos(theta_s + angle_rad)
            wy = cy + r_px * math.sin(theta_s + angle_rad)
            pygame.draw.circle(self.screen, COLOR_WHEEL, (int(wx), int(wy)), 7)

            # Draw wheel speed label next to wheel if toggle is on
            if self.show_wheel_speeds and hasattr(robot, 'omega_wheels'):
                spd = robot.omega_wheels[i] if i < len(robot.omega_wheels) else 0.0
                spd_txt = f"{spd:.1f}"
                spd_surf, spd_rect = self.font_label.render(spd_txt, COLOR_CYAN)
                # Offset label outward from wheel
                lx = cx + (r_px + 14) * math.cos(theta_s + angle_rad)
                ly = cy + (r_px + 14) * math.sin(theta_s + angle_rad)
                self.screen.blit(spd_surf,
                                 (int(lx) - spd_rect.width // 2,
                                  int(ly) - spd_rect.height // 2))

    def draw_ball(self, ball: BallModel):
        bx, by = self.fc.world_to_screen(ball.x, ball.y)
        r_px   = self.fc.world_length_to_px(ball.radius)
        if r_px < 3:
            r_px = 3
        pygame.draw.circle(self.screen, COLOR_BALL, (bx, by), r_px)

    def draw_scoreboard(self):
        """Draw floating scoreboard di pojok kiri atas field area."""
        self.scoreboard.team_score = self.team_score
        self.scoreboard.enemy_score = self.enemy_score
        self.scoreboard.draw(self.screen, self.mode.name, margin_x=90, margin_y=10)
    
    def handle_scroll(self, y_delta: int):
        """Handle mouse wheel scroll untuk sidebar.
        
        Parameters
        ----------
        y_delta : int  positive = scroll up, negative = scroll down
        """
        scroll_speed = 30
        self.sidebar_scroll_offset -= y_delta * scroll_speed
        # Clamping akan dilakukan di draw_sidebar
    
    def check_scoreboard_reset_click(self, pos: tuple[int, int]) -> bool:
        """Cek apakah klik mengenai tombol reset scoreboard."""
        return self.scoreboard.check_reset_click(pos)
    
    def reset_scoreboard(self):
        """Reset semua skor ke 0."""
        self.team_score = 0
        self.enemy_score = 0
        self.scoreboard.reset_scores()

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
        """Gambar sidebar informasi di sisi kanan dengan scrolling.

        Parameters
        ----------
        mode         : active SimMode
        ally_robots  : list of dicts with keys:
                        name, x, y, theta, vx, vy, omega, strategy, state,
                        is_ready, is_gripped, is_stopped, dribbler_pwm
        enemy_robots : list of dicts with keys: name, x, y, theta
        ball_info    : dict with keys: x, y, speed
        obstacles    : list of DraggableObstacle objects with .enabled
        extra_lines  : additional text lines (keybindings etc.)
        input_mode   : True while coordinate input is active
        input_text   : current input buffer text
        """
        sb_x = self.render_width
        sb_w = SIDEBAR_WIDTH
        sb_h = self.screen_height

        # Background fill
        pygame.draw.rect(self.screen, SB_BG, (sb_x, 0, sb_w, sb_h))
        # Left-edge accent bar
        pygame.draw.rect(self.screen, SB_ACCENT, (sb_x, 0, 2, sb_h))

        # Calculate content height for scrolling
        content_height = self._calculate_sidebar_content_height(
            mode, ally_robots, enemy_robots, obstacles, extra_lines, input_mode)
        self.sidebar_content_height = content_height
        
        # Clamp scroll offset
        max_scroll = max(0, content_height - sb_h)
        self.sidebar_scroll_offset = max(0, min(self.sidebar_scroll_offset, max_scroll))
        
        pad  = 14          # left content padding
        rpad = sb_w - 14   # right alignment edge
        y    = 0

        # ── Header strip (fixed, always visible) ─────────────────────
        pygame.draw.rect(self.screen, SB_HEADER_BG, (sb_x + 2, 0, sb_w - 2, 48))
        title_surf, title_rect = self.font_title.render("FUKURO  SIM  2D", SB_ACCENT)
        self.screen.blit(title_surf, (sb_x + pad, 14))
        mode_surf, mode_rect = self.font_label.render(mode.name, SB_TEXT_DIM)
        self.screen.blit(mode_surf, (sb_x + rpad - mode_rect.width, 18))
        
        # Start scrollable content after header
        y = 52
        scroll_start_y = y
        
        # Keep track of logical Y (for click detection) and draw Y (with scroll offset)
        y_logical = y  # Absolute coordinate for button rects
        scroll_offset = int(self.sidebar_scroll_offset)

        # ── Mode switch buttons ───────────────────────────────────────
        y_logical += 8
        btn_gap = 8
        btn_w   = (sb_w - 28 - btn_gap) // 2
        btn_h   = 30

        # Store rects with LOGICAL coordinates (for click detection)
        self.btn_regional_rect = pygame.Rect(sb_x + pad, y_logical, btn_w, btn_h)
        self.btn_nasional_rect = pygame.Rect(sb_x + pad + btn_w + btn_gap, y_logical, btn_w, btn_h)

        # Draw with OFFSET coordinates
        for rect_logical, lbl, active, act_col in (
            (self.btn_regional_rect, "REGIONAL",
             mode == SimMode.REGIONAL, SB_BTN_REG_ACT),
            (self.btn_nasional_rect, "NASIONAL",
             mode == SimMode.NASIONAL, SB_BTN_NAS_ACT),
        ):
            bg   = act_col         if active else SB_BTN_INACT
            bord = act_col         if active else SB_BORDER
            tc   = SB_TEXT         if active else SB_TEXT_DIM
            # Create draw rect with scroll offset applied
            rect_draw = pygame.Rect(rect_logical.x, rect_logical.y - scroll_offset,
                                   rect_logical.w, rect_logical.h)
            pygame.draw.rect(self.screen, bg,   rect_draw, border_radius=5)
            pygame.draw.rect(self.screen, bord, rect_draw, 1, border_radius=5)
            ts, ts_rect = self.font_btn.render(lbl, tc)
            self.screen.blit(ts, (
                rect_draw.x + (rect_draw.w - ts_rect.width)  // 2,
                rect_draw.y + (rect_draw.h - ts_rect.height) // 2,
            ))
        y_logical += btn_h + 12

        # ── Ball ─────────────────────────────────────────────────────
        y_logical = self._sb_divider(y_logical, sb_x, sb_w, scroll_offset)
        y_logical = self._sb_section_header(y_logical, sb_x + pad, "BALL", scroll_offset)
        y_logical = self._sb_kv_row(y_logical, sb_x + pad, sb_x + rpad, "Position",
                             f"({ball_info.get('x', 0):.2f},"
                             f" {ball_info.get('y', 0):.2f})", scroll_offset=scroll_offset)
        y_logical = self._sb_kv_row(y_logical, sb_x + pad, sb_x + rpad, "Speed",
                             f"{ball_info.get('speed', 0):.2f} m/s", scroll_offset=scroll_offset)
        y_logical += 4

        # ── Ally robots ──────────────────────────────────────────────
        for rinfo in ally_robots:
            y_logical = self._sb_divider(y_logical, sb_x, sb_w, scroll_offset)
            y_logical = self._sb_robot_card(y_logical, sb_x + pad, sb_x + rpad, rinfo, ally=True, scroll_offset=scroll_offset)

        # ── Enemy robots ─────────────────────────────────────────────
        for rinfo in enemy_robots:
            y_logical = self._sb_divider(y_logical, sb_x, sb_w, scroll_offset)
            y_logical = self._sb_robot_card(y_logical, sb_x + pad, sb_x + rpad, rinfo, ally=False, scroll_offset=scroll_offset)

        # ── Toggles ───────────────────────────────────────────────
        y_logical = self._sb_divider(y_logical, sb_x, sb_w, scroll_offset)
        y_logical = self._sb_section_header(y_logical, sb_x + pad, "TOGGLES", scroll_offset)

        # Wheel speed toggle
        self.btn_wheel_speed_rect = pygame.Rect(sb_x + pad + 2, y_logical + 1, 13, 13)
        ws_col = SB_GREEN if self.show_wheel_speeds else SB_BTN_INACT
        # Draw with offset
        rect_draw = pygame.Rect(self.btn_wheel_speed_rect.x, self.btn_wheel_speed_rect.y - scroll_offset,
                               self.btn_wheel_speed_rect.w, self.btn_wheel_speed_rect.h)
        pygame.draw.rect(self.screen, ws_col, rect_draw, border_radius=2)
        if self.show_wheel_speeds:
            pygame.draw.line(self.screen, SB_BG,
                             (rect_draw.x + 2, rect_draw.y + 6),
                             (rect_draw.x + 5, rect_draw.y + 9), 2)
            pygame.draw.line(self.screen, SB_BG,
                             (rect_draw.x + 5, rect_draw.y + 9),
                             (rect_draw.x + 11, rect_draw.y + 3), 2)
        else:
            pygame.draw.rect(self.screen, SB_BORDER, rect_draw, 1, border_radius=2)
        ws_label, ws_label_rect = self.font_small.render("Wheel Speeds", SB_TEXT)
        self.screen.blit(ws_label, (sb_x + pad + 22, y_logical - scroll_offset))
        y_logical += 20

        # Obstacle coords toggle
        self.btn_obs_coords_rect = pygame.Rect(sb_x + pad + 2, y_logical + 1, 13, 13)
        oc_col = SB_GREEN if self.show_obstacle_coords else SB_BTN_INACT
        rect_draw = pygame.Rect(self.btn_obs_coords_rect.x, self.btn_obs_coords_rect.y - scroll_offset,
                               self.btn_obs_coords_rect.w, self.btn_obs_coords_rect.h)
        pygame.draw.rect(self.screen, oc_col, rect_draw, border_radius=2)
        if self.show_obstacle_coords:
            pygame.draw.line(self.screen, SB_BG,
                             (rect_draw.x + 2, rect_draw.y + 6),
                             (rect_draw.x + 5, rect_draw.y + 9), 2)
            pygame.draw.line(self.screen, SB_BG,
                             (rect_draw.x + 5, rect_draw.y + 9),
                             (rect_draw.x + 11, rect_draw.y + 3), 2)
        else:
            pygame.draw.rect(self.screen, SB_BORDER, rect_draw, 1, border_radius=2)
        oc_label, oc_label_rect = self.font_small.render("Obstacle Coords", SB_TEXT)
        self.screen.blit(oc_label, (sb_x + pad + 22, y_logical - scroll_offset))
        y_logical += 24

        # ── Obstacles ────────────────────────────────────────────────
        y_logical = self._sb_divider(y_logical, sb_x, sb_w, scroll_offset)
        # Header row with section title and "+" add button
        y_header_logical = y_logical
        y_logical = self._sb_section_header(y_logical, sb_x + pad,
                                     f"OBSTACLES  [{len(obstacles) if obstacles else 0}]",
                                     scroll_offset)
        # "+" button at right side of header
        add_btn_size = 20
        self.btn_add_obstacle_rect = pygame.Rect(
            sb_x + rpad - add_btn_size, y_header_logical, add_btn_size, add_btn_size)
        rect_draw = pygame.Rect(self.btn_add_obstacle_rect.x,
                                self.btn_add_obstacle_rect.y - scroll_offset,
                                add_btn_size, add_btn_size)
        pygame.draw.rect(self.screen, SB_GREEN, rect_draw, border_radius=4)
        plus_surf, plus_rect = self.font_btn.render("+", SB_BG)
        self.screen.blit(plus_surf, (
            rect_draw.x + (add_btn_size - plus_rect.width) // 2,
            rect_draw.y + (add_btn_size - plus_rect.height) // 2,
        ))

        self._obstacle_checkbox_rects = []
        self._obstacle_delete_rects = []
        if obstacles:
            for obs in obstacles:
                # Checkbox - store logical coordinates
                cb_rect_logical = pygame.Rect(sb_x + pad + 2, y_logical + 1, 13, 13)
                self._obstacle_checkbox_rects.append(cb_rect_logical)
                en_col = SB_GREEN if obs.enabled else SB_BTN_INACT
                # Draw with offset
                cb_rect_draw = pygame.Rect(cb_rect_logical.x, cb_rect_logical.y - scroll_offset,
                                          cb_rect_logical.w, cb_rect_logical.h)
                pygame.draw.rect(self.screen, en_col, cb_rect_draw, border_radius=2)
                if obs.enabled:
                    pygame.draw.line(self.screen, SB_BG,
                                     (cb_rect_draw.x + 2,  cb_rect_draw.y + 6),
                                     (cb_rect_draw.x + 5,  cb_rect_draw.y + 9), 2)
                    pygame.draw.line(self.screen, SB_BG,
                                     (cb_rect_draw.x + 5,  cb_rect_draw.y + 9),
                                     (cb_rect_draw.x + 11, cb_rect_draw.y + 3), 2)
                else:
                    pygame.draw.rect(self.screen, SB_BORDER, cb_rect_draw, 1,
                                     border_radius=2)
                tc   = SB_TEXT if obs.enabled else SB_TEXT_DIM
                lsrf, lsrf_rect = self.font_small.render(
                    f"({obs.x:.1f}, {obs.y:.1f})", tc)
                self.screen.blit(lsrf, (sb_x + pad + 22, y_logical - scroll_offset))

                # Delete button ("x") at right side - store logical coordinates
                del_rect_logical = pygame.Rect(sb_x + rpad - 16, y_logical, 15, 15)
                self._obstacle_delete_rects.append(del_rect_logical)
                # Draw with offset
                del_rect_draw = pygame.Rect(del_rect_logical.x, del_rect_logical.y - scroll_offset,
                                           del_rect_logical.w, del_rect_logical.h)
                pygame.draw.rect(self.screen, SB_RED, del_rect_draw, border_radius=3)
                x_surf, x_rect = self.font_label.render("x", SB_BG)
                self.screen.blit(x_surf, (
                    del_rect_draw.x + (del_rect_draw.w - x_rect.width) // 2,
                    del_rect_draw.y + (del_rect_draw.h - x_rect.height) // 2,
                ))
                y_logical += 18
            y_logical += 4

        # ── Coordinate input ─────────────────────────────────────────
        if input_mode:
            y_logical = self._sb_divider(y_logical, sb_x, sb_w, scroll_offset)
            y_logical = self._sb_section_header(y_logical, sb_x + pad, "COORDINATE INPUT", scroll_offset)
            box_logical = pygame.Rect(sb_x + pad, y_logical, sb_w - 28, 26)
            box_draw = pygame.Rect(box_logical.x, box_logical.y - scroll_offset,
                                  box_logical.w, box_logical.h)
            pygame.draw.rect(self.screen, SB_SURFACE, box_draw, border_radius=4)
            pygame.draw.rect(self.screen, SB_ACCENT,  box_draw, 1, border_radius=4)
            inp, inp_rect = self.font_small.render(input_text + "|", SB_TEXT)
            self.screen.blit(inp, (box_draw.x + 6, box_draw.y + 5))
            y_logical += 32

        # ── Key bindings ─────────────────────────────────────────────
        if extra_lines:
            y_logical = self._sb_divider(y_logical, sb_x, sb_w, scroll_offset)
            y_logical = self._sb_section_header(y_logical, sb_x + pad, "CONTROLS", scroll_offset)
            for line in extra_lines:
                if ":" in line:
                    key, desc = line.split(":", 1)
                    ks, ks_rect = self.font_label.render(key + ":", SB_ACCENT)
                    ds, ds_rect = self.font_label.render(desc, SB_TEXT_DIM)
                    self.screen.blit(ks, (sb_x + pad, y_logical - scroll_offset))
                    self.screen.blit(ds, (sb_x + pad + ks_rect.width + 4, y_logical - scroll_offset))
                else:
                    s, s_rect = self.font_label.render(line, SB_TEXT_DIM)
                    self.screen.blit(s, (sb_x + pad, y_logical - scroll_offset))
                y_logical += 15
            y_logical += 4

    # ------------------------------------------------------------------
    # Sidebar primitive helpers
    # ------------------------------------------------------------------

    def _calculate_sidebar_content_height(self, mode, ally_robots, enemy_robots,
                                          obstacles, extra_lines, input_mode) -> int:
        """Calculate total height of sidebar content for scrolling.
        
        Returns approximate height in pixels.
        """
        # Rough estimation based on sections
        height = 52  # Header (not scrolled)
        height += 50  # Mode buttons
        height += 80  # Ball info
        height += len(ally_robots) * 140  # Ally robot cards
        height += len(enemy_robots) * 80  # Enemy robot cards
        height += 70  # Toggles section
        height += 60  # Obstacles header
        if obstacles:
            height += len(obstacles) * 20
        if input_mode:
            height += 60  # Input box
        if extra_lines:
            height += len(extra_lines) * 18 + 30
        return height
    
    def _is_visible_in_sidebar(self, y: int, height: int, scroll_start: int, viewport_h: int) -> bool:
        """Check if element at y with given height is visible in sidebar viewport."""
        return (y + height >= scroll_start and y < scroll_start + viewport_h)

    def _sb_divider(self, y: int, sb_x: int, sb_w: int, scroll_offset: int = 0) -> int:
        """Thin horizontal separator line."""
        pygame.draw.rect(self.screen, SB_BORDER, (sb_x + 2, y - scroll_offset, sb_w - 4, 1))
        return y + 9

    def _sb_section_header(self, y: int, pad: int, title: str, scroll_offset: int = 0) -> int:
        """Section label with a left-side teal pip."""
        y_draw = y - scroll_offset
        pygame.draw.rect(self.screen, SB_ACCENT, (pad, y_draw + 1, 3, 14))
        surf, surf_rect = self.font_sec.render(title, SB_ACCENT)
        self.screen.blit(surf, (pad + 8, y_draw))
        return y + 22

    def _sb_kv_row(self, y: int, pad: int, rpad: int,
                   key: str, value: str,
                   val_color: tuple = SB_TEXT, scroll_offset: int = 0) -> int:
        """Left-aligned dim label, right-aligned bold value."""
        y_draw = y - scroll_offset
        k, k_rect = self.font_label.render(key.upper(), SB_TEXT_DIM)
        v, v_rect = self.font_value.render(value, val_color)
        self.screen.blit(k, (pad + 8, y_draw + 1))
        self.screen.blit(v, (rpad - v_rect.width, y_draw))
        return y + 17

    def _sb_status_pill(self, x: int, y: int, label: str,
                         active: bool,
                         on_color:  tuple,
                         off_color: tuple = SB_BTN_INACT,
                         scroll_offset: int = 0) -> int:
        """Inline rounded pill indicator. Returns pill width."""
        y_draw = y - scroll_offset
        col   = on_color  if active else off_color
        tc    = SB_BG     if active else SB_TEXT_DIM
        surf, surf_rect = self.font_label.render(label, tc)
        pw    = surf_rect.width + 10
        ph    = 14
        pygame.draw.rect(self.screen, col,
                         pygame.Rect(x, y_draw, pw, ph), border_radius=7)
        if not active:
            pygame.draw.rect(self.screen, SB_BORDER,
                             pygame.Rect(x, y_draw, pw, ph), 1, border_radius=7)
        self.screen.blit(surf, (x + 5, y_draw + 1))
        return pw

    def _sb_robot_card(self, y: int, pad: int, rpad: int,
                        rinfo: dict, ally: bool, scroll_offset: int = 0) -> int:
        """Compact robot data card. Returns new y."""
        name_col = SB_GREEN if ally else SB_RED
        dot_col  = SB_GREEN if ally else SB_RED

        # Name row with colored dot indicator
        y_draw = y - scroll_offset
        pygame.draw.circle(self.screen, dot_col, (pad + 5, y_draw + 7), 4)
        name_surf, name_rect = self.font_sec.render(rinfo['name'], name_col)
        self.screen.blit(name_surf, (pad + 15, y_draw))
        y += 20

        y = self._sb_kv_row(y, pad, rpad, "Position",
                             f"({rinfo['x']:.2f}, {rinfo['y']:.2f})",
                             scroll_offset=scroll_offset)
        y = self._sb_kv_row(y, pad, rpad, "Theta",
                             f"{math.degrees(rinfo['theta']):.1f} deg",
                             scroll_offset=scroll_offset)

        if ally:
            y = self._sb_kv_row(y, pad, rpad, "Velocity",
                                  f"({rinfo['vx']:.2f}, {rinfo['vy']:.2f})",
                                  scroll_offset=scroll_offset)
            y = self._sb_kv_row(y, pad, rpad, "Strategy",
                                  rinfo.get('strategy', '-'),
                                  val_color=SB_ACCENT, scroll_offset=scroll_offset)
            # Status pills
            px = pad + 8
            px += self._sb_status_pill(
                px, y, "READY",
                rinfo.get('is_ready',   False), SB_GREEN, scroll_offset=scroll_offset)  + 5
            px += self._sb_status_pill(
                px, y, "GRIP",
                rinfo.get('is_gripped', False), SB_AMBER, scroll_offset=scroll_offset)  + 5
            self._sb_status_pill(
                px, y, "STOP",
                rinfo.get('is_stopped', False), SB_RED, scroll_offset=scroll_offset)
            y += 20

        y += 4
        return y

    # ------------------------------------------------------------------

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

    def check_obstacle_checkbox_click(self, pos: tuple[int, int],
                                       obstacles: list) -> int | None:
        """Cek apakah klik mengenai checkbox obstacle. Return index atau None."""
        for i, rect in enumerate(self._obstacle_checkbox_rects):
            if rect.collidepoint(pos):
                return i
        return None

    def check_obstacle_delete_click(self, pos: tuple[int, int]) -> int | None:
        """Return index of obstacle whose delete button was clicked."""
        for i, rect in enumerate(self._obstacle_delete_rects):
            if rect.collidepoint(pos):
                return i
        return None

    def check_add_obstacle_click(self, pos: tuple[int, int]) -> bool:
        """Return True if the '+' add obstacle button was clicked."""
        return self.btn_add_obstacle_rect.collidepoint(pos)

    def check_wheel_speed_toggle_click(self, pos: tuple[int, int]) -> bool:
        """Return True if the wheel speed toggle was clicked."""
        return self.btn_wheel_speed_rect.collidepoint(pos)

    def check_obs_coords_toggle_click(self, pos: tuple[int, int]) -> bool:
        """Return True if the obstacle coords toggle was clicked."""
        return self.btn_obs_coords_rect.collidepoint(pos)

    def draw_obstacles(self, obstacles: list):
        fc = self.fc
        for obs in obstacles:
            if obs.enabled:
                half = obs.size / 2.0
                rect = fc.world_rect_to_screen(
                    obs.x - half, obs.y - half, obs.size, obs.size)
                pygame.draw.rect(self.screen, COLOR_OBSTACLE, rect)

                # Draw coordinate label on field if toggle is on
                if self.show_obstacle_coords:
                    sx, sy = fc.world_to_screen(obs.x, obs.y)
                    coord_txt = f"({obs.x:.1f},{obs.y:.1f})"
                    coord_surf, coord_rect = self.font_label.render(coord_txt, COLOR_WHITE)
                    self.screen.blit(coord_surf,
                                     (sx - coord_rect.width // 2,
                                      sy - fc.world_length_to_px(obs.size / 2) - 14))

    # ==================================================================
    # Debug visualization
    # ==================================================================

    def draw_margin_debug(self):
        """Gambar garis batas margin untuk debugging."""
        fc = self.fc
        f  = self.field
        COLOR_MARGIN = (255, 0, 255)

        if self.mode == SimMode.REGIONAL:
            tl = fc.world_to_screen(0, 0)
            br = fc.world_to_screen(f.width, f.length)
            mx = fc.margin_screen_x * fc.scale
            my = fc.margin_screen_y * fc.scale
            pygame.draw.rect(self.screen, COLOR_MARGIN,
                             (int(tl[0]-mx), int(tl[1]-my),
                              int(br[0]-tl[0]+2*mx),
                              int(br[1]-tl[1]+2*my)), 4)
        else:
            tl = fc.world_to_screen(0, f.width)
            br = fc.world_to_screen(f.length, 0)
            mx = fc.margin_screen_x * fc.scale
            my = fc.margin_screen_y * fc.scale
            pygame.draw.rect(self.screen, COLOR_MARGIN,
                             (int(tl[0]-mx), int(tl[1]-my),
                              int(br[0]-tl[0]+2*mx),
                              int(br[1]-tl[1]+2*my)), 2)

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
        cr_px     = fc.world_length_to_px(f.center_circle_radius)
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
        g_br = fc.world_to_screen(0,           f.goal_left_line_y1)
        g_tr = fc.world_to_screen(0,           f.goal_left_line_y2)
        pygame.draw.line(self.screen, COLOR_GOAL_NET, g_bl, g_tl, 4)
        pygame.draw.line(self.screen, COLOR_GOAL_NET, g_bl, g_br, 4)
        pygame.draw.line(self.screen, COLOR_GOAL_NET, g_tl, g_tr, 4)
        pygame.draw.line(self.screen, COLOR_GOAL_LINE, g_br, g_tr, 5)

        # ── Gawang KANAN ──
        g_bl2 = fc.world_to_screen(f.length,              f.goal_right_line_y1)
        g_tl2 = fc.world_to_screen(f.length,              f.goal_right_line_y2)
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