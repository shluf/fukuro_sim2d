"""
simulation_node.py — ROS2 Node utama simulasi 2D.

Mendukung 2 mode:
  - REGIONAL : 8×6 m, obstacle, 2 robot (r2, r3)
  - NASIONAL : 12×8 m, 3 robot (r1=keeper, r2=striker, r3=striker) + 3 lawan
"""

import math
import json

import pygame
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from geometry_msgs.msg import Twist, Pose2D
from std_msgs.msg import Bool

from fukuro_interface.msg import WorldState, StrategyState, Robot as RobotMsg, Obstacle
from fukuro_interface.srv import DribblerControl, KickService, SetReady
from cob_srvs.srv import SetString

from fukuro_sim2d.physics.robot_model import RobotModel
from fukuro_sim2d.physics.ball_model import BallModel
from fukuro_sim2d.objects.field import (
    SimMode,
    RegionalFieldConfig, RegionalObstacleConfig,
    NasionalFieldConfig, NasionalObstacleConfig,
)
from fukuro_sim2d.render.renderer import Renderer, COLOR_ROBOT_BODY, COLOR_ENEMY_BODY


# ======================================================================
# DraggableObstacle
# ======================================================================

class DraggableObstacle:
    """Obstacle yang bisa di-drag dan di-toggle visibility."""
    
    def __init__(self, x: float, y: float, size: float = 0.4):
        self.x = x
        self.y = y
        self.size = size
        self.enabled = True  # Checkbox state
    
    def to_obstacle_msg(self):
        """Convert ke ROS Obstacle message."""
        obs = Obstacle()
        half = self.size / 2.0
        obs.position.position.x = self.x - half
        obs.position.position.y = self.y - half
        obs.position.orientation.w = 1.0
        obs.width = self.size
        obs.length = self.size
        return obs


# ======================================================================
# RobotAgent — robot kawan dengan ROS2 interfaces
# ======================================================================

class RobotAgent:
    """Robot kawan dengan cmd_vel, services (dribbler, kick, set_ready, is_stop)."""

    def __init__(self, name: str, node: 'SimulationNode',
                 model: RobotModel, display_name: str, role: str = "striker"):
        self.name = name
        self.node = node
        self.model = model
        self.display_name = display_name
        self.role = role

        self.current_strategy = "idle"
        self.current_state = "idle"
        self.is_stopped = False  # State untuk is_stop

        cb = node.callback_group

        self.vel_sub = node.create_subscription(
            Twist, f'/{name}/cmd_vel',
            self._vel_callback, 10, callback_group=cb)

        self.dribbler_srv = node.create_service(
            DribblerControl, f'/{name}/fukuro/controller/dribbler',
            self._handle_dribbler, callback_group=cb)

        self.kick_srv = node.create_service(
            KickService, f'/{name}/fukuro/controller/kick',
            self._handle_kick, callback_group=cb)

        self.set_ready_srv = node.create_service(
            SetReady, f'/{name}/fukuro/strategy/set_ready',
            self._handle_set_ready, callback_group=cb)

        # Service is_stop
        self.stop_srv = node.create_service(
            SetReady, f'/{name}/fukuro/controller/is_stop',
            self._handle_is_stop, callback_group=cb)

        self.world_state_pub = node.create_publisher(
            WorldState, f'/{name}/fukuro/world_model/state', 10)

    def _vel_callback(self, msg: Twist):
        # Jika robot di-stop, abaikan cmd_vel
        if self.is_stopped:
            self.model.set_velocity(0, 0, 0)
        else:
            self.model.set_velocity(msg.linear.x, msg.linear.y, msg.angular.z)

    def _handle_dribbler(self, req, res):
        if req.is_active:
            self.model.activate_dribbler(req.dribbler_pwm)
            self.node.get_logger().info(
                f"[{self.name}] Dribbler ON, PWM={req.dribbler_pwm}")
        else:
            self.model.deactivate_dribbler()
            self.node.get_logger().info(f"[{self.name}] Dribbler OFF")
        res.success = True
        return res

    def _handle_kick(self, req, res):
        m = self.model
        if m.is_gripped:
            MAX_KICK_SPEED = 5.0
            kick_speed = (req.kick_power / 255.0) * MAX_KICK_SPEED
            ball = self.node.ball
            ball.kick(angle=m.theta, speed=kick_speed)
            m.is_gripped = False
            m.deactivate_dribbler()
            self.node.get_logger().info(
                f"[{self.name}] Kick! power={req.kick_power:.0f}, "
                f"speed={kick_speed:.2f} m/s")
            res.kick_done = True
        else:
            self.node.get_logger().warn(
                f"[{self.name}] Kick gagal: bola tidak dalam grip!")
            res.kick_done = False
        return res

    def _handle_set_ready(self, req, res):
        self.model.is_ready = req.is_ready
        self.node.get_logger().info(
            f"[{self.name}] is_ready = {self.model.is_ready}")
        res.success = True
        return res

    def _handle_is_stop(self, req, res):
        """Service untuk toggle stop state robot."""
        self.is_stopped = req.is_ready  # Reuse SetReady.srv dengan is_ready sebagai stop flag
        if self.is_stopped:
            self.model.set_velocity(0, 0, 0)
            self.node.get_logger().info(f"[{self.name}] STOPPED")
        else:
            self.node.get_logger().info(f"[{self.name}] RESUMED")
        res.success = True
        return res

    def set_strategy_info(self, strategy: str, state: str):
        self.current_strategy = strategy
        self.current_state = state

    def info_dict(self) -> dict:
        m = self.model
        return {
            "name": self.display_name,
            "x": m.x, "y": m.y, "theta": m.theta,
            "vx": m.vx, "vy": m.vy, "omega": m.omega,
            "strategy": self.current_strategy,
            "state": self.current_state,
            "is_ready": m.is_ready,
            "is_gripped": m.is_gripped,
            "is_stopped": self.is_stopped,
            "dribbler_pwm": m.dribbler_pwm,
        }


# ======================================================================
# EnemyRobot
# ======================================================================

class EnemyRobot:
    """Robot lawan (draggable)."""

    def __init__(self, name: str, model: RobotModel, display_name: str):
        self.name = name
        self.model = model
        self.display_name = display_name

    def info_dict(self) -> dict:
        m = self.model
        return {
            "name": self.display_name,
            "x": m.x, "y": m.y, "theta": m.theta,
        }


# ======================================================================
# SimulationNode
# ======================================================================

class SimulationNode(Node):
    """ROS2 Node utama simulasi 2D."""

    def __init__(self):
        super().__init__('simulation_node')
        self.callback_group = ReentrantCallbackGroup()
        self.Ts = 0.033

        # Mode awal
        self.mode = SimMode.REGIONAL

        # Robot models & agents (akan dibuat di _create_ros_agents)
        self.robot1_model: RobotModel = None
        self.robot2_model: RobotModel = None
        self.robot3_model: RobotModel = None
        self.agent1: RobotAgent = None
        self.agent2: RobotAgent = None
        self.agent3: RobotAgent = None
        self.ball: BallModel = None

        # Enemy robots (mode nasional)
        self.enemies: list[EnemyRobot] = []

        # Custom obstacles (draggable)
        self.custom_obstacles: list[DraggableObstacle] = []

        # Field config
        self.field_cfg = None
        self.obs_cfg = None

        # Buat ROS agents
        self._create_ros_agents()

        # Setup mode awal
        self._setup_mode(self.mode)

        # Strategy subscriber
        self.strategy_sub = self.create_subscription(
            StrategyState, '/fukuro/strategy/goal',
            self._strategy_callback, 10,
            callback_group=self.callback_group)

        # Strategy change service client
        self.strategy_change_client = self.create_client(
            SetString, '/fukuro/strategy/change',
            callback_group=self.callback_group)

        # Renderer
        self.renderer = Renderer(
            screen_width=1500, screen_height=900,
            mode=self.mode)

        # Timer
        self.timer = self.create_timer(
            self.Ts, self._update_simulation,
            callback_group=self.callback_group)

        self.get_logger().info(
            f"SimulationNode initialized — mode={self.mode.value}")

    # ------------------------------------------------------------------
    # Mode setup
    # ------------------------------------------------------------------

    def _create_ros_agents(self):
        """Buat 3 RobotModel & RobotAgent (hanya sekali)."""
        self.robot1_model = RobotModel(x=0, y=0, theta=0, radius=0.2, wheel_R=0.5, wheel_r=0.05)
        self.robot2_model = RobotModel(x=0, y=0, theta=0, radius=0.2, wheel_R=0.5, wheel_r=0.05)
        self.robot3_model = RobotModel(x=0, y=0, theta=0, radius=0.2, wheel_R=0.5, wheel_r=0.05)
        self.ball = BallModel(x=0, y=0, radius=0.12, friction=0.98)

        self.agent1 = RobotAgent('r1', self, self.robot1_model, "R1 Keeper", role="keeper")
        self.agent2 = RobotAgent('r2', self, self.robot2_model, "R2 Striker", role="striker")
        self.agent3 = RobotAgent('r3', self, self.robot3_model, "R3 Striker", role="striker")

    def _setup_mode(self, mode: SimMode):
        """Reset posisi robot, bola, enemy, dan obstacle sesuai mode."""
        self.mode = mode

        if mode == SimMode.REGIONAL:
            cfg = RegionalFieldConfig()
            self.obs_cfg = RegionalObstacleConfig()
            self.field_cfg = cfg

            # Regional: hanya r2 dan r3
            # r1 disabled (posisi di luar lapangan atau tidak aktif)
            self.robot1_model.x, self.robot1_model.y, self.robot1_model.theta = -10, -10, 0
            x2, y2, th2 = cfg.robot1_start  # r2 ambil posisi robot1_start
            x3, y3, th3 = cfg.robot2_start  # r3 ambil posisi robot2_start
            bx, by = cfg.kickoff_position

            self.enemies = []

            # Load default obstacles dari config
            self.custom_obstacles = [
                DraggableObstacle(ox, oy, self.obs_cfg.size)
                for ox, oy in self.obs_cfg.positions
            ]

        else:  # NASIONAL
            cfg = NasionalFieldConfig()
            self.obs_cfg = NasionalObstacleConfig()
            self.field_cfg = cfg

            # Nasional: r1 keeper, r2 striker, r3 striker
            x1, y1, th1 = cfg.robot1_start
            x2, y2, th2 = cfg.robot2_start
            # r3 posisi custom
            x3, y3, th3 = (3.0, 5.5, 0.0)
            bx, by = cfg.kickoff_position

            # Enemy robots
            e1x, e1y, e1th = cfg.enemy1_start
            e2x, e2y, e2th = cfg.enemy2_start
            e3x, e3y, e3th = cfg.enemy3_start
            self.enemies = [
                EnemyRobot("e1",
                           RobotModel(e1x, e1y, e1th, radius=0.2, wheel_R=0.5, wheel_r=0.05),
                           "Enemy 1"),
                EnemyRobot("e2",
                           RobotModel(e2x, e2y, e2th, radius=0.2, wheel_R=0.5, wheel_r=0.05),
                           "Enemy 2"),
                EnemyRobot("e3",
                           RobotModel(e3x, e3y, e3th, radius=0.2, wheel_R=0.5, wheel_r=0.05),
                           "Enemy 3"),
            ]

            # Nasional: tidak ada obstacle default
            self.custom_obstacles = []

        # Reset robot positions
        if mode == SimMode.NASIONAL:
            self.robot1_model.x, self.robot1_model.y, self.robot1_model.theta = x1, y1, th1
        self.robot1_model.vx = self.robot1_model.vy = self.robot1_model.omega = 0.0
        self.robot1_model.is_gripped = False
        self.robot1_model.deactivate_dribbler()
        self.agent1.is_stopped = False

        self.robot2_model.x, self.robot2_model.y, self.robot2_model.theta = x2, y2, th2
        self.robot2_model.vx = self.robot2_model.vy = self.robot2_model.omega = 0.0
        self.robot2_model.is_gripped = False
        self.robot2_model.deactivate_dribbler()
        self.agent2.is_stopped = False

        self.robot3_model.x, self.robot3_model.y, self.robot3_model.theta = x3, y3, th3
        self.robot3_model.vx = self.robot3_model.vy = self.robot3_model.omega = 0.0
        self.robot3_model.is_gripped = False
        self.robot3_model.deactivate_dribbler()
        self.agent3.is_stopped = False

        # Reset ball
        self.ball.set_position(bx, by)

    def switch_mode(self, new_mode: SimMode):
        if new_mode != self.mode:
            self._setup_mode(new_mode)
            self.renderer.switch_mode(new_mode)
            self.get_logger().info(f"Mode switched to: {new_mode.value}")

    # ------------------------------------------------------------------
    # Strategy callback
    # ------------------------------------------------------------------

    def _strategy_callback(self, msg: StrategyState):
        if msg.is_local:
            target = None
            if "r1" in msg.strategy_name.lower():
                target = self.agent1
            elif "r2" in msg.strategy_name.lower():
                target = self.agent2
            elif "r3" in msg.strategy_name.lower():
                target = self.agent3

            if target:
                target.model.set_velocity(
                    msg.local_nav.x, msg.local_nav.y, msg.local_nav.theta)
                target.set_strategy_info(msg.strategy_name, msg.present_state)

    # ------------------------------------------------------------------
    # Physics update
    # ------------------------------------------------------------------

    def _update_simulation(self):
        Ts = self.Ts
        
        # Update ally robots
        agents = [self.agent1, self.agent2, self.agent3]
        if self.mode == SimMode.REGIONAL:
            agents = [self.agent2, self.agent3]  # Skip r1 di regional
        
        for agent in agents:
            agent.model.update(Ts)

        # Update enemies
        for enemy in self.enemies:
            enemy.model.update(Ts)

        # Reset grip
        for agent in agents:
            agent.model.is_gripped = False

        ball_gripped = False

        # Update ball first (if not gripped)
        if not ball_gripped:
            self.ball.update(Ts)

        # Handle physics collisions BEFORE grip check
        self._handle_collisions()

        # Check dribbler grip AFTER collision (bola sudah dekat & stabil)
        for agent in agents:
            m = agent.model
            if m.dribbler_active:
                # Hitung posisi bola relatif terhadap robot
                dx = self.ball.x - m.x
                dy = self.ball.y - m.y
                dist = math.hypot(dx, dy)
                
                # Cek jarak center-to-center
                if dist < m.radius + self.ball.radius + 0.03:  # Tambah toleransi
                    # Cek apakah bola di depan robot (dalam "mulut" dribbler)
                    cos_th = math.cos(m.theta)
                    sin_th = math.sin(m.theta)
                    # Transform bola ke robot frame
                    forward = dx * cos_th + dy * sin_th  # X robot (maju)
                    lateral = -dx * sin_th + dy * cos_th  # Y robot (lateral)
                    
                    # Bola harus di depan (forward > 0) dan tidak terlalu samping
                    if forward > 0 and abs(lateral) < m.radius * 0.8:
                        self.ball.grip_to(m)
                        m.is_gripped = True
                        ball_gripped = True
                        break

        # Check for goals (update scoreboard)
        self._check_goals()

        self._publish_world_states()

    # ------------------------------------------------------------------
    # Goal detection
    # ------------------------------------------------------------------

    def _check_goals(self):
        """Cek apakah bola melewati gawang dan update scoreboard."""
        bx, by = self.ball.x, self.ball.y
        
        if self.mode == SimMode.REGIONAL:
            # Regional: hanya team score, gawang di X=0 (kiri), Y antara 3-5
            f = self.field_cfg
            if bx < -0.1:  # Bola melewati garis gawang
                if f.goal_line_y1 <= by <= f.goal_line_y2:
                    # Goal!
                    self.renderer.team_score += 1
                    self.get_logger().info(f"GOAL! Team score: {self.renderer.team_score}")
                    # Reset bola ke kickoff
                    self.ball.set_position(*f.kickoff_position)
        
        else:  # NASIONAL
            # Nasional: team vs enemy
            # Team gawang di x=0, enemy gawang di x=12
            f = self.field_cfg
            
            # Team mencetak gol (bola masuk gawang enemy di x=12)
            if bx > f.length + 0.1:
                if f.goal_right_line_y1 <= by <= f.goal_right_line_y2:
                    self.renderer.team_score += 1
                    self.get_logger().info(
                        f"GOAL! Team scores! {self.renderer.team_score} - {self.renderer.enemy_score}")
                    self.ball.set_position(*f.kickoff_position)
            
            # Enemy mencetak gol (bola masuk gawang team di x=0)
            elif bx < -0.1:
                if f.goal_left_line_y1 <= by <= f.goal_left_line_y2:
                    self.renderer.enemy_score += 1
                    self.get_logger().info(
                        f"GOAL! Enemy scores! {self.renderer.team_score} - {self.renderer.enemy_score}")
                    self.ball.set_position(*f.kickoff_position)

    # ------------------------------------------------------------------
    # World state publisher
    # ------------------------------------------------------------------

    def _publish_world_states(self):
        obs_list = self._build_obstacle_msgs()
        enemy_msgs = self._build_enemy_msgs()

        bx, by = self.ball.x, self.ball.y

        agents = [self.agent1, self.agent2, self.agent3]
        if self.mode == SimMode.REGIONAL:
            agents = [self.agent2, self.agent3]

        for agent in agents:
            m = agent.model
            
            # Bola dalam robot frame
            dx = bx - m.x
            dy = by - m.y
            cos_th = math.cos(m.theta)
            sin_th = math.sin(m.theta)
            forward = dx * cos_th + dy * sin_th
            lateral = -dx * sin_th + dy * cos_th
            bearing = math.atan2(lateral, forward) if (forward or lateral) else 0.0

            ws = WorldState()
            ws.robot_name = agent.name
            ws.robot_role = agent.role

            ws.posisi_diri = Pose2D(x=m.x, y=m.y, theta=m.theta)
            ws.bola = Pose2D(x=forward, y=lateral, theta=bearing)

            # Kawan (tim list)
            tim_list = []
            for ally in agents:
                if ally.name != agent.name:
                    ally_msg = RobotMsg()
                    ally_msg.robot_pose = Pose2D(
                        x=ally.model.x, y=ally.model.y, theta=ally.model.theta)
                    ally_msg.robot_role = ally.role
                    ally_msg.is_ready = ally.model.is_ready
                    tim_list.append(ally_msg)
            ws.tim = tim_list

            ws.is_grip = m.is_gripped
            ws.is_ready = m.is_ready
            ws.obstacles = obs_list

            # Enemy list (mode nasional)
            if self.mode == SimMode.NASIONAL:
                ws.enemy = enemy_msgs

            agent.world_state_pub.publish(ws)

    def _build_obstacle_msgs(self) -> list:
        """Build obstacle msgs hanya dari custom obstacles yang enabled."""
        msgs = []
        for obs in self.custom_obstacles:
            if obs.enabled:
                msgs.append(obs.to_obstacle_msg())
        return msgs

    def _build_enemy_msgs(self) -> list:
        msgs = []
        for enemy in self.enemies:
            em = RobotMsg()
            em.robot_pose = Pose2D(
                x=enemy.model.x, y=enemy.model.y, theta=enemy.model.theta)
            em.robot_role = "enemy"
            em.is_ready = False
            msgs.append(em)
        return msgs

    # ------------------------------------------------------------------
    # Collision detection and response
    # ------------------------------------------------------------------

    def _is_ball_gripped(self) -> bool:
        """Check if ball is gripped by any robot."""
        robots = [self.robot1_model, self.robot2_model, self.robot3_model]
        if self.mode == SimMode.REGIONAL:
            robots = [self.robot2_model, self.robot3_model]
        return any(r.is_gripped for r in robots)

    def _check_circle_collision(self, x1: float, y1: float, r1: float,
                                x2: float, y2: float, r2: float) -> bool:
        """Check if two circles collide."""
        dist = math.hypot(x2 - x1, y2 - y1)
        return dist < (r1 + r2)

    def _resolve_circle_collision(self, obj1, obj2, r1: float, r2: float,
                                  elasticity: float = 0.5):
        """Resolve collision between two circular objects."""
        dx = obj2.x - obj1.x
        dy = obj2.y - obj1.y
        dist = math.hypot(dx, dy)
        
        if dist < 1e-6:
            return
        
        # Normalize collision vector
        nx = dx / dist
        ny = dy / dist
        
        # Get velocities (handle BallModel's speed/angle)
        if isinstance(obj1, BallModel):
            v1x = math.cos(obj1.angle) * obj1.speed
            v1y = math.sin(obj1.angle) * obj1.speed
        else:
            v1x, v1y = obj1.vx, obj1.vy
        
        if isinstance(obj2, BallModel):
            v2x = math.cos(obj2.angle) * obj2.speed
            v2y = math.sin(obj2.angle) * obj2.speed
        else:
            v2x, v2y = obj2.vx, obj2.vy
        
        # Relative velocity
        dvx = v2x - v1x
        dvy = v2y - v1y
        
        # Velocity along collision normal
        dvn = dvx * nx + dvy * ny
        
        # Do not resolve if objects are separating
        if dvn >= 0:
            return
        
        # Get masses
        m1 = getattr(obj1, 'mass', 1.0)
        m2 = getattr(obj2, 'mass', 1.0)
        total_mass = m1 + m2
        
        # Separate overlapping objects (weighted by mass)
        overlap = (r1 + r2) - dist
        if overlap > 0:
            sep1 = overlap * (m2 / total_mass)
            sep2 = overlap * (m1 / total_mass)
            obj1.x -= nx * sep1
            obj1.y -= ny * sep1
            obj2.x += nx * sep2
            obj2.y += ny * sep2
        
        # Apply impulse (considering mass)
        impulse = -(1 + elasticity) * dvn / (1.0/m1 + 1.0/m2)
        
        # Get masses
        m1 = getattr(obj1, 'mass', 1.0)
        m2 = getattr(obj2, 'mass', 1.0)
        
        # Update velocities
        if isinstance(obj1, BallModel):
            v1x -= (impulse / m1) * nx
            v1y -= (impulse / m1) * ny
            obj1.speed = math.hypot(v1x, v1y)
            if obj1.speed > 1e-6:
                obj1.angle = math.atan2(v1y, v1x)
        else:
            obj1.vx -= (impulse / m1) * nx
            obj1.vy -= (impulse / m1) * ny
        
        if isinstance(obj2, BallModel):
            v2x += (impulse / m2) * nx
            v2y += (impulse / m2) * ny
            obj2.speed = math.hypot(v2x, v2y)
            if obj2.speed > 1e-6:
                obj2.angle = math.atan2(v2y, v2x)
        else:
            obj2.vx += (impulse / m2) * nx
            obj2.vy += (impulse / m2) * ny

    def _check_rect_collision(self, cx: float, cy: float, cr: float,
                             rx: float, ry: float, rw: float, rh: float) -> bool:
        """Check if circle collides with rectangle (obstacle)."""
        # Find closest point on rectangle to circle center
        closest_x = max(rx, min(cx, rx + rw))
        closest_y = max(ry, min(cy, ry + rh))
        
        # Calculate distance
        dist = math.hypot(cx - closest_x, cy - closest_y)
        return dist < cr

    def _resolve_rect_collision(self, obj, cx: float, cy: float, cr: float,
                               rx: float, ry: float, rw: float, rh: float,
                               elasticity: float = 0.3):
        """Resolve collision between circle and rectangle."""
        # Find closest point on rectangle
        closest_x = max(rx, min(cx, rx + rw))
        closest_y = max(ry, min(cy, ry + rh))
        
        dx = cx - closest_x
        dy = cy - closest_y
        dist = math.hypot(dx, dy)
        
        if dist < 1e-6:
            # Circle center inside rectangle - push out in nearest direction
            edges = [
                (cx - rx, -1, 0),           # left
                (rx + rw - cx, 1, 0),       # right
                (cy - ry, 0, -1),           # bottom
                (ry + rh - cy, 0, 1),       # top
            ]
            min_edge = min(edges, key=lambda e: e[0])
            dx, dy = min_edge[1], min_edge[2]
            dist = min_edge[0]
            overlap = cr + dist
        else:
            # Normalize
            dx /= dist
            dy /= dist
            overlap = cr - dist
        
        if overlap > 0:
            # Push object out
            obj.x += dx * overlap
            obj.y += dy * overlap
            
            # Get velocity (handle BallModel's speed/angle)
            if isinstance(obj, BallModel):
                vx = math.cos(obj.angle) * obj.speed
                vy = math.sin(obj.angle) * obj.speed
            else:
                vx, vy = obj.vx, obj.vy
            
            # Reflect velocity
            vn = vx * dx + vy * dy
            if vn < 0:
                vx -= (1 + elasticity) * vn * dx
                vy -= (1 + elasticity) * vn * dy
                
                # Update velocity
                if isinstance(obj, BallModel):
                    obj.speed = math.hypot(vx, vy)
                    if obj.speed > 1e-6:
                        obj.angle = math.atan2(vy, vx)
                else:
                    obj.vx = vx
                    obj.vy = vy

    def _handle_collisions(self):
        """Handle all physics collisions."""
        # Get active robots
        robots = [self.robot1_model, self.robot2_model, self.robot3_model]
        if self.mode == SimMode.REGIONAL:
            robots = [self.robot2_model, self.robot3_model]
        
        # Add enemies
        all_robots = robots + [e.model for e in self.enemies]
        
        # Robot-Robot collisions
        for i, r1 in enumerate(all_robots):
            for r2 in all_robots[i+1:]:
                if self._check_circle_collision(r1.x, r1.y, r1.radius,
                                               r2.x, r2.y, r2.radius):
                    self._resolve_circle_collision(r1, r2, r1.radius, r2.radius,
                                                  elasticity=0.5)
        
        # Robot-Ball collisions (when not gripped)
        if not self._is_ball_gripped():
            for robot in all_robots:
                if self._check_circle_collision(robot.x, robot.y, robot.radius,
                                               self.ball.x, self.ball.y, self.ball.radius):
                    # Cek apakah robot memiliki dribbler aktif dan bola di depan
                    # Jika ya, kurangi elasticity drastis (bola "masuk" ke mulut robot)
                    dx = self.ball.x - robot.x
                    dy = self.ball.y - robot.y
                    cos_th = math.cos(robot.theta)
                    sin_th = math.sin(robot.theta)
                    forward = dx * cos_th + dy * sin_th
                    lateral = -dx * sin_th + dy * cos_th
                    
                    # Jika robot adalah ally dengan dribbler aktif DAN bola di depan
                    is_front_collision = forward > 0 and abs(lateral) < robot.radius * 0.9
                    has_dribbler = hasattr(robot, 'dribbler_active') and robot.dribbler_active
                    
                    if has_dribbler and is_front_collision:
                        # Collision sangat lembut (cekung) - bola "tertahan" di depan
                        self._resolve_circle_collision(robot, self.ball,
                                                      robot.radius, self.ball.radius,
                                                      elasticity=0.05)  # Hampir tidak memantul
                    else:
                        # Collision normal untuk sisi/belakang robot atau robot tanpa dribbler
                        self._resolve_circle_collision(robot, self.ball,
                                                      robot.radius, self.ball.radius,
                                                      elasticity=0.5)
        
        # Robot-Obstacle and Ball-Obstacle collisions
        for obs in self.custom_obstacles:
            if not obs.enabled:
                continue
            
            half = obs.size / 2.0
            ox = obs.x - half
            oy = obs.y - half
            
            # Robot-Obstacle
            for robot in all_robots:
                if self._check_rect_collision(robot.x, robot.y, robot.radius,
                                             ox, oy, obs.size, obs.size):
                    self._resolve_rect_collision(robot, robot.x, robot.y, robot.radius,
                                                ox, oy, obs.size, obs.size,
                                                elasticity=0.3)
            
            # Ball-Obstacle
            if not self._is_ball_gripped():
                if self._check_rect_collision(self.ball.x, self.ball.y, self.ball.radius,
                                             ox, oy, obs.size, obs.size):
                    self._resolve_rect_collision(self.ball, self.ball.x, self.ball.y,
                                                self.ball.radius, ox, oy, obs.size, obs.size,
                                                elasticity=0.6)

    # ------------------------------------------------------------------
    # Strategy change service client
    # ------------------------------------------------------------------

    def _send_strategy_change(self, mode_name: str):
        if not self.strategy_change_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("Strategy change service not available")
            return
        
        req = SetString.Request()
        req.data = json.dumps({"mode": mode_name})
        
        future = self.strategy_change_client.call_async(req)
        future.add_done_callback(
            lambda f: self.get_logger().info(
                f"Strategy → {mode_name}: {'success' if f.result().success else 'failed'}"))

    # ------------------------------------------------------------------
    # Obstacle management
    # ------------------------------------------------------------------

    def add_obstacle(self, x: float, y: float):
        """Tambahkan obstacle baru di koordinat world."""
        obs = DraggableObstacle(x, y, 0.4)
        self.custom_obstacles.append(obs)
        self.get_logger().info(f"Obstacle added at ({x:.2f}, {y:.2f})")

    def remove_last_obstacle(self):
        """Hapus obstacle terakhir."""
        if self.custom_obstacles:
            self.custom_obstacles.pop()
            self.get_logger().info("Last obstacle removed")

    def _find_obstacle_at(self, wx: float, wy: float) -> DraggableObstacle | None:
        """Cari obstacle di posisi klik."""
        for obs in self.custom_obstacles:
            dist = math.hypot(obs.x - wx, obs.y - wy)
            if dist < obs.size / 2 + 0.1:
                return obs
        return None

    def _find_enemy_at(self, wx: float, wy: float) -> EnemyRobot | None:
        """Cari enemy di posisi klik."""
        for enemy in self.enemies:
            dist = math.hypot(enemy.model.x - wx, enemy.model.y - wy)
            if dist < enemy.model.radius + 0.15:
                return enemy
        return None

    # ------------------------------------------------------------------
    # Main loop
    # ------------------------------------------------------------------

    def run(self):
        running = True
        renderer = self.renderer
        dragging_enemy: EnemyRobot | None = None
        dragging_obstacle: DraggableObstacle | None = None
        
        # State untuk input koordinat obstacle
        input_mode = False
        input_text = ""

        while running and rclpy.ok():
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False

                elif event.type == pygame.KEYDOWN:
                    if input_mode:
                        # Mode input koordinat obstacle
                        if event.key == pygame.K_RETURN:
                            try:
                                parts = input_text.split(',')
                                if len(parts) == 2:
                                    x = float(parts[0].strip())
                                    y = float(parts[1].strip())
                                    self.add_obstacle(x, y)
                            except:
                                pass
                            input_mode = False
                            input_text = ""
                        elif event.key == pygame.K_ESCAPE:
                            input_mode = False
                            input_text = ""
                        elif event.key == pygame.K_BACKSPACE:
                            input_text = input_text[:-1]
                        else:
                            input_text += event.unicode
                    else:
                        running = self._handle_key(event.key, running)
                        # Toggle input mode
                        if event.key == pygame.K_o:
                            input_mode = True
                            input_text = ""

                elif event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
                    # Cek scoreboard reset button
                    if renderer.check_scoreboard_reset_click(event.pos):
                        renderer.reset_scoreboard()
                        self.get_logger().info("Scoreboard reset")
                    # Cek klik tombol mode
                    else:
                        new_mode = renderer.check_mode_button_click(event.pos)
                        if new_mode is not None:
                            self.switch_mode(new_mode)
                        # Cek wheel speed toggle
                        elif renderer.check_wheel_speed_toggle_click(event.pos):
                            renderer.show_wheel_speeds = not renderer.show_wheel_speeds
                        # Cek obstacle coords toggle
                        elif renderer.check_obs_coords_toggle_click(event.pos):
                            renderer.show_obstacle_coords = not renderer.show_obstacle_coords
                        # Cek add obstacle button
                        elif renderer.check_add_obstacle_click(event.pos):
                            input_mode = True
                            input_text = ""
                        else:
                            # Cek obstacle delete button clicks
                            del_idx = renderer.check_obstacle_delete_click(event.pos)
                            if del_idx is not None:
                                if 0 <= del_idx < len(self.custom_obstacles):
                                    removed = self.custom_obstacles.pop(del_idx)
                                    self.get_logger().info(
                                        f"Obstacle removed at ({removed.x:.2f}, {removed.y:.2f})")
                            else:
                                # Cek obstacle checkbox clicks
                                clicked_obs_idx = renderer.check_obstacle_checkbox_click(
                                    event.pos, self.custom_obstacles)
                                if clicked_obs_idx is not None:
                                    obs = self.custom_obstacles[clicked_obs_idx]
                                    obs.enabled = not obs.enabled
                                else:
                                    # Cek drag
                                    wx, wy = renderer.fc.screen_to_world(*event.pos)
                                    dragging_obstacle = self._find_obstacle_at(wx, wy)
                                    if dragging_obstacle is None:
                                        dragging_enemy = self._find_enemy_at(wx, wy)
                
                elif event.type == pygame.MOUSEWHEEL:
                    # Scroll sidebar
                    renderer.handle_scroll(event.y)

                elif event.type == pygame.MOUSEBUTTONUP and event.button == 1:
                    dragging_enemy = None
                    dragging_obstacle = None

                elif event.type == pygame.MOUSEMOTION:
                    if dragging_enemy is not None:
                        wx, wy = renderer.fc.screen_to_world(*event.pos)
                        dragging_enemy.model.x = wx
                        dragging_enemy.model.y = wy
                    elif dragging_obstacle is not None:
                        wx, wy = renderer.fc.screen_to_world(*event.pos)
                        dragging_obstacle.x = wx
                        dragging_obstacle.y = wy

            # ROS spin
            rclpy.spin_once(self, timeout_sec=0.001)

            # ── Render ──
            renderer.clear()
            renderer.draw_field()
            renderer.draw_ball(self.ball)

            # Draw obstacles
            renderer.draw_obstacles(self.custom_obstacles)

            # Ally robots
            if self.mode == SimMode.NASIONAL:
                renderer.draw_robot(self.robot1_model, label="R1", color=COLOR_ROBOT_BODY)
            renderer.draw_robot(self.robot2_model, label="R2", color=COLOR_ROBOT_BODY)
            renderer.draw_robot(self.robot3_model, label="R3", color=COLOR_ROBOT_BODY)

            # Enemy robots
            for enemy in self.enemies:
                renderer.draw_robot(enemy.model, label=enemy.display_name,
                                    color=COLOR_ENEMY_BODY)

            # ── Scoreboard ──
            renderer.draw_scoreboard()

            # ── Sidebar ──
            ally_infos = []
            if self.mode == SimMode.NASIONAL:
                ally_infos.append(self.agent1.info_dict())
            ally_infos.append(self.agent2.info_dict())
            ally_infos.append(self.agent3.info_dict())

            enemy_infos = [e.info_dict() for e in self.enemies]
            ball_info = {"x": self.ball.x, "y": self.ball.y, "speed": self.ball.speed}

            keybindings = [
                "── Ball ──",
                "Z:kickoff  X:corner-L  C:corner-R",
                "",
                "── Robot Actions ──",
                "SPACE:kick R2  ENTER:kick R3",
                "A:dribbler R2  S:dribbler R3",
                "D:dribbler R1 (nasional)",
                "",
                "── Robot Control ──",
                "Q:stop R2  W:stop R3  E:stop R1",
                "",
                "── Strategy ──",
                "1:kickoff_kanan  2:kickoff_kiri",
                "0:idle mode",
                "",
                "── Other ──",
                "BACKSPACE:quit simulation",
            ]
            if self.mode == SimMode.NASIONAL:
                keybindings.append("Mouse:drag enemies & obstacles")

            renderer.draw_sidebar(
                mode=self.mode,
                ally_robots=ally_infos,
                enemy_robots=enemy_infos,
                ball_info=ball_info,
                obstacles=self.custom_obstacles,
                extra_lines=keybindings,
                input_mode=input_mode,
                input_text=input_text,
            )

            renderer.flip(fps=60)

        pygame.quit()

    def _handle_key(self, key, running: bool) -> bool:
        cfg = self.field_cfg

        if key == pygame.K_BACKSPACE:
            return False

        # Ball resets
        elif key == pygame.K_z:
            self.ball.set_position(*cfg.kickoff_position)
        elif key == pygame.K_x:
            if self.mode == SimMode.REGIONAL:
                self.ball.set_position(*cfg.corner_left_position)
            else:
                self.ball.set_position(*cfg.corner_bl)
        elif key == pygame.K_c:
            if self.mode == SimMode.REGIONAL:
                self.ball.set_position(*cfg.corner_right_position)
            else:
                self.ball.set_position(*cfg.corner_tr)

        # Kicks
        elif key == pygame.K_SPACE:
            if self.robot2_model.is_gripped:
                req = KickService.Request()
                req.kick_power = 200
                self.agent2._handle_kick(req, KickService.Response())
        elif key == pygame.K_RETURN:
            if self.robot3_model.is_gripped:
                req = KickService.Request()
                req.kick_power = 200
                self.agent3._handle_kick(req, KickService.Response())

        # Dribbler toggle (A=r2, S=r3, D=r1)
        elif key == pygame.K_a:
            self._toggle_dribbler(self.agent2)
        elif key == pygame.K_s:
            self._toggle_dribbler(self.agent3)
        elif key == pygame.K_d:
            if self.mode == SimMode.NASIONAL:
                self._toggle_dribbler(self.agent1)

        # Stop toggle (Q=r2, W=r3, E=r1)
        elif key == pygame.K_q:
            self._toggle_stop(self.agent2)
        elif key == pygame.K_w:
            self._toggle_stop(self.agent3)
        elif key == pygame.K_e:
            if self.mode == SimMode.NASIONAL:
                self._toggle_stop(self.agent1)

        # Obstacle management
        elif key == pygame.K_DELETE:
            self.remove_last_obstacle()

        # Strategy
        elif key == pygame.K_1:
            self._send_strategy_change("kickoff_kanan")
        elif key == pygame.K_2:
            self._send_strategy_change("kickoff_kiri")
        elif key == pygame.K_0:
            self._send_strategy_change("idle")

        return running

    def _toggle_dribbler(self, agent: RobotAgent):
        """Toggle dribbler untuk robot."""
        if agent.model.dribbler_active:
            req = DribblerControl.Request()
            req.is_active = False
            req.dribbler_pwm = 0
            agent._handle_dribbler(req, DribblerControl.Response())
        else:
            req = DribblerControl.Request()
            req.is_active = True
            req.dribbler_pwm = 150
            agent._handle_dribbler(req, DribblerControl.Response())

    def _toggle_stop(self, agent: RobotAgent):
        """Toggle stop state untuk robot."""
        req = SetReady.Request()
        req.is_ready = not agent.is_stopped
        agent._handle_is_stop(req, SetReady.Response())


# ======================================================================
# Entry point
# ======================================================================

def main():
    rclpy.init()
    try:
        node = SimulationNode()
        node.get_logger().info("Starting simulation...")
        node.run()
    except Exception as e:
        import traceback
        print(f"Error in simulation: {e}")
        traceback.print_exc()
    finally:
        if rclpy.ok():
            rclpy.shutdown()
            print("ROS2 shutdown complete")


if __name__ == '__main__':
    main()
