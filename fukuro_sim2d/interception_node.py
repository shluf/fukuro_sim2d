import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from fukuro_interface.msg import WorldState
from fukuro_interface.srv import DribblerControl
import math

class BallInterceptionNode(Node):
    def __init__(self):
        super().__init__('ball_interception_node')
        
        self.cmd_vel_pub = self.create_publisher(Twist, '/r2/cmd_vel', 10)
        self.state_sub = self.create_subscription(
            WorldState,
            '/r2/fukuro/world_model/state',
            self.state_callback,
            10
        )

        # Dribbler service client
        self.dribbler_client = self.create_client(DribblerControl, '/r2/fukuro/controller/dribbler')
        self.dribbler_active = False
        self.dribbler_distance = 1.5  # aktifkan dribbler kalau bola dalam jarak 1.5m

        self.Kp_linear = 2.0
        self.Kp_angular = 3.0
        self.max_linear_speed = 3.0
        self.max_angular_speed = 3.14
        self.stop_distance = 0.2

        # Estimasi velocity bola dalam world frame
        self.prev_bx_world = None
        self.prev_by_world = None
        self.prev_time = None
        self.est_bvx = 0.0
        self.est_bvy = 0.0
        self.alpha = 0.3
        self.ball_speed_threshold = 1.0

    def activate_dribbler(self):
        if not self.dribbler_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Dribbler service not available')
            return
        req = DribblerControl.Request()
        req.is_active = True
        req.dribbler_pwm = 150
        self.dribbler_client.call_async(req)
        self.dribbler_active = True
        self.get_logger().info('Dribbler activated')

    def deactivate_dribbler(self):
        if not self.dribbler_client.wait_for_service(timeout_sec=1.0):
            return
        req = DribblerControl.Request()
        req.is_active = False
        req.dribbler_pwm = 0
        self.dribbler_client.call_async(req)
        self.dribbler_active = False
        self.get_logger().info('Dribbler deactivated')

    def state_callback(self, msg):
        rx = msg.posisi_diri.x
        ry = msg.posisi_diri.y
        rtheta = msg.posisi_diri.theta

        # Konversi bola dari body frame ke world frame
        bx_local = msg.bola.x
        by_local = msg.bola.y
        bx_world = rx + bx_local * math.cos(rtheta) - by_local * math.sin(rtheta)
        by_world = ry + bx_local * math.sin(rtheta) + by_local * math.cos(rtheta)

        if msg.is_grip:
            self.stop_robot()
            self._reset_estimator(bx_world, by_world)
            return

        distance_to_ball = math.hypot(bx_world - rx, by_world - ry)
        if distance_to_ball < self.stop_distance:
            self.stop_robot()
            return

        # Estimasi velocity bola dalam world frame
        now = self.get_clock().now().nanoseconds / 1e9
        if self.prev_bx_world is not None and self.prev_time is not None:
            dt = now - self.prev_time
            if dt > 0.001:
                raw_vx = (bx_world - self.prev_bx_world) / dt
                raw_vy = (by_world - self.prev_by_world) / dt
                self.est_bvx = self.alpha * raw_vx + (1 - self.alpha) * self.est_bvx
                self.est_bvy = self.alpha * raw_vy + (1 - self.alpha) * self.est_bvy

        self.prev_bx_world = bx_world
        self.prev_by_world = by_world
        self.prev_time = now

        ball_speed = math.hypot(self.est_bvx, self.est_bvy)

        cmd = Twist()

        if ball_speed > self.ball_speed_threshold:
            # Bola bergerak — prediksi titik intersepsi dan bergerak ke sana
            lookahead_time = distance_to_ball / self.max_linear_speed
            target_wx = bx_world + self.est_bvx * lookahead_time
            target_wy = by_world + self.est_bvy * lookahead_time

            # Hadap arah datangnya bola
            ball_dir = math.atan2(self.est_bvy, self.est_bvx)
            target_theta = ball_dir + math.pi

            # Transformasi error ke body frame
            error_wx = target_wx - rx
            error_wy = target_wy - ry
            error_local_x =  error_wx * math.cos(rtheta) + error_wy * math.sin(rtheta)
            error_local_y = -error_wx * math.sin(rtheta) + error_wy * math.cos(rtheta)

            error_theta = math.atan2(
                math.sin(target_theta - rtheta),
                math.cos(target_theta - rtheta)
            )

            vx = self.Kp_linear * error_local_x
            vy = self.Kp_linear * error_local_y
            wz = self.Kp_angular * error_theta

            # Aktifkan dribbler kalau bola sudah dekat
            if distance_to_ball < self.dribbler_distance and not self.dribbler_active:
                self.activate_dribbler()
            elif distance_to_ball >= self.dribbler_distance and self.dribbler_active:
                self.deactivate_dribbler()

        else:
            # Bola diam — robot diam, hanya hadap ke bola, dribbler off
            target_theta = math.atan2(by_world - ry, bx_world - rx)
            error_theta = math.atan2(
                math.sin(target_theta - rtheta),
                math.cos(target_theta - rtheta)
            )

            vx = 0.0
            vy = 0.0
            wz = self.Kp_angular * error_theta

            if self.dribbler_active:
                self.deactivate_dribbler()

        cmd.linear.x = max(min(vx, self.max_linear_speed), -self.max_linear_speed)
        cmd.linear.y = max(min(vy, self.max_linear_speed), -self.max_linear_speed)
        cmd.angular.z = max(min(wz, self.max_angular_speed), -self.max_angular_speed)
        self.cmd_vel_pub.publish(cmd)

    def stop_robot(self):
        cmd = Twist()
        self.cmd_vel_pub.publish(cmd)

    def _reset_estimator(self, bx_world, by_world):
        self.prev_bx_world = bx_world
        self.prev_by_world = by_world
        self.prev_time = self.get_clock().now().nanoseconds / 1e9
        self.est_bvx = 0.0
        self.est_bvy = 0.0

def main(args=None):
    rclpy.init(args=args)
    node = BallInterceptionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
