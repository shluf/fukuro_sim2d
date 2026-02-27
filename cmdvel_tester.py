import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class CmdVelTester(Node):
    def __init__(self):
        super().__init__('cmdvel_tester')
        self.pub = self.create_publisher(Twist, '/r2/cmd_vel', 10)
        # Ganti test_case = 1, 2, 3, atau 4 lalu jalankan masing-masing
        self.test_case = 4
        self.timer = self.create_timer(0.1, self.run)
        self.start_time = self.get_clock().now().seconds_nanoseconds()[0]

    def run(self):
        now = self.get_clock().now().seconds_nanoseconds()[0]
        elapsed = now - self.start_time
        
        # Kirim selama 2 detik lalu stop
        if elapsed > 2:
            self.pub.publish(Twist())
            self.get_logger().info("STOP - catat arah gerak robot!")
            return

        cmd = Twist()

        if self.test_case == 1:
            cmd.linear.x = 1.0   # positif X
            self.get_logger().info("TEST 1: linear.x = +1.0")
        elif self.test_case == 2:
            cmd.linear.x = -1.0  # negatif X
            self.get_logger().info("TEST 2: linear.x = -1.0")
        elif self.test_case == 3:
            cmd.linear.y = 1.0   # positif Y
            self.get_logger().info("TEST 3: linear.y = +1.0")
        elif self.test_case == 4:
            cmd.linear.y = -1.0  # negatif Y
            self.get_logger().info("TEST 4: linear.y = -1.0")

        self.pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = CmdVelTester()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
