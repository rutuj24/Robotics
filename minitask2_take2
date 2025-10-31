import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import random
from enum import Enum
import math

class State(Enum):
    RANDOM_WALK = 0
    FOLLOW_WALL = 1
    AVOID_OBSTACLE = 2

class Minitask2(Node):

    def __init__(self):
        super().__init__('minitask2')
        # --- LIDAR directional distances ---
        self.front = float('inf')
        self.left = float('inf')
        self.right = float('inf')
        self.front_left = float('inf')
        self.front_right = float('inf')

        # --- Robot state ---
        self.state = State.RANDOM_WALK
        self.avoid_direction = None
        self.avoid_counter = 0  # To prevent rapid switching
        self.wall_following_side = 'left'  # or 'right', configurable

        # --- Random walk state ---
        self.rw_forward_time = 0
        self.rw_turn_time = 0
        self.rw_moving_forward = True

        # --- Parameters ---
        self.obstacle_threshold = 0.5   # meters
        self.wall_threshold = 0.8       # meters (to follow)
        self.wall_distance = 0.4        # meters (desired distance to wall)
        self.kp = 1.5                   # for P-controller in wall following
        self.max_ang = 1.2              # max angular speed
        self.linear_speed = 0.15        # m/s
        self.angular_speed = 0.5        # rad/s

        # --- ROS Pub/Sub ---
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.sub = self.create_subscription(LaserScan, '/scan', self.lidar_callback, qos)

        self.create_timer(0.1, self.control_loop)  # 10Hz

    def lidar_callback(self, msg):
        n = len(msg.ranges)
        def safe_min(values):
            vals = [r for r in values if not math.isinf(r) and not math.isnan(r)]
            return min(vals) if vals else float('inf')
        # We average over small windows to smoothen out noise
        self.front = safe_min(msg.ranges[0:10] + msg.ranges[-10:])
        self.left = safe_min(msg.ranges[n//4+10:n//4+20])
        self.right = safe_min(msg.ranges[n*3//4-10:n*3//4])
        self.front_left = safe_min(msg.ranges[20:40])
        self.front_right = safe_min(msg.ranges[-40:-20])

    def control_loop(self):
        twist = Twist()

        # ============== State Machine Logic ==============
        if self.front < self.obstacle_threshold or self.front_left < self.obstacle_threshold or self.front_right < self.obstacle_threshold:
            # OBSTACLE AVOIDANCE (priority)
            if self.state != State.AVOID_OBSTACLE:
                self.state = State.AVOID_OBSTACLE
                # Pick turn direction: away from closest wall
                if self.left > self.right:
                    self.avoid_direction = 'left'
                else:
                    self.avoid_direction = 'right'
                self.avoid_counter = 0
                self.get_logger().info(f"STATE: OBSTACLE AVOID -- turning {self.avoid_direction.upper()}")
            self.avoid_counter += 1

            twist.linear.x = 0.0
            twist.angular.z = self.angular_speed if self.avoid_direction == 'left' else -self.angular_speed

            # Only return to other states after obstacle is cleared for a bit
            if self.front > self.obstacle_threshold*1.35 and self.front_left > self.obstacle_threshold*1.35 and self.front_right > self.obstacle_threshold*1.35 and self.avoid_counter > 8:
                self.state = State.RANDOM_WALK
                self.avoid_direction = None
                self.avoid_counter = 0
        
        elif ((self.wall_following_side == 'left' and self.left < self.wall_threshold) or
              (self.wall_following_side == 'right' and self.right < self.wall_threshold)):
            # WALL FOLLOWING
            if self.state != State.FOLLOW_WALL:
                self.state = State.FOLLOW_WALL
                self.get_logger().info("STATE: FOLLOW_WALL")
            twist.linear.x = self.linear_speed
            if self.wall_following_side == 'left':
                error = self.left - self.wall_distance
                twist.angular.z = self.kp * error
            else:
                error = self.right - self.wall_distance
                twist.angular.z = -self.kp * error
            twist.angular.z = max(min(twist.angular.z, self.max_ang), -self.max_ang)
        else:
            # RANDOM WALK
            if self.state != State.RANDOM_WALK:
                self.rw_forward_time, self.rw_turn_time = 0, 0
                self.rw_moving_forward = True
                self.state = State.RANDOM_WALK
                self.get_logger().info("STATE: RANDOM_WALK")
            if self.rw_moving_forward:
                twist.linear.x = self.linear_speed
                twist.angular.z = 0.0
                self.rw_forward_time += 0.1
                if self.rw_forward_time > random.uniform(2.0, 4.5):
                    self.rw_moving_forward = False
                    self.rw_forward_time = 0
                    self.rw_turn_amt = random.choice([-1, 1]) * random.uniform(math.pi/6, math.pi/2)  # random left/right, 30-90 deg
                    self.rw_turn_time = 0
            else:
                twist.linear.x = 0.0
                twist.angular.z = self.angular_speed if self.rw_turn_amt > 0 else -self.angular_speed
                self.rw_turn_time += 0.1
                if self.rw_turn_time > abs(self.rw_turn_amt/self.angular_speed):
                    self.rw_moving_forward = True
                    self.rw_turn_time = 0

        self.pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = Minitask2()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
