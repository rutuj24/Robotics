# Import the necessary ROS 2 Python libraries and message type
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        
        # Create a publisher that sends Twist messages to the 'cmd_vel' topic
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # --- MODIFIED: Define the duration for each action ---
        self.move_duration = 5.0  # seconds
        self.turn_duration = 5.0  # seconds
        self.stop_duration = 0.25 # seconds
        
        self.i = 0
        self.timer = None  # We will create the timer in the callback

        # --- MODIFIED: Start the cycle immediately ---
        self.timer_callback()

    def timer_callback(self):
        # Determine which state we are in (0=forward, 1=stop, 2=turn, 3=stop)
        state = self.i % 4
        
        # Create the message and determine the duration for the NEXT timer
        msg = Twist()
        next_timer_duration = 0.0
        
        if state == 0:
            # State 0: Move forward
            msg.linear.x = 1.0
            self.get_logger().info(f'Publishing: Move Forward for {self.move_duration} seconds')
            next_timer_duration = self.move_duration
        elif state == 1:
            # State 1: Stop
            self.get_logger().info(f'Publishing: Stop for {self.stop_duration} seconds')
            next_timer_duration = self.stop_duration
        elif state == 2:
            # State 2: Turn left (yaw)
            msg.angular.z = 1.0
            self.get_logger().info(f'Publishing: Turn Left for {self.turn_duration} seconds')
            next_timer_duration = self.turn_duration
        else: # state == 3
            # State 3: Stop
            self.get_logger().info(f'Publishing: Stop for {self.stop_duration} seconds')
            next_timer_duration = self.stop_duration
            
        # Publish the message for the current state
        self.publisher_.publish(msg)
        
        # Increment state for the next cycle
        self.i += 1
        
        # --- MODIFIED: Create a new one-shot timer for the next state ---
        self.timer = self.create_timer(next_timer_duration, self.timer_callback)

def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    # We only need to spin, as the timer cycle is self-perpetuating
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
