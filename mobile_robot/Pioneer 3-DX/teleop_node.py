import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from pynput import keyboard

class KeyboardTeleop(Node):
    def __init__(self):
        super().__init__('keyboard_teleop')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.linear_speed = 0.3                           # unit is m/s
        self.angular_speed = 1.0                          # unit is rad/s
        self.twist = Twist()

        
        self.listener = keyboard.Listener(
            on_press=self.on_press,
        )
        self.listener.start()
        self.get_logger().info("Keyboard teleop started. Use W,A,S,D to move, SPACE to stop.")

    def on_press(self, key):
        try:
            if key.char.lower() == 'w':
                self.twist.linear.x = self.linear_speed
                self.twist.angular.z = 0.0
            elif key.char.lower() == 's':
                self.twist.linear.x = -self.linear_speed
                self.twist.angular.z = 0.0
            elif key.char.lower() == 'a':
                self.twist.linear.x = 0.0
                self.twist.angular.z = self.angular_speed
            elif key.char.lower() == 'd':
                self.twist.linear.x = 0.0
                self.twist.angular.z = -self.angular_speed
        except AttributeError:
            
            if key == keyboard.Key.space:
                self.twist.linear.x = 0.0
                self.twist.angular.z = 0.0

        
        self.publisher_.publish(self.twist)


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()