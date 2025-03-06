import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial

class MotorControlNode(Node):
    def __init__(self):
        super().__init__('motor_control_node')

        # Serial connection to Arduino (modify port if needed)
        self.serial_port = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)

        # Subscribe to /cmd_vel topic
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

    def cmd_vel_callback(self, msg):
        linear_x = msg.linear.x  # Forward/backward speed
        angular_z = msg.angular.z  # Turning speed

        # Convert velocity to motor speeds (differential drive logic)
        max_speed = 255  # Assuming PWM range 0-255
        left_pwm = int(max(-max_speed, min(max_speed, (linear_x - angular_z) * max_speed)))
        right_pwm = int(max(-max_speed, min(max_speed, (linear_x + angular_z) * max_speed)))

        # Send speeds to Arduino as formatted string
        command = f"{left_pwm},{right_pwm}\n"
        self.serial_port.write(command.encode('utf-8'))

        # Log the command for debugging
        self.get_logger().info(f"Sent to Arduino: {command.strip()}")

def main(args=None):
    rclpy.init(args=args)
    node = MotorControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

