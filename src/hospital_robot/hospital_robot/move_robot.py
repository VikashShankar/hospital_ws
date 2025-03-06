import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial

class TeleopLEDController(Node):
    def __init__(self):
        super().__init__('teleop_led_controller')

        # Initialize Serial Communication
        try:
            self.serial_port = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
            self.get_logger().info("Connected to Arduino on /dev/ttyUSB0")
        except serial.SerialException:
            self.get_logger().error("Failed to connect to /dev/ttyUSB0")
            return

        # Subscribe to teleop Twist messages
        self.subscription = self.create_subscription(
            Twist, '/cmd_vel', self.command_callback, 10)

    def command_callback(self, msg):
        """Convert teleop messages to LED commands."""
        if msg.linear.x > 0:   # Move forward -> Turn LED ON
            command = "ON"
        elif msg.linear.x < 0: # Move backward -> Turn LED OFF
            command = "OFF"
        elif msg.angular.z != 0: # Turn left/right -> Blink LED
            command = "BLINK"
        else:
            return  # Ignore zero movement

        # Send command to Arduino
        self.serial_port.write((command + "\n").encode())
        self.get_logger().info(f"Sent to Arduino: {command}")

def main(args=None):
    rclpy.init(args=args)
    node = TeleopLEDController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

