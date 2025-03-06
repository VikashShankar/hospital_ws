#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial

class MotorControlNode(Node):
    def __init__(self):
        super().__init__('motor_control')

        # Serial port configuration
        self.serial_port = '/dev/ttyUSB0'  # Change this to your Arduino's serial port
        self.baud_rate = 115200

        # Initialize serial connection
        try:
            self.ser = serial.Serial(self.serial_port, self.baud_rate, timeout=1)
            self.get_logger().info(f"Connected to Arduino on {self.serial_port}")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to connect to Arduino: {e}")
            exit(1)

        # Subscribe to the /cmd_vel topic
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.twist_callback,
            10
        )
        self.subscription  # Prevent unused variable warning

    def twist_callback(self, msg):
        """
        Callback function for the /cmd_vel topic.
        Converts Twist messages to motor commands and sends them to the Arduino.
        """
        linear_x = msg.linear.x
        angular_z = msg.angular.z

        if linear_x > 0:  # Move forward
            self.ser.write(b'F')
        elif linear_x < 0:  # Move backward
            self.ser.write(b'B')
        elif angular_z > 0:  # Turn left
            self.ser.write(b'L')
        elif angular_z < 0:  # Turn right
            self.ser.write(b'R')
        else:  # Stop
            self.ser.write(b'S')

        # Map linear and angular velocities to PWM values (0-9)
        pwm_value = int((abs(linear_x) + abs(angular_z)) * 9)
        if pwm_value > 9:
            pwm_value = 9
        self.ser.write(str(pwm_value).encode())

def main(args=None):
    rclpy.init(args=args)

    motor_control_node = MotorControlNode()

    try:
        rclpy.spin(motor_control_node)
    except KeyboardInterrupt:
        pass
    finally:
        # Close serial connection on shutdown
        motor_control_node.ser.close()
        motor_control_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
