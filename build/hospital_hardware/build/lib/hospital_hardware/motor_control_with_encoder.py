import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
import serial
import time
import math

class MotorControlEncoderNode(Node):
    def __init__(self):
        super().__init__('motor_control_with_encoder')

        # Parameters
        self.declare_parameters(namespace='', parameters=[
            ('serial_port', '/dev/ttyUSB1'),
            ('baud_rate', 57600),
            ('wheel_radius', 0.05),
            ('wheel_base', 0.45),
            ('encoder_counts_per_rev', 1000),
            ('encoder_counts_per_rev_2', 1000)
        ])

        self.serial_port = self.get_parameter('serial_port').value
        self.baud_rate = self.get_parameter('baud_rate').value
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.wheel_base = self.get_parameter('wheel_base').value
        self.encoder_counts_per_rev = self.get_parameter('encoder_counts_per_rev').value
        self.encoder_counts_per_rev_2 = self.get_parameter('encoder_counts_per_rev_2').value

        # Serial connection to Arduino
        try:
            self.ser = serial.Serial(self.serial_port, self.baud_rate, timeout=1)
            time.sleep(2)
            self.ser.reset_input_buffer()
            self.get_logger().info(f"Connected to Arduino on {self.serial_port}")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to connect to Arduino: {e}")
            exit(1)

        # Subscribers and Publishers
        self.subscription = self.create_subscription(Twist, '/cmd_vel', self.twist_callback, 10)
        self.encoder_pub = self.create_publisher(Float32MultiArray, '/encoder_rpm', 10)
        self.joint_state_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # Joint State
        self.joint_state = JointState()
        self.joint_state.name = ['left_wheel_joint', 'right_wheel_joint']
        self.joint_state.position = [0.0, 0.0]
        self.joint_state.velocity = [0.0, 0.0]
        self.joint_state.effort = [0.0, 0.0]

        # Odometry
        self.odom = Odometry()
        self.odom.header.frame_id = 'odom'
        self.odom.child_frame_id = 'base_link'
        self.odom.pose.pose.orientation.w = 1.0

        # Encoder Data
        self.last_enc1 = 0
        self.last_enc2 = 0
        self.last_time = self.get_clock().now()

        # Robot State
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # Timer
        self.timer = self.create_timer(0.1, self.read_encoder_data)  # 10Hz

    def twist_callback(self, msg):
        linear_x = msg.linear.x
        angular_z = msg.angular.z

        # Send motor commands
        if linear_x > 0:
            self.ser.write(b'F')  # Forward
        elif linear_x < 0:
            self.ser.write(b'B')  # Backward
        elif angular_z > 0:
            self.ser.write(b'L')  # Left
        elif angular_z < 0:
            self.ser.write(b'R')  # Right
        else:
            self.ser.write(b'S')  # Stop

        # Send PWM value (0-9)
        pwm_value = min(int((abs(linear_x) + abs(angular_z)) * 9), 9)
        self.ser.write(str(pwm_value).encode())

    def read_encoder_data(self):
        if self.ser.in_waiting > 0:
            try:
                serial_data = self.ser.readline().decode('utf-8').strip()
                if "ENC1:" in serial_data and "ENC2:" in serial_data:
                    enc1 = int(serial_data.split("ENC1:")[1].split(",")[0])
                    enc2 = int(serial_data.split("ENC2:")[1])

                    current_time = self.get_clock().now()
                    dt = (current_time - self.last_time).nanoseconds / 1e9

                    if dt == 0:
                        return

                    # Calculate RPM
                    rpm1 = ((enc1 - self.last_enc1) / self.encoder_counts_per_rev) * (60 / dt)
                    rpm2 = ((enc2 - self.last_enc2) / self.encoder_counts_per_rev_2) * (60 / dt)

                    # Log RPM values
                    self.get_logger().info(f"RPM Left: {rpm1:.2f}, RPM Right: {rpm2:.2f}")

                    # Update joint states (radians)
                    delta_theta_left = (enc1 - self.last_enc1) * (2 * math.pi / self.encoder_counts_per_rev)
                    delta_theta_right = (enc2 - self.last_enc2) * (2 * math.pi / self.encoder_counts_per_rev_2)
                    self.joint_state.position[0] += delta_theta_left
                    self.joint_state.position[1] += delta_theta_right

                    # Update joint velocities (rad/s)
                    self.joint_state.velocity[0] = rpm1 * (2 * math.pi / 60)
                    self.joint_state.velocity[1] = rpm2 * (2 * math.pi / 60)

                    # Publish joint states
                    self.joint_state.header.stamp = self.get_clock().now().to_msg()
                    self.joint_state_pub.publish(self.joint_state)

                    # Update odometry
                    self.update_odometry(enc1 - self.last_enc1, enc2 - self.last_enc2, dt)

                    # Publish RPM
                    rpm_msg = Float32MultiArray()
                    rpm_msg.data = [rpm1, rpm2]
                    self.encoder_pub.publish(rpm_msg)

                    self.last_enc1 = enc1
                    self.last_enc2 = enc2
                    self.last_time = current_time

            except (UnicodeDecodeError, ValueError) as e:
                self.get_logger().warn(f"Encoder data error: {e}")

    def update_odometry(self, delta_enc1, delta_enc2, dt):
        # Calculate wheel distances
        distance_left = (delta_enc1 / self.encoder_counts_per_rev) * (2 * math.pi * self.wheel_radius)
        distance_right = (delta_enc2 / self.encoder_counts_per_rev_2) * (2 * math.pi * self.wheel_radius)

        # Calculate linear and angular velocity
        linear_velocity = (distance_right + distance_left) / 2.0
        angular_velocity = (distance_right - distance_left) / self.wheel_base

        # Update pose
        self.x += linear_velocity * math.cos(self.theta)
        self.y += linear_velocity * math.sin(self.theta)
        self.theta += angular_velocity

        # Normalize theta
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))

        # Update odometry message
        self.odom.header.stamp = self.get_clock().now().to_msg()
        self.odom.pose.pose.position.x = self.x
        self.odom.pose.pose.position.y = self.y
        self.odom.pose.pose.orientation.z = math.sin(self.theta / 2.0)
        self.odom.pose.pose.orientation.w = math.cos(self.theta / 2.0)
        self.odom.twist.twist.linear.x = linear_velocity / dt
        self.odom.twist.twist.angular.z = angular_velocity / dt
        self.odom_pub.publish(self.odom)

        # Broadcast TF
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = 'odom'
        transform.child_frame_id = 'base_link'
        transform.transform.translation.x = self.x
        transform.transform.translation.y = self.y
        transform.transform.rotation.z = math.sin(self.theta / 2.0)
        transform.transform.rotation.w = math.cos(self.theta / 2.0)
        self.tf_broadcaster.sendTransform(transform)

def main(args=None):
    rclpy.init(args=args)
    node = MotorControlEncoderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.ser.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
