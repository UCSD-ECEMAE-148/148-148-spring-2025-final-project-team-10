import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import numpy as np
import sys
sys.path.append("/home/projects/ros2_ws/src/aruco_marker/aruco_marker/")
from vesc_submodule.vesc_client import VESC_

NODE_NAME = 'vesc_twist_node'
TOPIC_NAME = '/cmd_vel'

class VescTwist(Node):
    def __init__(self):
        super().__init__(NODE_NAME)
        self.vesc = VESC_()
        self.subscription = self.create_subscription(Twist, TOPIC_NAME, self.cmd_vel_callback, 10)

        # Declare parameters with defaults
        self.declare_parameters('', [
            ('max_rpm', 20000),
            ('steering_polarity', 1),
            ('throttle_polarity', 1),
            ('max_right_steering', 0.65),
            ('straight_steering', 0.5),
            ('max_left_steering', 0.35),
            ('zero_throttle', 0.0),
            ('max_throttle', 0.1),
            ('min_throttle', 0.09),
        ])

        # Load parameters
        self.max_rpm = int(self.get_parameter('max_rpm').value)
        self.steering_polarity = int(self.get_parameter('steering_polarity').value)
        self.throttle_polarity = int(self.get_parameter('throttle_polarity').value)
        self.max_right_steering = self.get_parameter('max_right_steering').value
        self.straight_steering = self.get_parameter('straight_steering').value
        self.max_left_steering = self.get_parameter('max_left_steering').value
        self.zero_throttle = self.get_parameter('zero_throttle').value
        self.max_throttle = self.get_parameter('max_throttle').value
        self.min_throttle = self.get_parameter('min_throttle').value

        # Derived remappings
        self.max_right_angle = self.remap(self.max_right_steering)
        self.straight_offset = self.remap(self.straight_steering) - 0.5
        self.max_left_angle = self.remap(self.max_left_steering)

        self.zero_rpm = int(self.zero_throttle * self.max_rpm)
        self.max_rpm = int(self.max_throttle * self.max_rpm)
        self.min_rpm = int(self.min_throttle * self.max_rpm)

        self.get_logger().info(f'[{NODE_NAME}] initialized and subscribed to {TOPIC_NAME}')

    def cmd_vel_callback(self, msg: Twist):
        # Get yaw (angular.z) and forward speed (linear.x)
        # yaw = self.clamp(msg.angular.z, 1.0)
        yaw = msg.angular.z
        forward = self.clamp(msg.linear.x, 1.0)

        # Map yaw to steering [0,1]
        # steering = self.remap(yaw) + self.straight_offset
        steering = self.remap(np.degrees(yaw))
        # self.get_logger().info(f"yaw: {yaw} - steering: {steering}")
        # self.get_logger().info(f"Steering: {steering}")
        rpm = int(self.max_rpm * forward)

        self.get_logger().info(f'Twist received → RPM: {rpm}, Steering: {steering:.3f}')

        self.vesc.send_rpm(self.throttle_polarity * rpm)
        self.vesc.send_servo_angle(self.steering_polarity * steering)

    @staticmethod
    def remap(value_deg, in_min_deg=-30.0, in_max_deg=30.0, out_min=0.0, out_max=1.0):
        """
        Maps a yaw value in degrees to a steering value between out_min and out_max.
        """
        # print(f"{value_deg}, {in_min_deg}, {in_max_deg}, {out_min}, {out_max}")
        return out_min + ((value_deg - in_min_deg) * (out_max - out_min) / (in_max_deg - in_min_deg))


    @staticmethod
    def clamp(val, max_val, min_val=None):
        if min_val is None:
            min_val = -max_val
        return max(min(val, max_val), min_val)

def main(args=None):
    rclpy.init(args=args)
    try:
        node = VescTwist()
        rclpy.spin(node)
    except Exception as e:
        rclpy.get_logger(NODE_NAME).error(f"[ERROR] Could not start node: {e}")
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
