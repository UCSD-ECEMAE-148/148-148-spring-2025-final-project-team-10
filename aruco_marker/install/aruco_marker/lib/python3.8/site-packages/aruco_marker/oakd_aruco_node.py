#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import cv2
import depthai as dai
import numpy as np
import cv2.aruco as aruco

class ArucoTrackerNode(Node):
    def __init__(self):
        super().__init__('oakd_aruco_node')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        self.marker_size = 0.14  # meters
        self.min_safe_distance = 0.3 # in meters
        self.init_camera()

    def init_camera(self):
        pipeline = dai.Pipeline()
        cam_rgb = pipeline.create(dai.node.ColorCamera)
        cam_rgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
        cam_rgb.setInterleaved(False)
        cam_rgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)

        xout = pipeline.create(dai.node.XLinkOut)
        xout.setStreamName("video")
        cam_rgb.video.link(xout.input)

        self.device = dai.Device(pipeline)
        self.video_queue = self.device.getOutputQueue("video", maxSize=4, blocking=False)

        calib = self.device.readCalibration()
        intrinsics = calib.getCameraIntrinsics(dai.CameraBoardSocket.RGB, 1920, 1080)
        self.camera_matrix = np.array([
            [intrinsics[0][0], 0, intrinsics[0][2]],
            [0, intrinsics[1][1], intrinsics[1][2]],
            [0, 0, 1]
        ], dtype=np.float64)
        all_coeffs = calib.getDistortionCoefficients(dai.CameraBoardSocket.RGB)
        self.dist_coeffs = np.array(all_coeffs[:5], dtype=np.float64).reshape(1, 5)

        self.dict_aruco = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)

        self.get_logger().info("OAK-D camera initialized. Starting detection loop.")
        self.timer = self.create_timer(0.03, self.detect_and_publish)

    def detect_and_publish(self):
        in_frame = self.video_queue.get()
        frame = in_frame.getCvFrame()
        corners, ids, _ = aruco.detectMarkers(frame, self.dict_aruco)

        if ids is not None:
            cv2.imwrite("/home/projects/ros2_ws/src/aruco_marker/latest.jpg", frame)
            rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(
                corners, self.marker_size, self.camera_matrix, self.dist_coeffs
            )
            for i, marker_id in enumerate(ids.flatten()):
                tvec = tvecs[i][0]
                rvec = rvecs[i][0]

                x, y, z = tvec
                yaw_deg = np.degrees(np.arctan2(x, z))
                pitch_deg = np.degrees(np.arctan2(y, z))

                self.get_logger().info(
                    f"Marker {marker_id} — Distance: {z:.2f} m, Yaw: {yaw_deg:.2f}°, Pitch: {pitch_deg:.2f}°"
                )

                # Publish Twist
                msg = Twist()
                msg.angular.z = np.radians(yaw_deg)
                msg.linear.x = z
                if z < self.min_safe_distance:
                    self.get_logger().warn(f"Marker too close ({z:.2f} m). Stopping.")
                    msg.linear.x = 0.0
                    msg.angular.z = 0.0
                self.publisher.publish(msg)
        else:
            msg = Twist()
            msg.angular.z = 0.0
            msg.linear.x = 0.0
            self.publisher.publish(msg)
            self.get_logger().info(f"No Marker Detected")

def main(args=None):
    rclpy.init(args=args)
    node = ArucoTrackerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
