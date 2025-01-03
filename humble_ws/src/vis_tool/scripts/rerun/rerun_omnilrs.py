#!/usr/bin/env python3
"""
Simple example of a ROS node that republishes some common types to Rerun.

The solution here is mostly a toy example to show how ROS concepts can be
mapped to Rerun. Fore more information on future improved ROS support,
see the tracking issue: <https://github.com/rerun-io/rerun/issues/1537>.

NOTE: Unlike many of the other examples, this example requires a system installation of ROS
in addition to the packages from requirements.txt.
"""

from __future__ import annotations

import argparse
import sys

import numpy as np
import matplotlib
import rerun as rr  # pip install rerun-sdk

try:
    import cv_bridge
    import laser_geometry
    import rclpy
    import rerun_urdf
    import trimesh
    from image_geometry import PinholeCameraModel
    from nav_msgs.msg import Odometry
    from numpy.lib.recfunctions import structured_to_unstructured
    from rclpy.callback_groups import ReentrantCallbackGroup
    from rclpy.node import Node
    from rclpy.qos import QoSDurabilityPolicy, QoSProfile
    from rclpy.time import Duration, Time
    from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField, Imu, LaserScan, JointState
    from sensor_msgs_py import point_cloud2
    from std_msgs.msg import String
    from tf2_ros import TransformException
    from tf2_ros.buffer import Buffer
    from tf2_ros.transform_listener import TransformListener

except ImportError:
    print(
        """
Could not import the required ROS2 packages.

Make sure you have installed ROS2 (https://docs.ros.org/en/humble/index.html)
and sourced /opt/ros/humble/setup.bash

See: README.md for more details.
"""
    )
    sys.exit(1)


class OmniLRSSubscriber(Node):  # type: ignore[misc]
    def __init__(self) -> None:
        super().__init__("husky_rover_rerun")

        # Used for subscribing to latching topics
        latching_qos = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)

        # Allow concurrent callbacks
        self.callback_group = ReentrantCallbackGroup()

        # Subscribe to TF topics
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Define a mapping for transforms
        self.path_to_frame = {
            "world": "odom",
            "world/points": "vlp6",
            "world/robot": "base_link",
            "world/robot/lidar": "vlp6",
            "world/robot/camera": "Camera_Pseudo_Depth",
            "world/robot/lidar/points": "vlp6",
        }

        # Assorted helpers for data conversions
        self.model = PinholeCameraModel()
        self.cv_bridge = cv_bridge.CvBridge()
        self.laser_proj = laser_geometry.laser_geometry.LaserProjection()
        self.cmap = matplotlib.colormaps["turbo_r"]
        self.norm = matplotlib.colors.Normalize(vmin=3.0, vmax=75.0)

        # Log a bounding box as a visual placeholder for the map
        # TODO(jleibs): Log the real map once [#1531](https://github.com/rerun-io/rerun/issues/1531) is merged
        # rr.log(
        #     "world/box",
        #     rr.Boxes3D(half_sizes=[3, 3, 1], centers=[0, 0, 1], colors=[255, 255, 255, 255]),
        #     static=True,
        # )

        # Subscriptions
        self.info_sub = self.create_subscription(
            CameraInfo,
            "/front_camera/depth/depth_info",
            self.cam_info_callback,
            10,
            callback_group=self.callback_group,
        )

        self.img_sub = self.create_subscription(
            Image,
            "/front_camera/mono/rgb",
            self.rgb_image_callback,
            10,
            callback_group=self.callback_group,
        )

        self.depth_img_sub = self.create_subscription(
            Image,
            "/front_camera/depth/depth",
            self.depth_image_callback,
            10,
            callback_group=self.callback_group,
        )

        self.points_sub = self.create_subscription(
            PointCloud2,
            "/pointcloud",
            self.points_callback,
            10,
            callback_group=self.callback_group,
        )

        self.odom_sub = self.create_subscription(
            Odometry,
            "/odom",
            self.odom_callback,
            10,
            callback_group=self.callback_group,
        )

        self.imu_sub = self.create_subscription(
            Imu,
            "/imu",
            self.imu_callback,
            10,
            callback_group=self.callback_group,
        )

    def log_tf_as_transform3d(self, path: str, time: Time) -> None:
        """
        Helper to look up a transform with tf and log using `log_transform3d`.

        Note: we do the lookup on the client side instead of re-logging the raw transforms until
        Rerun has support for Derived Transforms [#1533](https://github.com/rerun-io/rerun/issues/1533)
        """
        # Get the parent path
        parent_path = path.rsplit("/", 1)[0]

        # Find the corresponding frames from the mapping
        child_frame = self.path_to_frame[path]
        parent_frame = self.path_to_frame[parent_path]

        # Do the TF lookup to get transform from child (source) -> parent (target)
        try:
            tf = self.tf_buffer.lookup_transform(parent_frame, child_frame, time, timeout=Duration(nanoseconds=100000000))
            t = tf.transform.translation
            q = tf.transform.rotation
            rr.log(path, rr.Transform3D(translation=[t.x, t.y, t.z], rotation=rr.Quaternion(xyzw=[q.x, q.y, q.z, q.w])))
        except TransformException as ex:
            print(f"Failed to get transform: {ex}")

    def cam_info_callback(self, info: CameraInfo) -> None:
        """Log a `CameraInfo` with `log_pinhole`."""
        time = Time.from_msg(info.header.stamp)
        rr.set_time_nanos("ros_time", time.nanoseconds)

        self.model.fromCameraInfo(info)

        rr.log(
            "world/robot/camera/rgb_img",
            rr.Pinhole(
                resolution=[self.model.width * 2 , self.model.height * 2],
                image_from_camera=self.model.intrinsicMatrix(),
            ),
        )

    def rgb_image_callback(self, img: Image) -> None:
        """Log an `Image` with `log_image` using `cv_bridge`."""
        time = Time.from_msg(img.header.stamp)
        rr.set_time_nanos("ros_time", time.nanoseconds)

        rr.log("world/robot/camera/rgb_img", rr.Image(self.cv_bridge.imgmsg_to_cv2(img)))
        self.log_tf_as_transform3d("world/robot/camera", time)

    def depth_image_callback(self, img: Image) -> None:
        """Log an `Image` with `log_image` using `cv_bridge`."""
        time = Time.from_msg(img.header.stamp)
        rr.set_time_nanos("ros_time", time.nanoseconds)

        # Convert ROS Image message to NumPy array
        depth_array = self.cv_bridge.imgmsg_to_cv2(img, desired_encoding="passthrough")

        # Define the region of interest (ROI)
        min_depth = 0.5  # Minimum depth in meters
        max_depth = 5.0  # Maximum depth in meters

        # Clip and normalize depth values to the ROI
        depth_array_clipped = np.clip(depth_array, min_depth, max_depth)
        depth_normalized = (depth_array_clipped - min_depth) / (max_depth - min_depth)

        # Apply colormap
        depth_colored = (self.cmap(depth_normalized)[:, :, :3] * 255).astype(np.uint8)

        # Log the colored depth image
        rr.log("world/robot/camera/depth_img", rr.Image(depth_colored))
        self.log_tf_as_transform3d("world/robot/camera", time)


    def points_callback(self, points: PointCloud2) -> None:
        """Log a `PointCloud2` with `log_points`."""
        time = Time.from_msg(points.header.stamp)
        rr.set_time_nanos("ros_time", time.nanoseconds)

        # Read points (x, y, z)
        pts = point_cloud2.read_points(points, field_names=["x", "y", "z"], skip_nans=True)
        pts = structured_to_unstructured(pts)  # Convert structured array to unstructured

        # Calculate distances for coloring
        point_distances = np.linalg.norm(pts, axis=1)

        # Normalize distances to colormap range
        self.norm = matplotlib.colors.Normalize(vmin=np.min(point_distances), vmax=np.max(point_distances))
        point_colors = self.cmap(self.norm(point_distances))  # Turbo colormap returns RGBA values

        # Convert colormap output (float) to 8-bit integer for Rerun
        point_colors = (point_colors[:, :3] * 255).astype(np.uint8)

        # Log points with Turbo colors
        rr.log("world/robot/lidar/points", rr.Points3D(pts, colors=point_colors.tolist()))
        self.log_tf_as_transform3d("world/robot/lidar/points", time)


    def odom_callback(self, odom: Odometry) -> None:
        """Update transforms when odom is updated."""
        time = Time.from_msg(odom.header.stamp)
        rr.set_time_nanos("ros_time", time.nanoseconds)

        # Capture time-series data for the linear and angular velocities
        rr.log("odometry/x", rr.Scalar(odom.pose.pose.position.x))
        rr.log("odometry/y", rr.Scalar(odom.pose.pose.position.y))
        rr.log("odometry/z", rr.Scalar(odom.pose.pose.position.z))

        # Update the robot pose itself via TF
        self.log_tf_as_transform3d("world/robot", time)


    def imu_callback(self, imu: Imu) -> None:
        """Log IMU data """
        time = Time.from_msg(imu.header.stamp)
        rr.set_time_nanos("ros_time", time.nanoseconds)

        # Capture time-series data for the imu orientation
        rr.log("imu/linear_acceleration/x", rr.Scalar(imu.linear_acceleration.x))
        rr.log("imu/linear_acceleration/y", rr.Scalar(imu.linear_acceleration.y))
        rr.log("imu/linear_acceleration/z", rr.Scalar(imu.linear_acceleration.z))

        rr.log("imu/angular_velocity/x", rr.Scalar(imu.angular_velocity.x))
        rr.log("imu/angular_velocity/y", rr.Scalar(imu.angular_velocity.y))
        rr.log("imu/angular_velocity/z", rr.Scalar(imu.angular_velocity.z))

def main() -> None:
    parser = argparse.ArgumentParser(description="Simple example of a ROS node that republishes to Rerun.")
    rr.script_add_args(parser)
    args, unknownargs = parser.parse_known_args()
    rr.script_setup(args, "rerun_example_ros_node")

    # Any remaining args go to rclpy
    rclpy.init(args=unknownargs)

    omnilrs_subscriber = OmniLRSSubscriber()

    # Use the MultiThreadedExecutor so that calls to `lookup_transform` don't block the other threads
    rclpy.spin(omnilrs_subscriber, executor=rclpy.executors.MultiThreadedExecutor())

    omnilrs_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()