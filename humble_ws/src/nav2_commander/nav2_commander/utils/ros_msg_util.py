# from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from builtin_interfaces.msg import Time
import rclpy

from ..src.data_type import StampedPose, StampedPoses


def pose_to_ros_pose(pose: PoseStamped):
    ros_pose = PoseStamped()
    ros_pose.header.frame_id = pose.frame_id
    ros_pose.header.stamp = Time(sec=int(pose.stamp), nanosec=int((pose.stamp - int(pose.stamp)) % 1e9))
    ros_pose.pose.position.x = pose.position[0]
    ros_pose.pose.position.y = pose.position[1]
    ros_pose.pose.position.z = pose.position[2]
    ros_pose.pose.orientation.x = pose.orientation[0]
    ros_pose.pose.orientation.y = pose.orientation[1]
    ros_pose.pose.orientation.z = pose.orientation[2]
    ros_pose.pose.orientation.w = pose.orientation[3]
    return ros_pose

def ros_register_stamp(src_msg, stamp_msg):
    src_msg.header.stamp = stamp_msg
    return src_msg

def poses_to_ros_poses(poses: StampedPoses):
    ros_poses = []
    for pose in poses.poses:
        ros_poses.append(pose_to_ros_pose(pose))
    return ros_poses


if __name__ == "__main__":
    init_pose = StampedPose(position=[1.0, 1.0, 0.0], orientation=[0.0, 0.0, 0.0, 1.0])
    pose_msg = pose_to_ros_pose(init_pose)
    print(pose_msg)

    stamped_poses = StampedPoses()
    stamped_poses.add(init_pose)
    stamped_poses.add(init_pose)
    stamped_poses.add(init_pose)
    ros_poses = poses_to_ros_poses(stamped_poses)
    print(ros_poses)