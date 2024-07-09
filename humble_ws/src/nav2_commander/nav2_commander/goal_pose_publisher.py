__author__ = "Junnosuke Kamohara"
__copyright__ = (
    "Copyright 2024, Space Robotics Lab, Tohoku University."
)
__maintainer__ = "Junnosuke Kamohara"
__email__ = "kamohara.junnosuke.t6@dc.tohoku.ac.jp"
__status__ = "development"

from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from geometry_msgs.msg import PoseStamped
import numpy as np

class GoalPosePublisher(Node):
    """
    ROS parameter manager node."""
    def __init__(self, node_name:str)->None:
        super().__init__(node_name)
        self.publisher = self.create_publisher(PoseStamped, 'goal_pose', 10)
        self.goal_pose = None
        self.timer = self.create_timer(0.1, self.timer_callback)
    
    def register_goal_pose(self, pose:np.ndarray)->None:
        """
        Register goal pose.
        Args:
            pose (np.ndarray): goal pose in 2D numpy array. (x, y, yaw)
        """
        self.goal_pose = pose
    
    def publish_goal_pose(self)->None:
        """
        Publish goal pose."""
        if self.goal_pose is not None:
            msg = PoseStamped()
            msg.header.frame_id = "map"
            msg.pose.position.x = self.goal_pose[0]
            msg.pose.position.y = self.goal_pose[1]
            msg.pose.position.z = 0.0
            msg.pose.orientation.x = 0.0
            msg.pose.orientation.y = 0.0
            msg.pose.orientation.z = np.sin(self.goal_pose[2]/2)
            msg.pose.orientation.w = np.cos(self.goal_pose[2]/2)
            self.publisher.publish(msg)
    
    def timer_callback(self)->None:
        self.publish_goal_pose()
    
    def thread_function(self)->None:
        executor = SingleThreadedExecutor()
        executor.add_node(self)
        executor.spin()