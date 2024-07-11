__author__ = "Junnosuke Kamohara"
__copyright__ = (
    "Copyright 2024, Space Robotics Lab, Tohoku University."
)
__maintainer__ = "Junnosuke Kamohara"
__email__ = "kamohara.junnosuke.t6@dc.tohoku.ac.jp"
__status__ = "development"

#! /usr/bin/env python3
import rclpy
from threading import Thread

from .src.data_type import StampedPose
from .src.navigation_ros_wrapper import Navigator_ToPose
from .parameter_node import ParameterNode
from .goal_pose_publisher import GoalPosePublisher

def main()->None:
    """
    Main function to run the commander."""
    rclpy.init()
    parameter_node = ParameterNode("parameter_node")

    # get ros param
    use_localizer = parameter_node._get_param("use_localizer")
    use_slam = parameter_node._get_param("use_slam")
    
    ## setup navigator ##
    ## WIP: get initial pose from localizer
    print("here")
    init_pose = StampedPose(position=[20.0, 20.0], orientation="NORTH")
    ## WIP: get waypoints from npy file
    waypoints = [
        StampedPose(position=[20.0, 20.0], orientation="NORTH"),
        StampedPose(position=[25.0, 25.0], orientation="NORTH"),
        StampedPose(position=[30.0, 30.0], orientation="NORTH"),
        StampedPose(position=[40.0, 40.0], orientation="NORTH"),
        StampedPose(position=[50.0, 50.0], orientation="NORTH"),
        StampedPose(position=[60.0, 60.0], orientation="NORTH"),
    ]
    nv = Navigator_ToPose(init_pose=init_pose, use_localizer=use_localizer, use_slam=use_slam)
    nv.register_waypoints(waypoints)
    
    # run commander
    nv.run_mission()

    # Destroy node when finished
    parameter_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()