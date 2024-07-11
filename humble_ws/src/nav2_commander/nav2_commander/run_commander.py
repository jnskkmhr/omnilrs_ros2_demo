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

from src.data_type import StampedPose
from src.navigation_ros_wrapper import Navigator_ToPose
from parameter_node import ParameterNode
from goal_pose_publisher import GoalPosePublisher
from src.util import yaw2dir

import numpy as np
import argparse

def str2bool(v):
    if isinstance(v, bool):
        return v
    if v.lower() in ('yes', 'true', 'True', 't', 'y', '1'):
        return True
    elif v.lower() in ('no', 'false', 'False', 'f', 'n', '0'):
        return False
    else:
        raise argparse.ArgumentTypeError('Boolean value expected.')

def main(moon_coordinates, path_to_execute)->None:
    """
    Main function to run the navigation

    Args:
    moon_coordinates (str): Coordinates that represent a certain moon image. Format: "coord1_coord2"
    path_to_execute (str): Path to execute. Options: "interpolated_dikstra_path.npy", "interpolated_astar_path.npy"

    Returns:
    None
    """

    # Initialize the node
    rclpy.init()
    parameter_node = ParameterNode("parameter_node")

    # get ros param
    use_localizer = parameter_node._get_param("use_localizer")
    use_slam = parameter_node._get_param("use_slam")
    
    ## setup navigator ##
    original_path = np.load("./{}/{}".format(moon_coordinates, path_to_execute), allow_pickle=True)
    
    init_pose = StampedPose(position=[original_path[0][0][0], original_path[0][0][1]], orientation=yaw2dir(original_path[0][1]))
    waypoints = [StampedPose(position=[waypoint[0][0], waypoint[0][1]], orientation=yaw2dir(waypoint[1]) ) for waypoint in original_path]
    
    nv = Navigator_ToPose(init_pose=init_pose, use_localizer=use_localizer, use_slam=use_slam)
    nv.register_waypoints(waypoints)
    
    # run commander
    nv.run_mission()

    # Destroy node when finished
    parameter_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Execute the navigation')
    parser.add_argument('--moon_coordinates', type=str, default=None, help='Coordinates that represent a certain moon image. Format: "coord1_coord2"')
    parser.add_argument('--execute_dijkstra', type=str2bool, default=None, help='Flag to indicate whether to execute dijsktra. If chosen, the astar path will not be executed. If both true, dijsktra will be executed.')
    parser.add_argument('--execute_astar', type=str2bool, default=None, help='Flag to indicate whether to execute astar. If chosen, the dijsktra path will not be executed. If both true, dijsktra will be executed.')

    args = parser.parse_args()

    moon_coordinates = args.moon_coordinates
    execute_dijsktra = args.execute_dijkstra
    execute_astar = args.execute_astar

    if moon_coordinates is None:
        raise ValueError("Please provide the moon coordinates.")
    
    if execute_dijsktra is None and execute_astar is None:
        raise ValueError("Please provide at least one of the flags to execute dijsktra or astar.")
    
    if execute_dijsktra == False and execute_astar == False:
        raise ValueError("Please provide at least one of the flags to execute dijsktra or astar.")
    
    if execute_dijsktra:
        path_to_execute = "interpolated_dijkstra_path.npy"
    elif execute_astar:
        path_to_execute = "interpolated_astar_path.npy"

    main(moon_coordinates, path_to_execute)