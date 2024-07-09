__author__ = "Junnosuke Kamohara"
__copyright__ = (
    "Copyright 2024, Space Robotics Lab, Tohoku University."
)
__maintainer__ = "Junnosuke Kamohara"
__email__ = "kamohara.junnosuke.t6@dc.tohoku.ac.jp"
__status__ = "development"

#! /usr/bin/env python3

import time
from typing import List, Union

## TODO: replace turtlebot4 navigation with generic nav2_simple_commander
from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Directions, TurtleBot4Navigator
from nav2_simple_commander.robot_navigator import TaskResult

from .data_type import StampedPose, StampedPoses, WaypointMissionStatus
from .util import reverse_pose_direction

TB4_DIRECTION = {
    "NORTH": TurtleBot4Directions.NORTH, 
    "NORTH_WEST": TurtleBot4Directions.NORTH_WEST, 
    "WEST": TurtleBot4Directions.WEST, 
    "SOUTH_WEST": TurtleBot4Directions.SOUTH_WEST, 
    "SOUTH": TurtleBot4Directions.SOUTH, 
    "SOUTH_EAST": TurtleBot4Directions.SOUTH_EAST, 
    "EAST": TurtleBot4Directions.EAST, 
    "NORTH_EAST": TurtleBot4Directions.NORTH_EAST,
}

class Navigator_Base:
    def __init__(self, 
                 init_pose: StampedPose, 
                 use_localizer:bool = True, 
                 use_slam:bool = False,
                 ):
        self.navigator = TurtleBot4Navigator()
        self.init_pose = init_pose
        
        self._set_initial_pose(init_pose)
        if use_localizer:
            self.navigator.waitUntilNav2Active(localizer='amcl')
        elif use_slam:
            self.navigator._waitForNodeToActivate('bt_navigator')
            
        # pose buffer
        self.poses = StampedPoses()
    
    def _pose_to_tb4pose(self, pose: StampedPose):
        return self.navigator.getPoseStamped(pose.position, TB4_DIRECTION[pose.orientation])
    
    def _set_initial_pose(self, init_pose: StampedPose):
        self.navigator.setInitialPose(self._pose_to_tb4pose(init_pose))
    
    def _register_pose(self, pose: StampedPose):
        self.poses.add(pose)
    
    def register_waypoints(self, poses: List[StampedPose]):
        for pose in poses:
            self._register_pose(pose)
    
    def _terminate(self):
        self.navigator.cancelTask()
    
    def _sample_target_pose(self):
        raise NotImplementedError
    
    def run_mission(self):
        raise NotImplementedError

class Navigator_ToPose(Navigator_Base):
    """
    usage:
    nv = Navigator_ToPose(init_pose)
    nv._setup(poses)
    nv.run_mission()
    """
    def __init__(self, 
                 init_pose: StampedPose, 
                 use_localizer:bool = True, 
                 use_slam:bool = False,
                 ):
        super().__init__(init_pose, use_localizer, use_slam)
        self.status = WaypointMissionStatus()
        self.status.reset()
        self.exit_flag = False
    
    def _sample_target_pose(self):
        self.status.set_goal_pose(self.poses[self.status.waypoint_id])
    
    def goToPose(self, pose):
        self.navigator.goToPose(self._pose_to_tb4pose(pose))
    
    def run_mission(self):
        # 1sec for warmup
        time.sleep(1.0)
        while not self.exit_flag:
            while self.status.waypoint_id < len(self.poses):
                self._sample_target_pose()
                self.goToPose(self.status.goal_pose)

                while not self.navigator.isTaskComplete():
                    ## do nothing
                    pass
                
                result = self.navigator.getResult()
                if result == TaskResult.SUCCEEDED:
                    self.status.increment() # add 1 to waypoint counter
                    print(f"Completed task at {self.navigator.get_clock().now()}")
                elif result == TaskResult.FAILED:
                    print("Failed task")
            print("Mission complete")
            self.exit_flag = True