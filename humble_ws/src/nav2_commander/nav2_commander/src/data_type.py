__author__ = "Junnosuke Kamohara"
__copyright__ = (
    "Copyright 2024, Space Robotics Lab, Tohoku University."
)
__maintainer__ = "Junnosuke Kamohara"
__email__ = "kamohara.junnosuke.t6@dc.tohoku.ac.jp"
__status__ = "development"

from dataclasses import dataclass, field
from typing import List, Union

##############################
### Nav2 related dataclass ###
##############################

@dataclass
class StampedPose:
    """
    stamp and frame_id are given by TurtleBot4Navigator.getPoseStamped
    """
    position: List[float] = field(default_factory=[0.0, 0.0])
    orientation: str = "NORTH"
    def __post_init__(self):
        assert len(self.position) == 2, "position should be a 2D list"

@dataclass
class StampedPoses:
    poses: Union[List[StampedPose], list] = field(default_factory=list) 
    def __post_init__(self):
        assert type(self.poses) == list, "poses should be a list of StampedPose"
    
    def add(self, pose: StampedPose):
        self.poses.append(pose)
    
    def __len__(self):
        return len(self.poses)
    def __iter__(self):
        return iter(self.poses)
    def __getitem__(self, index):
        return self.poses[index]

@dataclass
class MissionStatus:
    status: int = 0
    def __post_init__(self):
        assert self.status in [0, 1, 2], "status should be 0(pending), 1(execution) or 2(terminate)"

@dataclass
class WaypointMissionStatus(MissionStatus):
    waypoint_id: int = None
    goal_pose: StampedPose = None
    def __post_init__(self):
        """
        status: 0(pending)
                1(go)
                2(terminate-> move robot to init pose)
        """
        assert self.status in [0, 1, 2], "status should be 0(pending), 1(execution) or 2(terminate)"
    def increment(self):
        self.waypoint_id += 1
    def reset(self):
        self.waypoint_id = 0
    def set_goal_pose(self, pose: StampedPose):
        assert self.waypoint_id >= 0, "waypoint_id should be greater than 0"
        self.goal_pose = pose
    
    
if __name__ == "__main__":
    stamped_pose = StampedPose(position=[1.0, 1.0], orientation="NORTH")
    print(stamped_pose)

    stamped_poses = StampedPoses()
    stamped_poses.add(stamped_pose)
    stamped_poses.add(stamped_pose)
    stamped_poses.add(stamped_pose)