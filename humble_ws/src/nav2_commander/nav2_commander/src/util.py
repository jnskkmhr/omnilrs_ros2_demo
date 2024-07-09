__author__ = "Junnosuke Kamohara"
__copyright__ = (
    "Copyright 2024, Space Robotics Lab, Tohoku University."
)
__maintainer__ = "Junnosuke Kamohara"
__email__ = "kamohara.junnosuke.t6@dc.tohoku.ac.jp"
__status__ = "development"

import numpy as np
from .data_type import StampedPose

direction_reverse = {
    "NORTH": "SOUTH",
    "NORTH_WEST": "SOUTH_EAST",
    "WEST": "EAST",
    "SOUTH_WEST": "NORTH_EAST",
    "SOUTH": "NORTH",
    "SOUTH_EAST": "NORTH_WEST",
    "EAST": "WEST",
    "NORTH_EAST": "SOUTH_WEST"
}

def reverse_pose_direction(pose:StampedPose): 
    pose.orientation = direction_reverse[pose.orientation]
    return pose


def yaw2dir(yaw:float):
    """
    TB4 logistics: NWSE is aligned with x up, y left
    NORTH = 0
    NORTH_WEST = 45
    WEST = 90
    SOUTH_WEST = 135
    SOUTH = 180
    SOUTH_EAST = 225
    EAST = 270
    NORTH_EAST = 315
    """
    if 337.5 <= yaw < 360 or 0 <= yaw < 22.5:
        return 'NORTH'
    elif 22.5 <= yaw < 67.5:
        return 'NORTH_WEST'
    elif 67.5 <= yaw < 112.5:
        return 'WEST'
    elif 112.5 <= yaw < 157.5:
        return 'SOUTH_WEST'
    elif 157.5 <= yaw < 202.5:
        return 'SOUTH'
    elif 202.5 <= yaw < 247.5:
        return 'SOUTH_EAST'
    elif 247.5 <= yaw < 292.5:
        return 'EAST'
    elif 292.5 <= yaw < 337.5:
        return 'NORTH_EAST'

def points2yaw(p1, p2):
    """
    p1 is root point in map frame
    p2 is top point in map frame
    arctan2 gives angle in (-180, 180).
    We want angle in (0, 360).
    """
    yaw = np.rad2deg(np.arctan2((p2[1] - p1[1]), p2[0] - p1[0]))
    if yaw < 0:
        yaw = 180 - yaw
    return yaw