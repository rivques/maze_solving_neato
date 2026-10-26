"""
runner_logic
This file provides the bulk of the logic for the MazeRunnerNode.
The logic is broken out into this file as pure functions for ease
of testing.
"""

from geometry_msgs.msg import Pose
from nav_msgs.msg import OccupancyGrid

def plan_trajectory(map: OccupancyGrid, initial_pose: Pose, goal_pose: Pose) -> list[Pose]:
    return []