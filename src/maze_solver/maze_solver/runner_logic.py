"""
runner_logic
This file provides the bulk of the logic for the MazeRunnerNode.
The logic is broken out into this file as pure functions for ease
of testing.
"""

from queue import Queue

from geometry_msgs.msg import Pose
from nav_msgs.msg import OccupancyGrid
from .angle_helpers import pose_to_tuple, tuple_to_pose
import numpy as np

Cell = tuple[int, int] # (x, y)

def plan_trajectory(map: OccupancyGrid, initial_pose: Pose, goal_pose: Pose) -> list[Pose]:
    """
    Plan a trajectory from the initial pose to the goal pose given a map.
    main source: https://www.redblobgames.com/pathfinding/a-star/introduction.html
    """
    # breadth-first search
    # doesn't scale well, but the map is small and computers are fast
    start_cell = pose_to_cell(map, initial_pose)
    goal_cell = pose_to_cell(map, goal_pose)
    assert is_cell_free(map, *start_cell), "The start cell is not free"
    assert is_cell_free(map, *goal_cell), "The goal cell is not free"
    frontier: Queue[Cell] = Queue()
    frontier.put(start_cell)
    came_from: dict[Cell, Cell] = {} # map a cell to the cell we found it from
    found_goal = False

    while not frontier.empty():
        current_cell = frontier.get()
        if current_cell == goal_cell:
            found_goal = True
            break
        for neighbor in get_neighbors(map, current_cell):
            if neighbor not in came_from and is_cell_free(map, *neighbor):
                frontier.put(neighbor)
                came_from[neighbor] = current_cell
    if not found_goal:
        raise ValueError("No path found from start to goal")
    # Reconstruct the path
    path_cells = []
    current_cell = goal_cell
    while current_cell != start_cell:
        path_cells.append(current_cell)
        current_cell = came_from[current_cell]

    path_cells.append(start_cell)
    path_cells.reverse()
    print(f"Found path with {len(path_cells)} cells from {start_cell} to {goal_cell}")

    # turn the list of cells into a list of poses
    path = [initial_pose] # start with the initial pose
    for cell in path_cells[1:-1]:
        x = cell[0] * map.info.resolution + map.info.origin.position.x + map.info.resolution / 2
        y = cell[1] * map.info.resolution + map.info.origin.position.y + map.info.resolution / 2
        path.append(tuple_to_pose(x, y, 0.0)) # yaw is 0 for now
    path.append(goal_pose) # end with the goal pose

    return path

def get_neighbors(map: OccupancyGrid, cell: Cell) -> list[Cell]:
    """
    Get the neighboring cells of a given cell in the occupancy grid.
    """
    x, y = cell
    neighbors = [
        (x + 1, y), # right
        (x - 1, y), # left
        (x, y + 1), # up
        (x, y - 1), # down
    ]
    return [n for n in neighbors if 0 <= n[0] < map.info.width and 0 <= n[1] < map.info.height]

def is_cell_free(map: OccupancyGrid, x: int, y: int) -> bool:
    """
    Check if a cell in the occupancy grid is free (not occupied).
    """
    if x < 0 or x >= map.info.width or y < 0 or y >= map.info.height:
        return False
    index = y * map.info.width + x
    return map.data[index] == 0

def pose_to_cell(map: OccupancyGrid, pose: Pose) -> Cell:
    """
    Convert a pose to a cell in the occupancy grid.
    """
    x = int((pose.position.x - map.info.origin.position.x) / map.info.resolution)
    y = int((pose.position.y - map.info.origin.position.y) / map.info.resolution)
    return (x, y)

def get_vels_for_traj(x_odom: float, y_odom: float, theta_odom: float, x_map: float, y_map: float, theta_map: float, planned_trajectory: list[Pose], params: dict[str, float]) -> tuple[float, float, bool, dict[str, tuple[float, float]]]:
    """
    For a given trajectory and position, compute the linear and angular gains to follow the trajectory.
    Also returns whether the trajectory is done and some debug points.
    Arguments:
        x_odom, y_odom, theta_odom: the current position of the robot in the odom frame
        x_map, y_map, theta_map: the pose of the origin of the odom frame in the map frame
        planned_trajectory: a list of poses in the map frame
        params: a dictionary of: {"target_dist_forward": float, "linear_kp": float, "angular_kp": float}
    Returns:
        linear: the linear velocity to apply to the robot
        angular: the angular velocity to apply to the robot
        traj_done: whether the trajectory is done
        debug_points: a dictionary of debug points: {"closest_index": int, "closest_point": (x, y), "target_index": int, "target_point": (x, y)}
    """
    # path following algorithm:
    # figure out the closest point to us on the path
    # target a point a configurable distance along the path
    # p controllers for angular and linear velocity
    # as we move along the path, so too does the target point
    # this is the "pure pursuit" algorithm from the ftc days

    # first: convert the odom pose to the map frame
    init_matrix_map = np.array([[np.cos(theta_map), -np.sin(theta_map), x_map],
                                [np.sin(theta_map), np.cos(theta_map), y_map],
                                [0, 0, 1]])
    odom_matrix_odom = np.array([[np.cos(theta_odom), -np.sin(theta_odom), x_odom],
                                    [np.sin(theta_odom), np.cos(theta_odom), y_odom],
                                    [0, 0, 1]])
    odom_matrix_map = init_matrix_map @ odom_matrix_odom
    x_map, y_map = odom_matrix_map[0, 2], odom_matrix_map[1, 2]
    theta_map = np.arctan2(odom_matrix_map[1, 0], odom_matrix_map[0, 0])

    # convert list of poses to list of points
    path_points = np.array([[pose.position.x, pose.position.y] for pose in planned_trajectory])
    odom_point = np.array([x_map, y_map])

    # find the closest point on the path to the robot
    distances = np.linalg.norm(path_points - odom_point, axis=1)
    closest_index = np.argmin(distances)
    
    target_index = closest_index # start by targeting the closest point
    for i in range(closest_index, len(planned_trajectory)):
        # move away from the robot until we find a point acceptably far away
        if np.linalg.norm(path_points[i] - odom_point) > params["target_dist_forward"]:
            target_index = i
            break
    else:
        # if we didn't find a point far enough away, just target the last point
        target_index = len(planned_trajectory) - 1

    target_point = path_points[target_index]
    closest_point = path_points[closest_index]

    # take the target point back to odom frame for error computation
    target_matrix_map = np.array([[np.cos(theta_map), -np.sin(theta_map), target_point[0]],
                                    [np.sin(theta_map), np.cos(theta_map), target_point[1]],
                                    [0, 0, 1]])
    target_matrix_odom = np.linalg.inv(init_matrix_map) @ target_matrix_map
    target_x_odom, target_y_odom = target_matrix_odom[0, 2], target_matrix_odom[1, 2]

    # P controllers for linear and angular velocity
    vector_between = np.array([target_x_odom, target_y_odom]) - np.array([x_odom, y_odom])
    linear_error = np.linalg.norm(vector_between)
    angular_error = np.arctan2(vector_between[1], vector_between[0]) - theta_odom
    # wrap angular error to [-pi, pi]
    angular_error = (angular_error + np.pi) % (2 * np.pi) - np.pi

    linear = params["linear_kp"] * linear_error
    angular = params["angular_kp"] * angular_error
    print(f"v_l: {linear:.2f}, v_a: {angular:.2f}")

    traj_done = closest_index == len(planned_trajectory) - 1

    debug_points = {"closest_index": closest_index, "closest_point": closest_point, "target_index": target_index, "target_point": target_point}

    return linear, angular, traj_done, debug_points