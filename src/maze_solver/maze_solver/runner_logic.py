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