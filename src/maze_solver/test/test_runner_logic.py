import pytest
from geometry_msgs.msg import Pose
from nav_msgs.msg import OccupancyGrid
import os
import yaml
from PIL import Image
import numpy as np
from maze_solver import runner_logic
from maze_solver.angle_helpers import tuple_to_pose, pose_to_tuple
# despite my best efforts, i couldn't get the map server to launch properly
# so we'll do this by hand
@pytest.fixture
def map() -> OccupancyGrid:
    root_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    map_file_path = os.path.join(root_dir, 'maps', 'fake_maze.yaml')
    with open(map_file_path, 'r') as f:
        map_yaml: dict = yaml.safe_load(f) # pyright: ignore[reportAssignmentType] # we just gotta trust that the yaml is good
    map_image_path = os.path.join(root_dir, 'maps', map_yaml['image'])
    map_image = Image.open(map_image_path)
    map_width, map_height = map_image.size
    map_data = list(map_image.get_flattened_data())
    map_data = [0 if pixel == 255 else 100 if pixel == 0 else -1 for pixel in map_data]
    map_msg = OccupancyGrid()
    map_msg.header.frame_id = "map"
    map_msg.info.width = map_width
    map_msg.info.height = map_height
    map_msg.info.resolution = map_yaml['resolution']
    map_msg.info.origin.position.x = map_yaml['origin'][0]
    map_msg.info.origin.position.y = map_yaml['origin'][1]
    map_msg.info.origin.position.z = 0.0
    # orientation is a quaternion, so we need to convert from euler angles
    roll, pitch, yaw = 0.0, 0.0, map_yaml['origin'][2]
    map_msg.info.origin.orientation.x = 0.0
    map_msg.info.origin.orientation.y = 0.0
    map_msg.info.origin.orientation.z = np.sin(yaw / 2.0)
    map_msg.info.origin.orientation.w = np.cos(yaw / 2.0)
    # the data is a flat list of int8 values in row-major order
    # we need to flip the y-axis because the image origin is at the top-left corner
    # and the occupancy grid origin is at the bottom-left corner
    flipped_map_data = []
    for y in range(map_height):
        for x in range(map_width):
            flipped_map_data.append(map_data[(map_height - 1 - y) * map_width + x])
    map_msg.data = flipped_map_data
    return map_msg

# check our work
def test_map_loaded(map: OccupancyGrid):
    assert map.info.width > 0
    assert map.info.height > 0
    assert len(map.data) == map.info.width * map.info.height


# ok now for the fun stuff
def test_plan_trajectory(map: OccupancyGrid):
    start_pose = tuple_to_pose(0, -0.9, np.pi/2)
    goal_pose = tuple_to_pose(0, 0.9, np.pi/2)
    trajectory = runner_logic.plan_trajectory(map, start_pose, goal_pose)
    # data structure checks
    assert len(trajectory) > 0, "There are no poses in the trajectory"
    assert all(isinstance(p, Pose) for p in trajectory), "Not all elements in the trajectory are Poses"
    assert trajectory[0] == start_pose, "The first pose in the trajectory is not the start pose"
    assert trajectory[-1] == goal_pose, "The last pose in the trajectory is not the goal pose"

    # basic geometry checks
    raw_poses = [pose_to_tuple(p) for p in trajectory]
    deltas = [np.linalg.norm(np.array(raw_poses[i+1][:2]) - np.array(raw_poses[i][:2])) for i in range(len(raw_poses)-1)]
    assert all(delta < 0.5 for delta in deltas), f"Poses are too far apart: {[(i, delta) for i, delta in enumerate(deltas) if delta >= 0.5]}"
    # assumes map is 2x2 meters, with origin at (0,0) and resolution of 0.01 m/pixel
    assert all(-1 <= p.position.x <= 1 and -1 <= p.position.y <= 1 for p in trajectory), f"Poses are outside the map bounds: {[pose_to_tuple(p) for p in trajectory]}"
    assert all(-np.pi <= pose_to_tuple(p)[2] <= np.pi for p in trajectory), f"At least one pose has an invalid yaw angle: {[pose_to_tuple(p)[2] for p in trajectory]}"

    # maze checks
    maze_walls = [(x, y) for y in range(map.info.height) for x in range(map.info.width) if map.data[y * map.info.width + x] == 100]
    for p in trajectory:
        # convert pose to map coordinates
        map_x = int((p.position.x - map.info.origin.position.x) / map.info.resolution)
        map_y = int((p.position.y - map.info.origin.position.y) / map.info.resolution)
        assert (map_x, map_y) not in maze_walls, f"Pose {p} is in a wall at map coordinates {(map_x, map_y)}"

    for i in range(len(raw_poses)-1):
        # check that the line segment between two consecutive poses does not intersect any walls
        p1 = np.array(raw_poses[i][:2])
        p2 = np.array(raw_poses[i+1][:2])
        for wall in maze_walls:
            wall_pos = np.array([(wall[0] + 0.5) * map.info.resolution + map.info.origin.position.x,
                                 (wall[1] + 0.5) * map.info.resolution + map.info.origin.position.y])
            wall_size = np.array([map.info.resolution, map.info.resolution])
            # check if the line segment intersects the wall's bounding box
            intersects_wall = (min(p1[0], p2[0]) <= wall_pos[0] + wall_size[0] and max(p1[0], p2[0]) >= wall_pos[0] and
                min(p1[1], p2[1]) <= wall_pos[1] + wall_size[1] and max(p1[1], p2[1]) >= wall_pos[1])
            assert not intersects_wall, \
                f"Line segment between poses {trajectory[i]} and {trajectory[i+1]} intersects wall at map coordinates {wall}"