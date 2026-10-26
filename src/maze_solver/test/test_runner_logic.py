import pytest
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Twist
from nav_msgs.msg import OccupancyGrid
import os
import yaml
from PIL import Image
import numpy as np
# despite my best efforts, i couldn't get the map server to launch properly
# so we'll do this by hand
@pytest.fixture
def map() -> OccupancyGrid:
    root_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    map_file_path = os.path.join(root_dir, 'maps', 'fake_maze.yaml')
    with open(map_file_path, 'r') as f:
        map_yaml = yaml.safe_load(f)
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
# TODO
# :)