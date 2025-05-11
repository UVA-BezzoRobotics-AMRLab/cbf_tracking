#!/usr/bin/env python3

import yaml
import numpy as np

from geometry_msgs.msg import Pose
from nav_msgs.msg import OccupancyGrid, MapMetaData
from yaml import CLoader as Loader, CDumper as Dumper


def generate_occupancy_grid(
        circle_data,
        grid_resolution,
        grid_width,
        grid_height
):
    # Initialize the grid with free space (0)
    occupancy_grid = np.zeros((grid_height, grid_width), dtype=np.int8)

    # Convert circle data to grid coordinates and mark occupied cells
    for x_center, y_center, radius in circle_data:
        x_center_idx = int(
            (x_center + grid_width * grid_resolution / 2) / grid_resolution)
        y_center_idx = int(
            (y_center + grid_height * grid_resolution / 2) / grid_resolution)
        radius_idx = int(radius / grid_resolution)

        print(x_center_idx, y_center_idx, radius_idx)

        for i in range(
            max(0, x_center_idx - radius_idx),
            min(grid_width, x_center_idx + radius_idx + 1)
        ):
            for j in range(
                max(0, y_center_idx - radius_idx),
                min(grid_height, y_center_idx + radius_idx + 1)
            ):
                if (i - x_center_idx)**2 + (j - y_center_idx)**2 \
                        <= radius_idx**2:
                    occupancy_grid[j, i] = 100  # Mark as occupied

    return occupancy_grid


def generate_empty_occupancy_grid(
        grid_resolution,
        grid_width,
        grid_height
):
    # Initialize the grid with free space (0)
    occupancy_grid = np.zeros((grid_height, grid_width), dtype=np.int8)

    return occupancy_grid


def generate_corridor_occupancy_grid(
        squares,
        grid_resolution,
        grid_width,
        grid_height
):

    occupancy_grid = np.zeros((grid_height, grid_width), dtype=np.int8)

    for square in squares:
        x_origin_idx = int(
            (square[0] + grid_width * grid_resolution / 2) / grid_resolution)
        y_origin_idx = int(
            (square[1] + grid_height * grid_resolution / 2) / grid_resolution)
        width_idx = int(square[2] / grid_resolution)
        height_idx = int(square[3] / grid_resolution)

        for i in range(x_origin_idx, x_origin_idx + width_idx):
            for j in range(y_origin_idx, y_origin_idx + height_idx):
                if i < grid_width and j < grid_height:
                    occupancy_grid[j, i] = 100

    return occupancy_grid


def create_occupancy_grid_msg(
    occupancy_grid,
    resolution,
    width,
    height
):
    occupancy_grid_msg = OccupancyGrid()

    occupancy_grid_msg.header.frame_id = 'odom'

    # Populate the metadata
    metadata = MapMetaData()
    metadata.resolution = resolution
    metadata.width = width
    metadata.height = height
    metadata.origin.position.x = -width * resolution / 2
    metadata.origin.position.y = -height * resolution / 2
    metadata.origin.position.z = 0
    occupancy_grid_msg.info = metadata

    # Flatten the 2D occupancy grid and assign it to the data field
    occupancy_grid_msg.data = occupancy_grid.flatten().tolist()

    return occupancy_grid_msg


def load_occ_grid_msg_from_file(occ_grid_file):
    # file is a yaml in the form of occupancy grid message

    with open(occ_grid_file, 'r') as f:
        grid_dict = yaml.load(f, Loader=yaml.CLoader)

    occupancy_grid_msg = OccupancyGrid()

    occupancy_grid_msg.header.seq = grid_dict['header']['seq']
    occupancy_grid_msg.header.stamp.secs = grid_dict['header']['stamp']['secs']
    occupancy_grid_msg.header.stamp.nsecs = grid_dict['header']['stamp']['nsecs']
    occupancy_grid_msg.header.frame_id = grid_dict['header']['frame_id']

    occupancy_grid_msg.info.map_load_time.secs = grid_dict['info']['map_load_time']['secs']
    occupancy_grid_msg.info.map_load_time.nsecs = grid_dict['info']['map_load_time']['nsecs']
    occupancy_grid_msg.info.resolution = grid_dict['info']['resolution']
    occupancy_grid_msg.info.width = grid_dict['info']['width']
    occupancy_grid_msg.info.height = grid_dict['info']['height']
    occupancy_grid_msg.info.origin.position.x = grid_dict['info']['origin']['position']['x']
    occupancy_grid_msg.info.origin.position.y = grid_dict['info']['origin']['position']['y']
    occupancy_grid_msg.info.origin.position.z = grid_dict['info']['origin']['position']['z']
    occupancy_grid_msg.info.origin.orientation.x = grid_dict['info']['origin']['orientation']['x']
    occupancy_grid_msg.info.origin.orientation.y = grid_dict['info']['origin']['orientation']['y']
    occupancy_grid_msg.info.origin.orientation.z = grid_dict['info']['origin']['orientation']['z']
    occupancy_grid_msg.info.origin.orientation.w = grid_dict['info']['origin']['orientation']['w']

    occupancy_grid_msg.data = grid_dict['data']

    return occupancy_grid_msg


def serialize_occupancy_grid_msg(grid):
    grid_dict = {
        "header": {
            "seq": grid.header.seq,
            "stamp": {
                "secs": grid.header.stamp.secs,
                "nsecs": grid.header.stamp.nsecs
            },
            "frame_id": grid.header.frame_id
        },
        "info": {
            "map_load_time": {
                "secs": grid.info.map_load_time.secs,
                "nsecs": grid.info.map_load_time.nsecs
            },
            "resolution": grid.info.resolution,
            "width": grid.info.width,
            "height": grid.info.height,
            "origin": {
                "position": {
                    "x": grid.info.origin.position.x,
                    "y": grid.info.origin.position.y,
                    "z": grid.info.origin.position.z
                },
                "orientation": {
                    "x": grid.info.origin.orientation.x,
                    "y": grid.info.origin.orientation.y,
                    "z": grid.info.origin.orientation.z,
                    "w": grid.info.origin.orientation.w
                }
            }
        },
        "data": list(grid.data)
    }

    return grid_dict
