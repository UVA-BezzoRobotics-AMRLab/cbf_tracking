#! /usr/bin/env python

import os
import math
import rospy
import numpy as np
import xml.dom.minidom

from lxml import etree
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose, Point, Quaternion


def parse_xml_file(xml_file):
    # parse the xml file

    cylinder_data = []

    tree = etree.parse(xml_file)
    root = tree.getroot()

    all_models = root.findall(".//model")

    unit_cylinder_models = [
        model
        for model in all_models
        if model.get("name").startswith("unit_cylinder_")
        and model.find("static") is not None
    ]

    for model in unit_cylinder_models:
        pose_element = model.find("./pose")
        pose = pose_element.text.strip().split()
        position = [float(p) for p in pose[:3]]
        radius = 0.075

        # shift everything back 5m in y direction
        cylinder_data.append([position[0], position[1] - 5.0, radius])

    return cylinder_data


def generate_map_from_cylinders(cylinders, theta, dx, dy):
    grid = OccupancyGrid()

    grid.info.width = 400
    grid.info.height = 400
    grid.info.resolution = 0.05
    grid.info.origin = Pose(Point(-10.0, -10.0, 0), Quaternion(0, 0, 0, 1))

    grid.data = [-1] * grid.info.width * grid.info.height

    R = np.array([[np.cos(theta), -np.sin(theta)], [np.sin(theta), np.cos(theta)]])

    for cylinder in cylinders:
        x, y, radius = cylinder

        [x, y] = np.matmul(R, np.array([x, y])) + [dx, dy]

        grid_x_center = int((x - grid.info.origin.position.x) / grid.info.resolution)
        grid_y_center = int((y - grid.info.origin.position.y) / grid.info.resolution)

        # Calculate the radius in grid cells
        radius_cells = int(radius / grid.info.resolution)

        # Mark cells within the circular obstacle boundary as occupied (100)
        for grid_x in range(
            grid_x_center - radius_cells, grid_x_center + radius_cells + 1
        ):
            for grid_y in range(
                grid_y_center - radius_cells, grid_y_center + radius_cells + 1
            ):
                # Check if the grid cell is within the grid bounds
                if (
                    grid_x >= 0
                    and grid_x < grid.info.width
                    and grid_y >= 0
                    and grid_y < grid.info.height
                ):
                    # Calculate distance from grid cell to obstacle center
                    dist = (
                        math.sqrt(
                            (grid_x - grid_x_center) ** 2
                            + (grid_y - grid_y_center) ** 2
                        )
                        * grid.info.resolution
                    )
                    if dist <= radius:
                        idx = grid_y * grid.info.width + grid_x
                        grid.data[idx] = 100

    return grid


def generate_corridor_occupancy_grid(
    squares, grid_resolution, grid_width, grid_height, origin
):
    grid = OccupancyGrid()

    grid.info.width = grid_width
    grid.info.height = grid_height
    grid.info.resolution = grid_resolution
    grid.info.origin = Pose(Point(origin[0], origin[1], 0), Quaternion(0, 0, 0, 1))

    grid.data = [-1] * grid.info.width * grid.info.height

    # add squares to grid data
    for square in squares:
        x_origin_idx = int(
            (square[0] - grid.info.origin.position.x) / grid.info.resolution
        )
        y_origin_idx = int(
            (square[1] - grid.info.origin.position.y) / grid.info.resolution
        )
        width = int(square[2] / grid.info.resolution)
        height = int(square[3] / grid.info.resolution)

        for x in range(x_origin_idx, x_origin_idx + width):
            for y in range(y_origin_idx, y_origin_idx + height):
                if x < 0 or x >= grid.info.width or y < 0 or y >= grid.info.height:
                    continue
                idx = y * grid.info.width + x
                grid.data[idx] = 100

    return grid


def main():
    rospy.init_node("occupancy_grid_publisher")

    world_file = rospy.get_param("~world_file", None)
    corridor = rospy.get_param("~corridor", False)
    rot = rospy.get_param("~rotation", 0)
    dx = rospy.get_param("~dx", 0)
    dy = rospy.get_param("~dy", 0)

    occupancy_grid_msg = OccupancyGrid()

    if corridor:
        origin = [-10.0, -10.0]
        square_data = [(-1.7, -0.5, 5, 7), (-7.6, -0.5, 5, 7), (-2.7, 9.5, 1, 1)]
        resolution = 0.05
        width = 400
        height = 400
        occupancy_grid_msg = generate_corridor_occupancy_grid(
            square_data, resolution, width, height, origin
        )

    # Replace with your XML file path
    elif world_file.endswith(".world"):

        if not os.path.exists(world_file):
            raise ValueError("Please provide a world file path")

        obstacles = parse_xml_file(world_file)

        # Generate occupancy grid message from obstacles
        occupancy_grid_msg = generate_map_from_cylinders(obstacles, rot, dx, dy)

    else:
        raise ValueError("Invalid arguments")
        exit(-1)

    # Publish the occupancy grid message (assuming you have a ROS node initialized)
    rospy.init_node("occupancy_grid_publisher")
    pub = rospy.Publisher("/map", OccupancyGrid, queue_size=10, latch=True)

    rate = rospy.Rate(1)

    while not rospy.is_shutdown():
        pub.publish(occupancy_grid_msg)
        rate.sleep()


if __name__ == "__main__":
    main()
