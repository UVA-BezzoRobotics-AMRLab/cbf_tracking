#!/usr/bin/env python3

import rospy

from nav_msgs.msg import OccupancyGrid
from generate_occ_grid import (
    generate_occupancy_grid,
    generate_empty_occupancy_grid,
    create_occupancy_grid_msg,
    serialize_occupancy_grid_msg,
    generate_corridor_occupancy_grid
)


def circle_occ_grid(grid_resolution, grid_width, grid_height):
    # Define circle data (x_center, y_center, radius)
    circle_data = [
        (2.121, -.879, .3),
        (2.121, -5.121, .6),
        (-2.121, -5.121, .15),
        (-2.121, -.879, .4),
        (3, -3, .4),
        (0, -3, 2.3)
    ]

    # Generate occupancy grid
    occupancy_grid = generate_occupancy_grid(
        circle_data, grid_resolution, grid_width, grid_height)

    return occupancy_grid


def corridor_occ_grid(grid_resolution, grid_width, grid_height):
    # define square by origin, width, height
    square_data = [
        (3, -20, 2, 19.5),
        (3, .5, 2, 19.5)
    ]

    occupancy_grid = generate_corridor_occupancy_grid(
        square_data, grid_resolution, grid_width, grid_height)

    return occupancy_grid


if __name__ == "__main__":
    rospy.init_node('occupancy_grid_publisher')

    # Define grid parameters
    grid_resolution = 0.02  # meters per cell
    grid_width = 1200  # number of cells
    grid_height = 1200  # number of cells

    occupancy_grid = circle_occ_grid(grid_resolution, grid_width, grid_height)

    # occupancy_grid = corridor_occ_grid(
    #     grid_resolution, grid_width, grid_height)

    # Create OccupancyGrid message
    occupancy_grid_msg = create_occupancy_grid_msg(
        occupancy_grid, grid_resolution, grid_width, grid_height)

    occupancy_grid_msg.header.stamp = rospy.Time.now()

    # Publish the OccupancyGrid message
    pub = rospy.Publisher(
        '/local_costmap/local_costmap/costmap', OccupancyGrid, queue_size=10)
    rate = rospy.Rate(1)  # 1 Hz

    while not rospy.is_shutdown():
        pub.publish(occupancy_grid_msg)
        rate.sleep()
