#include <map>
#include <chrono>
#include <fstream>
#include <iostream>
#include <ros/ros.h>
#include <Eigen/Core>
#include <nav_msgs/OccupancyGrid.h>
#include "filter/filter_nlopt_horizon.h"

#include <distance_map_core/distance_map_converter_base.h>
#include <distance_map_core/distance_map_converter_instantiater.h>

nav_msgs::OccupancyGrid occ_map;
bool map_received = false;
void map_cb(const nav_msgs::OccupancyGrid::ConstPtr &msg)
{
    occ_map = *msg;
    map_received = true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_mpc_solver");
    ros::NodeHandle nh;

    Eigen::VectorXd state(6);
    Eigen::MatrixXd reference;

    // load state from file
    std::ifstream state_file("/home/nick/catkin_ws/src/cbf_tracking/state.txt");
    if (state_file.is_open())
    {
        for (int i = 0; i < 6; i++)
        {
            state_file >> state(i);
        }
        state_file.close();
    }
    else
    {
        std::cout << "Unable to open state file" << std::endl;
    }

    // load reference from file
    std::ifstream reference_file("/home/nick/catkin_ws/src/cbf_tracking/wpts.txt");
    int row, col;
    if (reference_file.is_open())
    {
        reference_file >> row;
        reference_file >> col;
        reference.resize(row, col);
        for (int i = 0; i < col; i++)
        {
            for (int j = 0; j < row; j++)
            {
                reference_file >> reference(j, i);
            }
        }
        reference_file.close();
    }
    else
    {
        std::cout << "Unable to open reference file" << std::endl;
    }

    std::cout << reference << std::endl;
    std::cout << "**************" << std::endl;
    std::cout << state << std::endl;

    ros::Rate rate(10);

    // subscribe to map
    ros::Subscriber map_sub = nh.subscribe("/occupancy_grid", 1, map_cb);
    while (!map_received)
    {
        ros::spinOnce();
        rate.sleep();
    }

    // make filter object
    CBFHorizon filter;
    
    boost::shared_ptr<distmap::DistanceMapConverterBase> dist_map_conv;
    dist_map_conv = distmap::make_distance_mapper("distmap/DistanceMapDeadReck");
    if (!dist_map_conv->process(boost::make_shared<const nav_msgs::OccupancyGrid>(occ_map)))
    {
        std::cout << "Distance map failed to be set from occupancy grid" << std::endl;
        return false;
    }

    filter.set_dist_map(dist_map_conv->getDistanceFieldObstacle());

    // params
    std::map<std::string, double> params;
    params["DT"] = .1;
    params["STEPS"] = 16;
    params["USE_CBF"] = true;
    params["W_POS"] = 2;
    params["W_V"] = .8;
    params["W_CTE"] = 1;
    params["W_ETHETA"] = 1;
    params["W_ANGVEL"] = .3;
    params["W_DANGVEL"] = 1;
    params["W_DA"] = .5;
    params["MAX_LINACC"] = 3.3;
    params["LINVEL"] = 1.8;
    params["ANGVEL"] = 1.5;
    params["CBF_ALPHA"] = .4;
    params["CBF_COLINEAR"] = .01;
    params["CBF_PADDING"] = .1;
    params["CBF_DYNAMIC_ALPHA"] = false;

    filter.LoadParams(params);

    // time solve process
    auto start = std::chrono::high_resolution_clock::now();
    filter.Solve(state, reference);
    auto end = std::chrono::high_resolution_clock::now();
    std::cout << "Time: " << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() << "ms" << std::endl;

    // print x, y, theta, v horizons from mpc
    for (int i = 0; i < 16; ++i)
    {
        std::cout << "x: " << filter.mpc_x[i] << "\ty: " << filter.mpc_y[i] << "\ttheta: " << filter.mpc_theta[i] << "\tv: " << filter.mpc_linvels[i] << std::endl;
    }
}
