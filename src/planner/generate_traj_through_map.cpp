#include <ros/ros.h>
#include <tf2_ros/buffer.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <tf2_ros/transform_listener.h>

#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <visualization_msgs/Marker.h>
#include <trajectory_msgs/JointTrajectory.h>

#include <faster/solver.hpp>
#include <robust_fast_navigation/corridor.h>
#include <robust_fast_navigation/tinycolormap.hpp>

#include <fstream>

class TrajectoryGenerator
{
public:
    TrajectoryGenerator(ros::NodeHandle &nh) : _nh(nh)
    {
        // _odom = Eigen::Vector3d(-2.25, 2.5, M_PI / 2);
        _odom = Eigen::Vector3d(.4,0,0);

        double limits[3] = {1.2, 1.5, 2.0};

        solver.setN(6);
        solver.createVars();
        solver.setDC(.05);
        solver.setBounds(limits);
        solver.setForceFinalConstraint(true);
        solver.setFactorInitialAndFinalAndIncrement(1, 10, 1.0);
        solver.setThreads(0);
        solver.setWMax(2.0);
        solver.setVerbose(0);
        solver.setUseMinvo(false);
        solver.setMaxSolverTime(1.0);
    }

    void spin()
    {
        tf2_ros::Buffer tfBuffer;
        tf2_ros::TransformListener tfListener(tfBuffer);

        ROS_INFO("Waiting for costmap...");

        _costmap = std::make_shared<costmap_2d::Costmap2DROS>("global_costmap", tfBuffer);
        _costmap->start();
        ROS_INFO("Costmap started!");

        _map_sub = _nh.subscribe("/map", 1, &TrajectoryGenerator::mapcb, this);

        _odom_pub = _nh.advertise<nav_msgs::Odometry>("/gmapping/odometry", 1, true);
        _trajVizPub = _nh.advertise<visualization_msgs::Marker>("/MINCO_path", 1, true);

        ros::spin();
    }

    bool generate_traj()
    {
        publish_odom();

        _costmap->updateMap();

        bool simplify_jps = true;
        double curr_dist_horiz = 8.0;
        Eigen::MatrixXd initialPVAJ, finalPVAJ;
        std::vector<Eigen::Vector2d> jpsPath;
        std::vector<Eigen::MatrixX4d> hPolys;

        initialPVAJ = Eigen::MatrixXd::Zero(3, 4);
        finalPVAJ = Eigen::MatrixXd::Zero(3, 4);

        initialPVAJ << Eigen::Vector3d(_odom(0), _odom(1), 0),
            Eigen::Vector3d(0, 0, 0),
            Eigen::Vector3d(0, 0, 0),
            Eigen::Vector3d(0, 0, 0);

        // compute final position based on the current position and orientation
        finalPVAJ << Eigen::Vector3d(_odom(0) + 10 * cos(_odom(2)),
                                     _odom(1) + 10 * sin(_odom(2)),
                                     0),
            Eigen::Vector3d(0, 0, 0),
            Eigen::Vector3d(0, 0, 0),
            Eigen::Vector3d(0, 0, 0);

        ROS_INFO("odom: %.2f, %.2f, %.2f", _odom(0), _odom(1), _odom(2));
        ROS_INFO("finalPVAJ: %.2f, %.2f, %.2f", finalPVAJ(0, 0), finalPVAJ(1, 0), finalPVAJ(2, 0));

        // run solver boiler plate
        if (
            !solver_boilerplate(true,
                                curr_dist_horiz,
                                *_costmap->getCostmap(),
                                _odom,
                                initialPVAJ,
                                finalPVAJ,
                                jpsPath))
            return false;

        occupancy_grid_t occ_grid = costmap_to_occgrid(_costmap->getCostmap());
        
        if (
            !corridor::createCorridorJPS(jpsPath,
                                         occ_grid,
                                         hPolys,
                                         initialPVAJ,
                                         finalPVAJ))
            return false;

        // generate trajectory
        state initialState;
        state finalState;

        initialState.setPos(initialPVAJ(0, 0), initialPVAJ(1, 0), initialPVAJ(2, 0));
        initialState.setVel(initialPVAJ(0, 1), initialPVAJ(1, 1), initialPVAJ(2, 1));
        initialState.setAccel(initialPVAJ(0, 2), initialPVAJ(1, 2), initialPVAJ(2, 2));
        initialState.setJerk(initialPVAJ(0, 3), initialPVAJ(1, 3), initialPVAJ(2, 3));

        finalState.setPos(finalPVAJ.col(0));
        finalState.setVel(finalPVAJ.col(1));
        finalState.setAccel(finalPVAJ.col(2));
        finalState.setJerk(finalPVAJ.col(3));

        // ROS_INFO("setting up");
        solver.setX0(initialState);
        solver.setXf(finalState);
        solver.setPolytopes(hPolys);

        ros::Time start_solve = ros::Time::now();
        if (!solver.genNewTraj())
        {
            ROS_ERROR("solver could not find trajectory");
            return false;
        }
        else
        {
            ROS_INFO("solver found trajectory");
        }

        ROS_INFO("time to solve is %.4f", (ros::Time::now() - start_solve).toSec());

        solver.fillX();

        _generated_traj =
            convertTrajToMsg(solver.X_temp_, .05, "map");

        // save trajectory to file
        std::ofstream file;
        file.open("./traj.csv");
        for (trajectory_msgs::JointTrajectoryPoint p : _generated_traj.points)
        {
            file << p.positions[0] << "," << p.positions[1] << "," << p.positions[2] << ","
                 << p.velocities[0] << "," << p.velocities[1] << "," << p.velocities[2] << ","
                 << p.accelerations[0] << "," << p.accelerations[1] << "," << p.accelerations[2] << ","
                 << p.time_from_start.toSec() << std::endl;
        }

        visualizeTraj();

        return true;
    }

protected:
    void mapcb(const nav_msgs::OccupancyGrid::ConstPtr &msg)
    {
        _map = *msg;
        ROS_INFO("Map received! Generating trajectory...");
        if (!generate_traj())
        {
            ROS_ERROR("Failed to generate trajectory");
        }
    }

    void publish_odom()
    {
        nav_msgs::Odometry odom;
        odom.header.stamp = ros::Time::now();
        odom.header.frame_id = "map";
        odom.child_frame_id = "base_link";
        odom.pose.pose.position.x = _odom(0);
        odom.pose.pose.position.y = _odom(1);
        odom.pose.pose.position.z = 0.0;
        // convert yaw to quaternion
        tf2::Quaternion q;
        q.setRPY(0, 0, _odom(2));
        odom.pose.pose.orientation.x = q.x();
        odom.pose.pose.orientation.y = q.y();
        odom.pose.pose.orientation.z = q.z();
        odom.pose.pose.orientation.w = q.w();
        _odom_pub.publish(odom);
    }

    void visualizeTraj()
    {

        visualization_msgs::Marker msg;
        msg.header.frame_id = "map";
        msg.header.stamp = ros::Time::now();
        msg.ns = 'planTraj';
        msg.id = 80;
        msg.action = visualization_msgs::Marker::ADD;
        msg.type = visualization_msgs::Marker::LINE_STRIP;

        msg.scale.x = .1;
        msg.pose.orientation.w = 1;

        for (trajectory_msgs::JointTrajectoryPoint p : _generated_traj.points)
        {
            Eigen::Vector3d vel_vec(p.velocities[0],
                                    p.velocities[1],
                                    p.velocities[2]);
            double vel = vel_vec.norm();

            tinycolormap::Color color = tinycolormap::GetColor(vel / 1.0, tinycolormap::ColormapType::Plasma);

            std_msgs::ColorRGBA color_msg;
            color_msg.r = color.r();
            color_msg.g = color.g();
            color_msg.b = color.b();
            color_msg.a = 1.0;
            msg.colors.push_back(color_msg);

            geometry_msgs::Point point_msg;
            point_msg.x = p.positions[0];
            point_msg.y = p.positions[1];
            point_msg.z = p.positions[2];
            msg.points.push_back(point_msg);
        }

        _trajVizPub.publish(msg);
    }

    SolverGurobi solver;

    ros::NodeHandle _nh;

    trajectory_msgs::JointTrajectory _generated_traj;

    ros::Publisher _odom_pub;
    ros::Publisher _trajVizPub;

    ros::Subscriber _map_sub;

    nav_msgs::OccupancyGrid _map;

    std::shared_ptr<costmap_2d::Costmap2DROS> _costmap;

    Eigen::Vector3d _odom;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "planner");
    ros::NodeHandle nh;

    TrajectoryGenerator tg(nh);
    tg.spin();

    return 0;
}
