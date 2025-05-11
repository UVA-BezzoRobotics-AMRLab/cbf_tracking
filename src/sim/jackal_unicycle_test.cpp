#include "tf/tf.h"
#include "ros/ros.h"
#include "nav_msgs/Path.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PointStamped.h"
#include "visualization_msgs/MarkerArray.h"
#include "trajectory_msgs/JointTrajectory.h"

#include "filter/safety_filter_unicycle.h"

class Robot
{
public:
    Robot(ros::NodeHandle &nh);
    std::vector<double> getState();
    std::vector<std::tuple<double, double, double>> getTrajectory(int deriv);
    void addObstacles(const std::vector<std::tuple<double, double, double>> &obstacles);
    void addGoal(const std::vector<double> &goal);
    std::vector<double> evalTraj(double t);
    void publishTrail();

    // callbacks
    void trajectoryCallback(const trajectory_msgs::JointTrajectory::ConstPtr &msg);
    void odomCallback(const nav_msgs::Odometry::ConstPtr &msg);

    bool initialized;

private:
    ros::NodeHandle nh_;
    ros::Publisher pose_pub_;
    ros::Publisher goal_pub_;
    ros::Publisher trail_pub_;
    ros::Publisher obstacle_pub_;

    ros::Subscriber odom_sub_;
    ros::Subscriber trajectory_sub_;

    geometry_msgs::Pose2D pose_;
    visualization_msgs::MarkerArray obstacles_;

    std::vector<std::tuple<double, double>> trail_;
    std::vector<std::tuple<double, double, double>> trajectory_;
    std::vector<std::tuple<double, double, double>> points_;

};

Robot::Robot(ros::NodeHandle &nh) : nh_(nh)
{
    trail_pub_ = nh_.advertise<nav_msgs::Path>("trail", 1, true);
    pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("robot_pose", 1, true);
    goal_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("goal_pose", 1, true);
    obstacle_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("obstacles", 1, true);

    odom_sub_ = nh_.subscribe("/gmapping/odometry", 1, &Robot::odomCallback, this);
    trajectory_sub_ = nh_.subscribe("/trajectory", 1, &Robot::trajectoryCallback, this);

    pose_.x = 0.0;     // Initial X position
    pose_.y = 0.0;     // Initial Y position
    pose_.theta = 0.0; // Initial heading

    initialized = false;
}

void Robot::publishTrail()
{
    // Publish the updated pose
    nav_msgs::Path trail_msg;
    trail_msg.header.frame_id = "map";
    trail_msg.header.stamp = ros::Time::now();
    for (size_t i = 0; i < trail_.size(); ++i)
    {
        geometry_msgs::PoseStamped pose_msg;
        pose_msg.header.frame_id = "map";
        pose_msg.header.stamp = ros::Time::now();
        pose_msg.pose.position.x = std::get<0>(trail_[i]);
        pose_msg.pose.position.y = std::get<1>(trail_[i]);
        trail_msg.poses.push_back(pose_msg);
    }

    trail_pub_.publish(trail_msg);
}

void Robot::trajectoryCallback(const trajectory_msgs::JointTrajectory::ConstPtr &msg)
{
    if (trajectory_.size() > 0)
        return;

    ROS_INFO("Received trajectory");

    // save trajectory velocities into trajectory_ vector
    for (trajectory_msgs::JointTrajectoryPoint point : msg->points)
    {
        points_.push_back(std::make_tuple(point.positions[0], point.positions[1], point.time_from_start.toSec()));
        trajectory_.push_back(std::make_tuple(point.velocities[0], point.velocities[1], point.time_from_start.toSec()));
    }
}

void Robot::odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
    // save odometry into pose_ variable
    pose_.x = msg->pose.pose.position.x;
    pose_.y = msg->pose.pose.position.y;
    pose_.theta = tf::getYaw(msg->pose.pose.orientation);
    initialized = true;
}

std::vector<double> Robot::getState()
{
    return {pose_.x, pose_.y, pose_.theta};
}

std::vector<std::tuple<double, double, double>> Robot::getTrajectory(int deriv)
{
    if (deriv == 0)
        return points_;

    return trajectory_;
}

void Robot::addGoal(const std::vector<double> &goal)
{
    geometry_msgs::PoseStamped goal_msg;
    goal_msg.header.frame_id = "map";
    goal_msg.header.stamp = ros::Time::now();
    goal_msg.pose.position.x = goal[0];
    goal_msg.pose.position.y = goal[1];

    goal_pub_.publish(goal_msg);
}

void Robot::addObstacles(const std::vector<std::tuple<double, double, double>> &obstacles)
{
    obstacles_.markers.clear();

    for (size_t i = 0; i < obstacles.size(); ++i)
    {
        visualization_msgs::Marker obstacle;
        obstacle.header.frame_id = "map";
        obstacle.header.stamp = ros::Time::now();
        obstacle.id = i;
        obstacle.type = visualization_msgs::Marker::SPHERE;
        obstacle.action = visualization_msgs::Marker::ADD;

        double x = std::get<0>(obstacles[i]);
        double y = std::get<1>(obstacles[i]);
        double diameter = 2 * std::get<2>(obstacles[i]);

        obstacle.scale.x = diameter;
        obstacle.scale.y = diameter;
        obstacle.scale.z = diameter;
        obstacle.color.r = 1.0;
        obstacle.color.g = 0.0;
        obstacle.color.b = 0.0;
        obstacle.color.a = 1.0;
        obstacle.pose.position.x = x;
        obstacle.pose.position.y = y;
        obstacle.pose.position.z = 0.5; // Adjust height as needed
        obstacles_.markers.push_back(obstacle);
    }

    obstacle_pub_.publish(obstacles_);
}

std::vector<double> Robot::evalTraj(double t)
{

    if (t > std::get<2>(trajectory_.back()))
    {
        return {0, 0, std::get<0>(points_.back()), std::get<1>(points_.back())};
    }

    // evaluate trajectory at time t
    std::vector<double> x = {0, 0};
    std::vector<double> u = {0, 0};
    for (size_t i = 0; i < trajectory_.size(); ++i)
    {
        if (t < std::get<2>(trajectory_[i]))
        {
            u = {std::get<0>(trajectory_[i]), std::get<1>(trajectory_[i])};
            x = {std::get<0>(points_[i]), std::get<1>(points_[i])};
            break;
        }
    }

    return {u[0], u[1], x[0], x[1]};
}

std::vector<double> getClosestObs(
    const std::vector<double> &_x,
    const std::vector<std::tuple<double, double, double>> &obstacles)
{
    double min_dist = 1e9;
    double obs_x = 0;
    double obs_y = 0;
    double obs_r = 0;
    for (size_t i = 0; i < obstacles.size(); ++i)
    {
        double x = std::get<0>(obstacles[i]);
        double y = std::get<1>(obstacles[i]);
        double r = std::get<2>(obstacles[i]);
        double dist = std::pow(x - _x[0], 2) + std::pow(y - _x[1], 2);
        if (dist < min_dist)
        {
            min_dist = dist;
            obs_x = x;
            obs_y = y;
            obs_r = r;
        }
    }

    return {obs_x, obs_y, obs_r};
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "robot_node");
    ros::NodeHandle nh;

    double hz = 10.;

    // Simulate robot motion and update its state
    // You can replace this with your control logic
    double x = 0;
    double y = 0;
    double theta = 0;

    // double gx = 11.5;
    // double gy = 3;

    double v_max = 1.0;

    double k = 1.5;
    double k_theta = 1.2;

    // make robot
    Robot robot(nh);

    // make safety filter
    SafetyFilterUnicycle safety_filter;

    // Define obstacles as (x, y, radius) tuples
    std::vector<std::tuple<double, double, double>> obstacles;
    obstacles.push_back(std::make_tuple(1, -1, .3)); // Obstacle 2: x=6.5, y=2.5, radius=2

    robot.addObstacles(obstacles);

    bool started = false;
    ros::Time start_time;
    ros::Rate loop_rate(hz);

    ros::Publisher ref_pub = nh.advertise<geometry_msgs::PointStamped>("ref", 1, true);
    ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);

    while (ros::ok())
    {
        loop_rate.sleep();
        ros::spinOnce();

        std::vector<std::tuple<double, double, double>> trajectory = robot.getTrajectory(0);
        if (trajectory.size() == 0 || !robot.initialized){
            if (trajectory.size() == 0)
                ROS_INFO("Waiting for trajectory");
            else
                ROS_INFO("Waiting for odometry");
            continue;
        }

        if (!started)
        {
            start_time = ros::Time::now();
            started = true;
        }

        double t = (ros::Time::now() - start_time).toSec();

        // calculate velocity and make sure it is less than v_max
        std::vector<double> u_x = robot.evalTraj(t);
        double gx = u_x[2];
        double gy = u_x[3];

        std::vector<double> robo_state = robot.getState();
        x = robo_state[0];
        y = robo_state[1];
        theta = robo_state[2];

        std::vector<double> u_x_end = robot.evalTraj(1e6);

        if (std::pow(u_x_end[2] - x, 2) + std::pow(u_x_end[3] - y, 2) < 0.05)
        {
            ROS_INFO("Goal reached!");
            break;
        }

        double v = k * std::sqrt(std::pow(gx - x, 2) + std::pow(gy - y, 2));
        if (v > v_max)
        {
            v = v_max;
        }

        // calculate angular velocity
        double theta_des = std::atan2(gy - y, gx - x);

        // ensure difference between theta and theta_des is between -pi and pi
        double theta_diff = theta_des - theta;
        double theta_diff_normalized = std::atan2(std::sin(theta_diff), std::cos(theta_diff));

        // calculate angular velocity
        double w = k_theta * theta_diff_normalized;
        // double vx = v * (gx - x) / std::sqrt(std::pow(gx - x, 2) + std::pow(gy - y, 2));
        // double vy = v * (gy - y) / std::sqrt(std::pow(gx - x, 2) + std::pow(gy - y, 2));
        // std::vector<double> u_unfiltered = robot.evalTraj(t);
        // double vx = u_unfiltered[0];
        // double vy = u_unfiltered[1];

        // publish reference from trajectory
        geometry_msgs::PointStamped ref_msg;
        ref_msg.header.frame_id = "map";
        ref_msg.header.stamp = ros::Time::now();
        ref_msg.point.x = gx;
        ref_msg.point.y = gy;
        ref_pub.publish(ref_msg);

        // find closest obstacle to robot
        std::vector<double> obs = {std::get<0>(obstacles[0]),
                                   std::get<1>(obstacles[0]),
                                   std::get<2>(obstacles[0])}; // getClosestObs({x, y}, obstacles);

        // filter velocity
        std::vector<double> u = safety_filter.filter({x, y, theta, v}, {w}, obs);

        // update velocity
        w = u[0];

        ROS_INFO("{%.2f, %.2f}", w, v);
        geometry_msgs::Twist vel_msg;
        vel_msg.linear.x = v;
        vel_msg.angular.z = w;
        vel_pub.publish(vel_msg);

        robot.publishTrail();
    }

    return 0;
}
