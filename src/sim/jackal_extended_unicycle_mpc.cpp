#include "tf/tf.h"
#include "ros/ros.h"
#include "nav_msgs/Path.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/PointStamped.h"
#include "std_msgs/Float32MultiArray.h"
#include "visualization_msgs/MarkerArray.h"
#include "trajectory_msgs/JointTrajectory.h"

#include "filter/safety_filter_extended_unicycle.h"

class Robot
{
public:
    Robot(ros::NodeHandle &nh, double hz);

    void publishTrail();

    std::vector<double> getState();
    std::vector<double> getDesiredInput();
    std::vector<std::tuple<double, double>> getInputHorizon();

    std::vector<double> evalTraj(double t);

    void addGoal(const std::vector<double> &goal);
    void addObstacles(const std::vector<std::tuple<double, double, double>> &obstacles);

    void applyInput(const std::vector<double> &u);

    // callbacks
    void odomCallback(const nav_msgs::Odometry::ConstPtr &msg);
    void cmdvelCallback(const geometry_msgs::Twist::ConstPtr &msg);
    void horizonCallback(const std_msgs::Float32MultiArray::ConstPtr &msg);

    bool initialized;

    ros::NodeHandle nh_;
    ros::Publisher cmd_pub_;
    ros::Publisher pose_pub_;
    ros::Publisher goal_pub_;
    ros::Publisher trail_pub_;
    ros::Publisher obstacle_pub_;

    ros::Subscriber odom_sub_;
    ros::Subscriber cmdvel_sub_;
    ros::Subscriber horizon_sub_;

    geometry_msgs::Pose2D pose_;
    std::vector<double> state_;
    visualization_msgs::MarkerArray obstacles_;

    std::vector<double> desired_input_;
    std::vector<std::tuple<double, double>> input_horizon_;
    std::vector<std::tuple<double, double>> trail_;
    std::vector<std::tuple<double, double, double>> trajectory_;
    std::vector<std::tuple<double, double, double>> points_;

    double dt_;
};

Robot::Robot(ros::NodeHandle &nh, double hz) : nh_(nh), dt_(1. / hz)
{
    cmd_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    trail_pub_ = nh_.advertise<nav_msgs::Path>("trail", 1, true);
    pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("robot_pose", 1, true);
    goal_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("goal_pose", 1, true);
    obstacle_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("obstacles", 1, true);

    cmdvel_sub_ = nh_.subscribe("/mpc_vel", 1, &Robot::cmdvelCallback, this);
    odom_sub_ = nh_.subscribe("/odometry/filtered", 1, &Robot::odomCallback, this);
    // horizon_sub_ = nh_.subscribe("/mpc_horizon", 1, &Robot::horizonCallback, this);

    pose_.x = 0.0;     // Initial X position
    pose_.y = 0.0;     // Initial Y position
    pose_.theta = 0.0; // Initial heading

    desired_input_ = {0, 0};     // {a,w}
    state_ = {0, 0, 0, 0, 0, 0}; // {x,y,theta,v,x_dot,y_dot}

    initialized = false;
}

void Robot::publishTrail()
{
    // Publish the updated pose
    nav_msgs::Path trail_msg;
    trail_msg.header.frame_id = "odom";
    trail_msg.header.stamp = ros::Time::now();
    for (size_t i = 0; i < trail_.size(); ++i)
    {
        geometry_msgs::PoseStamped pose_msg;
        pose_msg.header.frame_id = "odom";
        pose_msg.header.stamp = ros::Time::now();
        pose_msg.pose.position.x = std::get<0>(trail_[i]);
        pose_msg.pose.position.y = std::get<1>(trail_[i]);
        trail_msg.poses.push_back(pose_msg);
    }

    trail_pub_.publish(trail_msg);
}

std::vector<std::tuple<double, double>> Robot::getInputHorizon()
{
    return input_horizon_;
}

void Robot::horizonCallback(const std_msgs::Float32MultiArray::ConstPtr &msg)
{
    if (msg->data.size() == 0)
        return;

    input_horizon_.clear();

    // save trajectory velocities into trajectory_ vector
    for (size_t i = 0; i < msg->data.size() - 1; i += 2)
    {
        // i is linvel, i+1 is angvel
        input_horizon_.push_back({msg->data[i], msg->data[i + 1]});
    }
}

void Robot::cmdvelCallback(const geometry_msgs::Twist::ConstPtr &msg)
{
    // this will be acceleration and angular velocity
    desired_input_ = {msg->linear.x, msg->angular.z};
}

void Robot::odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
    // save odometry into pose_ variable
    pose_.x = msg->pose.pose.position.x;
    pose_.y = msg->pose.pose.position.y;
    pose_.theta = tf::getYaw(msg->pose.pose.orientation);

    state_[0] = pose_.x;
    state_[1] = pose_.y;
    state_[2] = pose_.theta;

    initialized = true;
}

std::vector<double> Robot::getState()
{
    return state_;
}

std::vector<double> Robot::getDesiredInput()
{
    return desired_input_;
}

void Robot::addGoal(const std::vector<double> &goal)
{
    geometry_msgs::PoseStamped goal_msg;
    goal_msg.header.frame_id = "odom";
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
        obstacle.header.frame_id = "odom";
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
        obstacle.pose.orientation.w = 1.0;
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

void Robot::applyInput(const std::vector<double> &u)
{

    // update state
    state_[3] = state_[3] + u[0] * dt_;
    state_[4] = state_[3] * cos(pose_.theta);
    state_[5] = state_[3] * sin(pose_.theta);

    geometry_msgs::Twist vel_msg;
    vel_msg.linear.x = state_[3];
    vel_msg.angular.z = u[1];
    cmd_pub_.publish(vel_msg);
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

double evaluateHFunction(double x, double y, double theta, double v, double x_dot, double y_dot,
                         double a, double w, double obs_x, double obs_y, double obs_r, bool print = false)
{
    double alpha = .4;
    double d2 = 1;
    double dist = sqrt((x - obs_x) * (x - obs_x) + (y - obs_y) * (y - obs_y));

    double D = dist - obs_r;
    double p0 = ((obs_x - x) * cos(theta) + (obs_y - y) * sin(theta)) / dist;
    double pp = (-(obs_y - y) * cos(theta) + (obs_x - x) * sin(theta)) / dist;
    double P = p0 + v * d2;

    double Lfh1 = (exp(-P) * x_dot / dist) * ((x - obs_x) + D * (cos(theta) + (x - obs_x) * p0 / dist));
    double Lfh2 = (exp(-P) * y_dot / dist) * ((y - obs_y) + D * (sin(theta) + (y - obs_y) * p0 / dist));
    double Lfh = Lfh1 + Lfh2;

    double Lgh1 = -D * d2 * exp(-P);
    double Lgh2 = D * pp * exp(-P);

    double h = D * exp(-P);
    double constraint = Lfh + Lgh1 * a + Lgh2 * w + alpha * h;

    if (print)
    {
        // ROS_INFO("Lfh: %.2f", Lfh);
        // ROS_INFO("D: %.2f", D);
        // ROS_INFO("p0: %.2f", p0);
        // ROS_INFO("Lgh1: %.2f", Lgh1);
        // ROS_INFO("Lgh2: %.2f", Lgh2);
        // ROS_INFO("h: %.2f", h);
    }

    // return constraint;
    return h;
}

visualization_msgs::MarkerArray visualizeHFunction(
    Robot &robot,
    const std::vector<std::tuple<double, double, double>> &obstacles)
{
    double grid_resolution = .15;
    double grid_size = 3.0;
    int num_cells = static_cast<int>(grid_size / grid_resolution);

    std::vector<double> robo_state = robot.getState();
    double theta = robo_state[2];
    double v = robo_state[3];
    double x_dot = robo_state[4];
    double y_dot = robo_state[5];

    std::vector<double> input = robot.getDesiredInput();
    double a = input[0];
    double w = input[1];

    std::vector<double> obs = {std::get<0>(obstacles[0]),
                               std::get<1>(obstacles[0]),
                               std::get<2>(obstacles[0])}; // getClosestObs({x, y}, obstacles);

    // create marker array
    visualization_msgs::MarkerArray markers;
    for (int i = 0; i < num_cells; ++i)
    {
        for (int j = 0; j < num_cells; ++j)
        {
            double grid_x = (i - num_cells / 2) * grid_resolution;
            double grid_y = (j - num_cells / 2) * grid_resolution;

            double safety_value = evaluateHFunction(grid_x, grid_y, theta, v, x_dot, y_dot, a, w, obs[0], obs[1], obs[2]);

            // Create a marker
            visualization_msgs::Marker marker;
            marker.header.frame_id = "odom";
            marker.header.stamp = ros::Time::now();
            marker.ns = "safety_markers";
            marker.id = i * num_cells + j;
            marker.type = visualization_msgs::Marker::CUBE;
            marker.action = visualization_msgs::Marker::ADD;

            // Set the pose of the marker
            marker.pose.position.x = grid_x;
            marker.pose.position.y = grid_y;
            marker.pose.position.z = 0.0;
            marker.pose.orientation.x = 0.0;
            marker.pose.orientation.y = 0.0;
            marker.pose.orientation.z = 0.0;
            marker.pose.orientation.w = 1.0;

            // Set the scale of the marker (assuming a 1x1m cell size)
            marker.scale.x = grid_resolution;
            marker.scale.y = grid_resolution;
            marker.scale.z = 0.1; // Adjust the height as needed

            // Set the color based on the safety function value in a binary fashion
            marker.color.r = 1.0;
            marker.color.g = 0.0;
            // marker.color.r = (safety_value < 0) ? 1.0 : 0.0;
            // marker.color.g = (safety_value < 0) ? 0.0 : 1.0;
            // marker.color.b = 0.;
            marker.color.a = (safety_value < 0) ? 1.0 : 0.0;
            // marker.color.a = 1;

            markers.markers.push_back(marker);
        }
    }

    return markers;
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
    double v = 0;
    double x_dot = 0;
    double y_dot = 0;

    // double gx = 11.5;
    // double gy = 3;

    double v_max = 1.0;

    double k = 1.5;
    double k_theta = 1.2;

    // make robot
    Robot robot(nh, hz);

    // make safety filter
    SafetyFilterUnicycle safety_filter;

    // Define obstacles as (x, y, radius) tuples
    std::vector<std::tuple<double, double, double>> obstacles;
    // obstacles.push_back(std::make_tuple(2, .75, .4)); // SIGMOID LEFT
    // obstacles.push_back(std::make_tuple(2, 1.25, .4)); // SIGMOID RIGHT
    // obstacles.push_back(std::make_tuple(1, 1, .6)); // HAND CRAFTED

    // far away obstacle
    // obstacles.push_back(std::make_tuple(3, -10, .6));

    // circle CCW
    // obstacles.push_back(std::make_tuple(2.121, .879, .3));
    // obstacles.push_back(std::make_tuple(2.121, 5.121, .6));
    // obstacles.push_back(std::make_tuple(-2.121, 5.121, .15));
    // obstacles.push_back(std::make_tuple(-2.121, .879, .4));

    // circle CW
    obstacles.push_back(std::make_tuple(2.121, -.879, .3));
    obstacles.push_back(std::make_tuple(2.121, -5.121, .6));
    obstacles.push_back(std::make_tuple(-2.121, -5.121, .15));
    obstacles.push_back(std::make_tuple(-2.121, -.879, .4));

    // cluttered env 
    // obstacles.push_back(std::make_tuple(8.5, 1.25, .5));
    // obstacles.push_back(std::make_tuple(7.5,-2.8, .5));
    // obstacles.push_back(std::make_tuple(10.6, 2.0, .5));
    // obstacles.push_back(std::make_tuple(9.3, 4.0, .5));
    // obstacles.push_back(std::make_tuple(6.7, 3.0, .5));
    // obstacles.push_back(std::make_tuple(6.77, -.43, .5));
    // obstacles.push_back(std::make_tuple(10.87, -1.03, .5));

    robot.addObstacles(obstacles);

    ros::Time start_time;
    ros::Rate loop_rate(hz);

    ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    ros::Publisher obs_pub = nh.advertise<geometry_msgs::Point>("curr_obstacle", 1);
    ros::Publisher ref_pub = nh.advertise<geometry_msgs::PointStamped>("ref", 1, true);
    ros::Publisher filter_states_pub = nh.advertise<nav_msgs::Path>("/filter_states", 1);
    ros::Publisher marker_pub = nh.advertise<visualization_msgs::MarkerArray>("safety_markers", 1, true);

    while (ros::ok())
    {
        loop_rate.sleep();
        ros::spinOnce();

        visualization_msgs::MarkerArray markers = visualizeHFunction(robot, obstacles);
        marker_pub.publish(markers);

        if (!robot.initialized)
            continue;

        std::vector<double> robo_state = robot.getState();
        x = robo_state[0];
        y = robo_state[1];
        theta = robo_state[2];
        v = robo_state[3];
        x_dot = robo_state[4];
        y_dot = robo_state[5];

        // find closest obstacle to robot
        std::vector<double> closest_obs = getClosestObs({x, y}, obstacles);

        geometry_msgs::Point curr_obs_msg;
        curr_obs_msg.x = closest_obs[0];
        curr_obs_msg.y = closest_obs[1];
        curr_obs_msg.z = closest_obs[2];
        obs_pub.publish(curr_obs_msg);

        // std::vector<std::tuple<double, double>> input_horizon = robot.getInputHorizon();
        // std::vector<double> as, ws;

        // for (size_t i = 0; i < input_horizon.size(); ++i)
        // {
        //     as.push_back(std::get<0>(input_horizon[i]));
        //     ws.push_back(std::get<1>(input_horizon[i]));

        //     if (i == 0)
        //         break;
        // }

        std::vector<double> input = robot.getDesiredInput();
        double a = input[0];
        double w = input[1];

        // std::vector<double> u = safety_filter.filter(
        //     {x, y, theta, v, x_dot, y_dot}, {a}, {w}, obs);

        std::vector<double> u = {a, w};

        // ROS_INFO("input is %f, %f", a, w);
        // ROS_INFO("filtered input is %f, %f", u[0], u[1]);
        // double h_function = evaluateHFunction(x, y, theta, v, x_dot, y_dot,
        //                                          {a}, {w}, obs[0], obs[1], obs[2], true);
        // ROS_INFO("h_function: %f", h_function);

        // if (h_function < 0)
        // {
        //     ROS_WARN("Safety constraint violated!");
        //     // exit(0);
        // }

        robot.obstacles_.markers.clear();
        for (size_t i = 0; i < obstacles.size(); ++i)
        {
            visualization_msgs::Marker obstacle;
            obstacle.header.frame_id = "odom";
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

            if (fabs(closest_obs[0]-x) < 1e-3 && fabs(closest_obs[1]-y) < 1e-3)
                obstacle.color.a = 1.0;
            else
                obstacle.color.a = .3;

            obstacle.pose.position.x = x;
            obstacle.pose.position.y = y;
            obstacle.pose.position.z = 0.5; // Adjust height as needed
            obstacle.pose.orientation.w = 1.0;
            robot.obstacles_.markers.push_back(obstacle);
        }

        robot.obstacle_pub_.publish(robot.obstacles_);

        // publish filter states in path message
        nav_msgs::Path filter_states_msg;
        filter_states_msg.header.frame_id = "odom";
        filter_states_msg.header.stamp = ros::Time::now();

        for (size_t i = 0; i < safety_filter.filter_x.size(); ++i)
        {
            geometry_msgs::PoseStamped pose_msg;
            pose_msg.header.frame_id = "odom";
            pose_msg.header.stamp = ros::Time::now();
            pose_msg.pose.position.x = safety_filter.filter_x[i];
            pose_msg.pose.position.y = safety_filter.filter_y[i];
            filter_states_msg.poses.push_back(pose_msg);
        }

        filter_states_pub.publish(filter_states_msg);

        robot.applyInput(u);
        robot.publishTrail();
    }

    return 0;
}
