#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/OccupancyGrid.h>
#include <crazyswarm/VelocityWorld.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>

class Robot
{
public:
    Robot(ros::NodeHandle &nh, double hz)
    {
        collision_pub_ = nh.advertise<std_msgs::Bool>("/collision", 1);
        marker_pub_ = nh.advertise<visualization_msgs::Marker>("/visualization_marker", 1);
        crazyflie_cmd_vel_pub = nh.advertise<crazyswarm::VelocityWorld>("/cf3/cmd_velocity_world", 1);
        odom_pub_ = nh.advertise<nav_msgs::Odometry>("/odometry/filtered", 1);

        vicon_sub_ = nh.subscribe("/vicon/cf3/cf3", 1, &Robot::odom_callback, this);
        cmd_vel_sub_ = nh.subscribe("cmd_vel", 1, &Robot::cmd_vel_callback, this);
        map_sub_ = nh.subscribe("/map", 1, &Robot::map_callback, this);

        dt_ = 1.0 / hz;

        state_ = {0.0, 0.0, 0.0};
        desired_input_ = {0.0, 0.0};

        max_a_ = 3.5;
        max_v_ = 2.0;
        max_w_ = 1.8;

        vel_ = 0.0;
        acc_ = 0.0;

        map_received_ = false;
        odom_received_ = false;

        // ros service for resetting the robot

        nh.param("crazyflie_sim/frame_id", frame_id_, std::string("vicon/world"));
        nh.param("crazyflie_sim/child_frame_id", child_frame_id_, std::string("vicon/cf3/cf3"));

    }

    void odom_callback(const geometry_msgs::TransformStamped::ConstPtr &msg)
    {
        odom_received_ = true;

        tf::Quaternion q(
            msg->transform.rotation.x,
            msg->transform.rotation.y,
            msg->transform.rotation.z,
            msg->transform.rotation.w);

        tf::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);

        state_ = {msg->transform.translation.x, msg->transform.translation.y, yaw};

        nav_msgs::Odometry odom_msg;
        odom_msg.header.frame_id = frame_id_;
        odom_msg.header.stamp = ros::Time::now();
        odom_msg.pose.pose.position.x = msg->transform.translation.x;
        odom_msg.pose.pose.position.y = msg->transform.translation.y;

        tf::Quaternion quat_tf;
        quat_tf.setRPY(0,0,yaw);
        tf::quaternionTFToMsg(quat_tf, odom_msg.pose.pose.orientation);

        odom_pub_.publish(odom_msg);
    }

    void map_callback(const nav_msgs::OccupancyGrid::ConstPtr &msg)
    {

        map_received_ = true;
        map_ = *msg;

        for (int i = 0; i < map_.data.size(); i++)
        {
            if (map_.data[i] < 0)
                map_.data[i] = 0;
        }

        grid_map::GridMapRosConverter::fromOccupancyGrid(map_, "layer", grid_map_);
    }

    void cmd_vel_callback(const geometry_msgs::Twist::ConstPtr &msg)
    {
        // std::vector<double> prev_desired_input = desired_input_;
        desired_input_ = {msg->linear.x, msg->angular.z};

        // // determine acceleration
        // acc_ = (desired_input_[0] - prev_desired_input[0]) / dt_;

        // // limit acceleration
        // acc_ = std::max(std::min(acc_, max_a_), -max_a_);

        // // set velocity based on acceleration
        // desired_input_[0] = prev_desired_input[0] + acc_ * dt_;

        // ensure velocity is within limits
        desired_input_[0] = std::max(std::min(desired_input_[0], max_v_), -max_v_);
        desired_input_[1] = std::max(std::min(desired_input_[1], max_w_), -max_w_);

        // convert v, omega to vx vy
        double vx = desired_input_[0] * cos(state_[2]);
        double vy = desired_input_[0] * sin(state_[2]);
        double omega = desired_input_[1];

        crazyswarm::VelocityWorld cmd_msg;
        cmd_msg.header.stamp = ros::Time::now();
        cmd_msg.header.frame_id = frame_id_;
        cmd_msg.vel.x = vx;
        cmd_msg.vel.y = vy;
        cmd_msg.yawRate = omega;
    }

    void publishTrail()
    {
        return;
    }

    bool is_in_collision()
    {
        // width and height of robot
        double dx = .508;
        double dy = .43;

        std::vector<Eigen::Vector2d> footprint = {
            {dx / 2., dy / 2.},
            {dx / 2., -dy / 2.},
            {-dx / 2., -dy / 2.},
            {-dx / 2., dy / 2.}};

        Eigen::Matrix2d R;
        R << cos(state_[2]), -sin(state_[2]),
            sin(state_[2]), cos(state_[2]);

        // transform footprint to world frame
        for (Eigen::Vector2d &pt : footprint)
            pt = R * pt + Eigen::Vector2d(state_[0], state_[1]);

        // check if footprint is in collision
        grid_map::Polygon polygon;
        for (Eigen::Vector2d pt : footprint)
        {
            polygon.addVertex(pt);
        }

        for (grid_map::PolygonIterator iterator(grid_map_, polygon); !iterator.isPastEnd(); ++iterator)
        {
            if (grid_map_.at("layer", *iterator) == 100)
                return true;
        }

        // publish visualization marker
        visualization_msgs::Marker marker;
        marker.header.frame_id = frame_id_;
        marker.header.stamp = ros::Time();
        marker.ns = "footprint";
        marker.id = 0;
        marker.type = visualization_msgs::Marker::LINE_STRIP;
        marker.action = visualization_msgs::Marker::ADD;
        marker.scale.x = 0.03; // Line width
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        marker.color.a = 1.0;

        for (Eigen::Vector2d pt : footprint)
        {
            geometry_msgs::Point p;
            p.x = pt[0];
            p.y = pt[1];
            p.z = 0.0;
            marker.points.push_back(p);
        }

        marker.points.push_back(marker.points[0]);
        marker_pub_.publish(marker);

        return false;
    }

    void publish_transforms()
    {

        // transform between baselink and odom
        // geometry_msgs::TransformStamped transformStamped;

        // transformStamped.header.stamp = ros::Time::now();
        // transformStamped.header.frame_id = frame_id_;
        // transformStamped.child_frame_id = child_frame_id_;

        // // subtract pi/2 from yaw because robot forward is along y-axis
        // transformStamped.transform.translation.x = state_[0];
        // transformStamped.transform.translation.y = state_[1];
        // transformStamped.transform.translation.z = 0.0;
        // transformStamped.transform.rotation = tf::createQuaternionMsgFromYaw(state_[2]); // - M_PI / 2);

        // tf_broadcaster_.sendTransform(transformStamped);

        // transform between map and odom
        // transformStamped.header.frame_id = "map";
        // transformStamped.child_frame_id = frame_id_;

        // transformStamped.transform.translation.x = 0.0;
        // transformStamped.transform.translation.y = 0.0;
        // transformStamped.transform.translation.z = 0.0;
        // transformStamped.transform.rotation = tf::createQuaternionMsgFromYaw(0.0);

        // tf_broadcaster_.sendTransform(transformStamped);
    }

    void spin()
    {
        ros::AsyncSpinner spinner(1);
        spinner.start();

        ros::waitForShutdown();
    }

    bool initialized;

protected:
    ros::NodeHandle nh_;

    ros::Publisher trail_pub_;
    ros::Publisher collision_pub_;
    ros::Publisher marker_pub_;
    ros::Publisher crazyflie_cmd_vel_pub;
    ros::Publisher imu_pub_;
    ros::Publisher odom_pub_;

    ros::Subscriber cmd_vel_sub_;
    ros::Subscriber vicon_sub_;
    ros::Subscriber map_sub_;

    tf2_ros::TransformBroadcaster tf_broadcaster_;

    ros::Timer timer_;

    nav_msgs::OccupancyGrid map_;

    grid_map::GridMap grid_map_;

    std::vector<double> state_;

    std::vector<double> desired_input_;
    std::vector<std::tuple<double, double>> trail_;

    double dt_;

    double vel_;
    double acc_;
    double max_a_;
    double max_v_;
    double max_w_;

    double init_x_;
    double init_y_;
    double init_theta_;

    std::string frame_id_;
    std::string child_frame_id_;


    bool map_received_;
    bool odom_received_;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "unicycle_sim");
    ros::NodeHandle nh;

    Robot robot(nh, 50.);
    robot.spin();

    return 0;
}
