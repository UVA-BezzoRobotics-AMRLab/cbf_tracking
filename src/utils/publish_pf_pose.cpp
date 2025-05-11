
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

nav_msgs::Odometry msg;
bool is_init;

tf2_ros::Buffer tf_buffer;
ros::Publisher gmappingOdomPub;

std::string frame_id;
std::string child_frame_id;

void odomcb(const nav_msgs::Odometry::ConstPtr& msg){
    geometry_msgs::TransformStamped odom_to_map = tf_buffer.lookupTransform(frame_id, child_frame_id, ros::Time(0), ros::Duration(1.0));
    nav_msgs::Odometry gmappingOdom;
    gmappingOdom.header.frame_id=frame_id;
    gmappingOdom.header.stamp=ros::Time::now();
    tf2::doTransform(msg->pose.pose, gmappingOdom.pose.pose, odom_to_map);
    gmappingOdom.twist.twist = msg->twist.twist;
    gmappingOdomPub.publish(gmappingOdom);
}

int main(int argc, char** argv){
    ros::init(argc, argv, "publish_pf_pose");
    ros::NodeHandle nh;

    tf2_ros::TransformListener tf2_listener(tf_buffer);

    nh.param("publish_pf_pose/frame_id", frame_id, std::string("map"));
    nh.param("publish_pf_pose/child_frame_id", child_frame_id, std::string("odom"));

    ros::Subscriber odomSub = nh.subscribe<nav_msgs::Odometry>("/odometry/filtered", 1, &odomcb);
    gmappingOdomPub = nh.advertise<nav_msgs::Odometry>("/gmapping/odometry", 10);

    ros::spin();
    return 0;
}
