#include "tf/transform_datatypes.h"
#include <cbf_tracking/GetState.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Bool.h>
#include <std_srvs/Empty.h>
#include <tf/transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>

#include <grid_map_ros/GridMapRosConverter.hpp>
#include <grid_map_ros/grid_map_ros.hpp>

class Robot
{
public:
  Robot (ros::NodeHandle &nh, double hz)
  {
    odom_pub_ = nh.advertise<nav_msgs::Odometry> ("/odometry/filtered", 1);
    collision_pub_ = nh.advertise<std_msgs::Bool> ("/collision", 1);
    marker_pub_ = nh.advertise<visualization_msgs::Marker> (
        "/visualization_marker", 1);
    imu_pub_ = nh.advertise<sensor_msgs::Imu> ("/imu", 1);

    cmd_vel_sub_ = nh.subscribe ("cmd_vel", 1, &Robot::cmd_vel_callback, this);
    map_sub_ = nh.subscribe ("/map", 1, &Robot::map_callback, this);

    dt_ = 1.0 / hz;

    state_ = { 0.0, 0.0, 0.0 };
    desired_input_ = { 0.0, 0.0 };

    max_a_ = 3.5;
    max_v_ = 2.0;
    max_w_ = 1.8;

    vel_ = 0.0;
    acc_ = 0.0;

    is_paused = false;
    is_colliding_ = false;
    map_received_ = false;

    // setup timer to run update_state at dt_
    timer_ = nh.createTimer (ros::Duration (dt_), &Robot::update_state, this);

    // ros service for resetting the robot
    step_srv_ = nh.advertiseService ("step", &Robot::step_callback, this);
    reset_srv_ = nh.advertiseService ("reset", &Robot::reset_callback, this);
    pause_srv_ = nh.advertiseService ("pause", &Robot::pause_callback, this);
    state_srv_
        = nh.advertiseService ("get_state", &Robot::state_callback, this);

    nh.param ("unicycle_sim/x", init_x_, 0.0);
    nh.param ("unicycle_sim/y", init_y_, 0.0);
    nh.param ("unicycle_sim/yaw", init_theta_, 0.0);

    state_ = { init_x_, init_y_, init_theta_ };

    nh.param ("unicycle_sim/frame_id", frame_id_, std::string ("odom"));
    nh.param ("unicycle_sim/child_frame_id", child_frame_id_,
              std::string ("base_link"));
  }

  void
  map_callback (const nav_msgs::OccupancyGrid::ConstPtr &msg)
  {

    map_received_ = true;
    map_ = *msg;

    for (int i = 0; i < map_.data.size (); i++)
      {
        if (map_.data[i] < 0)
          map_.data[i] = 0;
      }

    grid_map::GridMapRosConverter::fromOccupancyGrid (map_, "layer",
                                                      grid_map_);
  }

  void
  cmd_vel_callback (const geometry_msgs::Twist::ConstPtr &msg)
  {
    // std::vector<double> prev_desired_input = desired_input_;
    desired_input_ = { msg->linear.x, msg->angular.z };

    if (is_colliding_)
      desired_input_ = { 0.0, 0.0 };

    // // determine acceleration
    // acc_ = (desired_input_[0] - prev_desired_input[0]) / dt_;

    // // limit acceleration
    // acc_ = std::max(std::min(acc_, max_a_), -max_a_);

    // // set velocity based on acceleration
    // desired_input_[0] = prev_desired_input[0] + acc_ * dt_;

    // ensure velocity is within limits
    desired_input_[0]
        = std::max (std::min (desired_input_[0], max_v_), -max_v_);
    desired_input_[1]
        = std::max (std::min (desired_input_[1], max_w_), -max_w_);
  }

  void
  publishTrail ()
  {
    return;
  }

  bool
  step_callback (std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
  {
    bool was_paused = is_paused;
    if (was_paused)
      is_paused = false;

    update_state (ros::TimerEvent ());

    if (was_paused)
      is_paused = true;

    return true;
  }

  bool
  reset_callback (std_srvs::Empty::Request &req,
                  std_srvs::Empty::Response &res)
  {
    state_ = { init_x_, init_y_, init_theta_ };
    desired_input_ = { 0.0, 0.0 };
    return true;
  }

  bool
  pause_callback (std_srvs::Empty::Request &req,
                  std_srvs::Empty::Response &res)
  {
    is_paused = true;
    return true;
  }

  bool
  state_callback (cbf_tracking::GetState::Request &req,
                  cbf_tracking::GetState::Response &res)
  {
    res.odom.pose.pose.position.x = state_[0];
    res.odom.pose.pose.position.y = state_[1];
    res.odom.pose.pose.orientation
        = tf::createQuaternionMsgFromYaw (init_theta_);

    return true;
  }

  bool
  is_in_collision ()
  {
    // width and height of robot
    double dx = .508;
    double dy = .43;

    std::vector<Eigen::Vector2d> footprint = { { dx / 2., dy / 2. },
                                               { dx / 2., -dy / 2. },
                                               { -dx / 2., -dy / 2. },
                                               { -dx / 2., dy / 2. } };

    Eigen::Matrix2d R;
    R << cos (state_[2]), -sin (state_[2]), sin (state_[2]), cos (state_[2]);

    // transform footprint to world frame
    for (Eigen::Vector2d &pt : footprint)
      pt = R * pt + Eigen::Vector2d (state_[0], state_[1]);

    // check if footprint is in collision
    grid_map::Polygon polygon;
    for (Eigen::Vector2d pt : footprint)
      {
        polygon.addVertex (pt);
      }

    for (grid_map::PolygonIterator iterator (grid_map_, polygon);
         !iterator.isPastEnd (); ++iterator)
      {
        if (grid_map_.at ("layer", *iterator) == 100)
          return true;
      }

    // publish visualization marker
    visualization_msgs::Marker marker;
    marker.header.frame_id = frame_id_;
    marker.header.stamp = ros::Time ();
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
        marker.points.push_back (p);
      }

    marker.points.push_back (marker.points[0]);
    marker_pub_.publish (marker);

    return false;
  }

  void
  publish_transforms ()
  {

    // transform between baselink and odom
    geometry_msgs::TransformStamped transformStamped;

    transformStamped.header.stamp = ros::Time::now ();
    transformStamped.header.frame_id = frame_id_;
    transformStamped.child_frame_id = child_frame_id_;

    // subtrack pi/2 from yaw because robot forward is along y-axis
    transformStamped.transform.translation.x = state_[0];
    transformStamped.transform.translation.y = state_[1];
    transformStamped.transform.translation.z = 0.0;
    transformStamped.transform.rotation
        = tf::createQuaternionMsgFromYaw (state_[2]); // - M_PI / 2);

    tf_broadcaster_.sendTransform (transformStamped);

    // transform between map and odom
    // transformStamped.header.frame_id = "map";
    // transformStamped.child_frame_id = frame_id_;

    // transformStamped.transform.translation.x = 0.0;
    // transformStamped.transform.translation.y = 0.0;
    // transformStamped.transform.translation.z = 0.0;
    // transformStamped.transform.rotation =
    // tf::createQuaternionMsgFromYaw(0.0);

    // tf_broadcaster_.sendTransform(transformStamped);
  }

  void
  update_state (const ros::TimerEvent &event)
  {
    if (is_paused)
      {
        return;
      }

    // check acceleration
    acc_ = (desired_input_[0] - vel_) / dt_;

    // limit acceleration
    acc_ = std::max (std::min (acc_, max_a_), -max_a_);

    // use discretized unicycle dynamics
    // double v0 = desired_input_[0];
    // double w0 = desired_input_[1];

    vel_ += acc_ * dt_;
    double v0 = vel_;
    double w0 = desired_input_[1];

    if (fabs (w0) < 1e-6)
      {
        state_[0] = state_[0] + v0 * dt_ * cos (state_[2]);
        state_[1] = state_[1] + v0 * dt_ * sin (state_[2]);
      }
    else
      {
        state_[0]
            = state_[0]
              + (v0 / w0) * (sin (state_[2] + w0 * dt_) - sin (state_[2]));
        state_[1]
            = state_[1]
              + (v0 / w0) * (-cos (state_[2] + w0 * dt_) + cos (state_[2]));
      }

    state_[2] = state_[2] + w0 * dt_;

    // wrap to [-pi, pi]
    state_[2] = atan2 (sin (state_[2]), cos (state_[2]));

    // publish odometry
    nav_msgs::Odometry odom;
    odom.header.stamp = ros::Time::now ();
    odom.header.frame_id = frame_id_;
    odom.child_frame_id = child_frame_id_;
    odom.pose.pose.position.x = state_[0];
    odom.pose.pose.position.y = state_[1];
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw (state_[2]);

    odom.twist.twist.linear.x = v0;
    odom.twist.twist.angular.z = w0;

    odom_pub_.publish (odom);

    // publish imu
    sensor_msgs::Imu imu;
    imu.header.stamp = ros::Time::now ();
    imu.header.frame_id = child_frame_id_;
    imu.orientation = tf::createQuaternionMsgFromYaw (state_[2]);
    imu.angular_velocity.z = w0;
    imu.linear_acceleration.x = acc_;
    imu_pub_.publish (imu);

    publish_transforms ();

    if (map_received_ && is_in_collision ())
      {
        std_msgs::Bool collision_msg;
        collision_msg.data = true;
        collision_pub_.publish (collision_msg);

        is_colliding_ = true;
      }
  }

  void
  spin ()
  {
    ros::AsyncSpinner spinner (1);
    spinner.start ();

    ros::waitForShutdown ();
  }

  bool initialized;
  bool is_paused;

protected:
  ros::NodeHandle nh_;

  ros::Publisher odom_pub_;
  ros::Publisher trail_pub_;
  ros::Publisher collision_pub_;
  ros::Publisher marker_pub_;
  ros::Publisher imu_pub_;

  ros::Subscriber cmd_vel_sub_;
  ros::Subscriber map_sub_;

  ros::ServiceServer step_srv_;
  ros::ServiceServer reset_srv_;
  ros::ServiceServer pause_srv_;
  ros::ServiceServer state_srv_;

  tf2_ros::TransformBroadcaster tf_broadcaster_;

  ros::Timer timer_;

  nav_msgs::OccupancyGrid map_;

  grid_map::GridMap grid_map_;

  std::vector<double> state_;

  std::vector<double> desired_input_;
  std::vector<std::tuple<double, double> > trail_;

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
  bool is_colliding_;
};

int
main (int argc, char **argv)
{
  ros::init (argc, argv, "unicycle_sim");
  ros::NodeHandle nh;

  Robot robot (nh, 50.);
  robot.spin ();

  return 0;
}
