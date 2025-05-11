#include <ros/ros.h>

#include <random>
#include <tf/tf.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <geometry_msgs/Pose.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/LaserScan.h>

class LidarSimulator
{
public:
  LidarSimulator () : tf_listener (tf_buffer), gen (rd ()), dis (0.0, 0.01)
  {
    // Initialize ROS node handle
    nh = ros::NodeHandle ();

    // Subscriber for ground truth map
    map_sub = nh.subscribe ("/map", 1, &LidarSimulator::mapCallback, this);

    // Publisher for LiDAR scan data
    lidar_pub = nh.advertise<sensor_msgs::LaserScan> ("/front/scan", 1);

    base_frame = "base_link";
    odom_frame = "odom";
    laser_frame = "front_laser";

    // base_frame = "vicon/cf3/cf3";
    // odom_frame = "vicon/world";
    // laser_frame = "front_laser";

    ros::Duration (2.5).sleep ();

    // Initialize LiDAR parameters
    initializeLidar ();

    // Simulate LiDAR scan
    simulateLidarScan ();
  }

  void
  mapCallback (const nav_msgs::OccupancyGrid::ConstPtr &msg)
  {
    // Store the received map data
    grid_map = *msg;
    map_initialized = true;
  }

  void
  initializeLidar ()
  {
    // Set LiDAR parameters
    lidar_scan.header.frame_id = laser_frame;
    lidar_scan.angle_min = -M_PI * .75;
    lidar_scan.angle_max = M_PI * .75;
    lidar_scan.angle_increment = 2. * M_PI / 720.; // Angular resolution
    lidar_scan.range_min = 0.1;                    // Minimum range
    lidar_scan.range_max = 30.0;                   // Maximum range
    lidar_scan.time_increment = 0.0;
    lidar_scan.scan_time = 0.0; // Time between scans (dummy value)

    // Calculate number of scan points
    lidar_scan.ranges.resize (
        static_cast<size_t> ((lidar_scan.angle_max - lidar_scan.angle_min)
                             / lidar_scan.angle_increment));
    lidar_scan.intensities.resize (lidar_scan.ranges.size ());

    // Initialize LiDAR pose
    lidar_pose.position.x = 0.0;
    lidar_pose.position.y = 0.0;
    lidar_pose.position.z = 0.0;
    lidar_pose.orientation
        = tf::createQuaternionMsgFromYaw (0.0); // Facing forward initially
  }

  void
  simulateLidarScan ()
  {
    ros::Rate rate (30); // 30 Hz update rate
    while (ros::ok ())
      {
        if (map_initialized)
          {
            // Update LiDAR scan based on current map and LiDAR pose
            updateLidarScan ();
            lidar_scan.header.stamp = ros::Time::now ();

            // Publish LiDAR scan
            lidar_pub.publish (lidar_scan);
          }

        try
          {
            // reset lidar pose
            lidar_pose.position.x = 0.0;
            lidar_pose.position.y = 0.0;
            lidar_pose.position.z = 0.0;
            lidar_pose.orientation = tf::createQuaternionMsgFromYaw (0.0);

            geometry_msgs::TransformStamped odom_to_laser
                = tf_buffer.lookupTransform (odom_frame, laser_frame,
                                             ros::Time (0));
            tf2::doTransform (lidar_pose, lidar_pose, odom_to_laser);

            // lidar_pose.position.x = odom_to_laser.transform.translation.x;
            // lidar_pose.position.y = odom_to_laser.transform.translation.y;
            // lidar_pose.position.z = odom_to_laser.transform.translation.z;

            // // Update LiDAR pose based on transform
            // lidar_pose.position.x =
            // transformStamped.transform.translation.x; lidar_pose.position.y
            // = transformStamped.transform.translation.y;
            // lidar_pose.position.z =
            // transformStamped.transform.translation.z; lidar_pose.orientation
            // = transformStamped.transform.rotation;

            // Broadcast the transform from base_link to front_laser
            // geometry_msgs::TransformStamped transformStamped;
            // transformStamped.header.stamp = ros::Time::now();
            // transformStamped.header.frame_id = base_frame;
            // transformStamped.child_frame_id = laser_frame;
            // transformStamped.transform.translation.x = 0.;
            // transformStamped.transform.translation.y = 0.;
            // transformStamped.transform.translation.z = 0.;
            // transformStamped.transform.rotation =
            // tf::createQuaternionMsgFromYaw(0.0);

            // tf_broadcaster.sendTransform(transformStamped);
          }
        catch (tf2::TransformException &ex)
          {
            ROS_WARN ("%s", ex.what ());
            ros::Duration (1.0).sleep ();
            continue;
          }

        ros::spinOnce ();
        rate.sleep ();
      }
  }

  void
  updateLidarScan ()
  {

    // use transform and castRay to get distances
    double yaw = tf::getYaw (lidar_pose.orientation);
    double angle = lidar_scan.angle_min;
    int index = 0;
    while (index < lidar_scan.ranges.size ())
      {
        double dx = cos (yaw + angle);
        double dy = sin (yaw + angle);

        // Cast rays to detect obstacles
        double range
            = castRay (lidar_pose.position.x, lidar_pose.position.y, dx, dy);

        // Update LiDAR scan data
        // add small gaussian noise to range
        lidar_scan.ranges[index] = range;
        lidar_scan.intensities[index] = 0.0;

        // Move to the next angle
        angle += lidar_scan.angle_increment;
        ++index;
      }
  }

  double
  castRay (double start_x, double start_y, double dx, double dy)
  {
    // Initialize variables
    double x = start_x;
    double y = start_y;

    // Calculate number of steps (cells) in the ray
    double dx_abs = fabs (dx);
    double dy_abs = fabs (dy);
    int steps = static_cast<int> (std::max (dx_abs, dy_abs)
                                  / grid_map.info.resolution);

    // Bresenham variables
    int x0 = static_cast<int> ((start_x - grid_map.info.origin.position.x)
                               / grid_map.info.resolution);
    int y0 = static_cast<int> ((start_y - grid_map.info.origin.position.y)
                               / grid_map.info.resolution);
    int x1 = static_cast<int> (
        (start_x + dx * lidar_scan.range_max - grid_map.info.origin.position.x)
        / grid_map.info.resolution);
    int y1 = static_cast<int> (
        (start_y + dy * lidar_scan.range_max - grid_map.info.origin.position.y)
        / grid_map.info.resolution);

    int dx_bres = abs (x1 - x0);
    int dy_bres = abs (y1 - y0);
    int sx = x0 < x1 ? 1 : -1;
    int sy = y0 < y1 ? 1 : -1;
    int err = (dx_bres > dy_bres ? dx_bres : -dy_bres) / 2;
    int err_tmp;

    while (true)
      {
        // ROS_INFO("x0: %d, y0: %d", x0, y0);
        // Check map boundaries
        if (x0 < 0 || x0 >= grid_map.info.width || y0 < 0
            || y0 >= grid_map.info.height)
          {
            break;
          }

        // Check for obstacle
        int map_index = y0 * grid_map.info.width + x0;
        if (grid_map.data[map_index] > 0)
          {
            // convert cell index to world coordinates
            double x_real = grid_map.info.origin.position.x
                            + x0 * grid_map.info.resolution;
            double y_real = grid_map.info.origin.position.y
                            + y0 * grid_map.info.resolution;
            // ROS_INFO("Obstacle detected at x: %d, y: %d", x0, y0);
            // ROS_INFO("Obstacle detected at x_real: %f, y_real: %f", x_real,
            // y_real); ROS_INFO("grid_map value is: %d",
            // grid_map.data[map_index]); Calculate range to obstacle
            double range = sqrt ((x_real - start_x) * (x_real - start_x)
                                 + (y_real - start_y) * (y_real - start_y));
            return range;
          }

        // Check if reached end of the line
        if (x0 == x1 && y0 == y1)
          {
            break;
          }

        err_tmp = err;
        if (err_tmp > -dx_bres)
          {
            err -= dy_bres;
            x0 += sx;
          }
        if (err_tmp < dy_bres)
          {
            err += dx_bres;
            y0 += sy;
          }
      }

    return lidar_scan.range_max; // No obstacle detected within range
  }

private:
  ros::NodeHandle nh;
  ros::Subscriber map_sub;
  ros::Publisher lidar_pub;
  nav_msgs::OccupancyGrid grid_map;
  sensor_msgs::LaserScan lidar_scan;
  geometry_msgs::Pose lidar_pose;
  std::string laser_frame;
  std::string base_frame;
  std::string odom_frame;
  tf2_ros::Buffer tf_buffer;
  tf2_ros::TransformListener tf_listener;
  tf2_ros::TransformBroadcaster tf_broadcaster;
  bool map_initialized = false;

  std::random_device rd;
  std::mt19937 gen;
  std::normal_distribution<> dis;
};

int
main (int argc, char **argv)
{
  ros::init (argc, argv, "lidar_simulator");

  LidarSimulator lidar_sim;

  ros::spin ();

  return 0;
}
