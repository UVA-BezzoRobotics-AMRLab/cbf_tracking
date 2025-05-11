#include <cmath>
#include <math.h>
#include <string>
#include <vector>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <std_msgs/Bool.h>
#include <tf/tf.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <unsupported/Eigen/Splines>

#include <robust_fast_navigation/corridor.h>
#include <robust_fast_navigation/planner_gurobi.h>
#include <robust_fast_navigation/spline.h>
#include <robust_fast_navigation/utils.h>

typedef Eigen::Spline<double, 1, 3> Spline1D;
typedef Eigen::SplineFitting<Spline1D> SplineFitting1D;

/**********************************************************************
  Constructor to read in ROS params, setup subscribers & publishers,
  ros timers, and initialize class fields.
***********************************************************************/
PlannerROS::PlannerROS (ros::NodeHandle &nh)
{

  // ROS Params
  nh.param ("robust_planner/w_max", _max_w, 3.);
  nh.param ("robust_planner/v_max", _max_vel, 1.0);
  nh.param ("robust_planner/a_max", _max_acc, 1.2);
  nh.param ("robust_planner/j_max", _max_jerk, 4.0);
  nh.param ("robust_planner/traj_dt", _traj_dt, .1);
  nh.param ("robust_planner/is_barn", _is_barn, false);
  nh.param ("robust_planner/teleop", _is_teleop, false);
  nh.param ("robust_planner/lookahead", _lookahead, .15);
  nh.param ("robust_planner/max_deviation", _max_dev, 1.);
  nh.param ("robust_planner/planner_frequency", _dt, .1);
  nh.param ("robust_planner/plan_once", _plan_once, false);
  nh.param ("robust_planner/use_arclen", _use_arclen, false);
  nh.param ("robust_planner/plan_in_free", _plan_in_free, false);
  nh.param ("robust_planner/simplify_jps", _simplify_jps, false);
  nh.param ("robust_planner/failsafe_count", _failsafe_count, 2);
  nh.param ("robust_planner/barn_goal_dist", _barn_goal_dist, 10.);
  nh.param<std::string> ("robust_planner/frame", _frame_str, "map");
  nh.param ("robust_planner/jps_hysteresis", _jps_hysteresis, false);
  nh.param ("robust_planner/max_dist_horizon", _max_dist_horizon, 4.);

  nh.param ("robust_planner/n_polys", _n_polys, 6);
  nh.param ("robust_planner/threads", _n_threads, 0);
  nh.param ("robust_planner/verbose", _solver_verbose, 0);
  nh.param ("robust_planner/use_minvo", _use_minvo, false);
  nh.param ("robust_planner/factor_init", _factor_init, 1.0);
  nh.param ("robust_planner/factor_final", _factor_final, 10.0);
  nh.param ("robust_planner/max_solve_time", _max_solve_time, .2);
  nh.param ("robust_planner/solver_traj_dt", _solver_traj_dt, .05);
  nh.param ("robust_planner/factor_increment", _factor_increment, 1.0);
  nh.param ("robust_planner/force_final_const", _force_final_const, true);

  // params
  _planner_params.W_MAX = _max_w;
  _planner_params.V_MAX = _max_vel;
  _planner_params.A_MAX = _max_acc;
  _planner_params.J_MAX = _max_jerk;
  _planner_params.DT_FACTOR_INIT = _factor_init;
  _planner_params.DT_FACTOR_FINAL = _factor_final;
  _planner_params.DT_FACTOR_INCREMENT = _factor_increment;
  _planner_params.SOLVER_TRAJ_DT = _solver_traj_dt;
  // this is actually number of polynomials in traj, not polys...
  _planner_params.N_SEGMENTS = _n_polys;
  _planner_params.N_THREADS = _n_threads;
  _planner_params.FORCE_FINAL_CONSTRAINT = _force_final_const;
  _planner_params.VERBOSE = _solver_verbose;
  _planner_params.USE_MINVO = _use_minvo;
  _planner_params.PLAN_IN_FREE = _plan_in_free;
  _planner_params.SIMPLIFY_JPS = _simplify_jps;
  _planner_params.MAX_SOLVE_TIME = _max_solve_time;

  _planner.set_params (_planner_params);

  // Publishers
  trajVizPub = nh.advertise<visualization_msgs::Marker> ("/MINCO_path", 0);
  wptVizPub = nh.advertise<visualization_msgs::Marker> ("/MINCO_wpts", 0);
  trajPub = nh.advertise<trajectory_msgs::JointTrajectory> (
      "/reference_trajectory", 0);
  trajPubNoReset = nh.advertise<trajectory_msgs::JointTrajectory> (
      "/reference_trajectory_no_reset", 0);

  meshPub
      = nh.advertise<visualization_msgs::Marker> ("/visualizer/mesh", 1000);
  edgePub
      = nh.advertise<visualization_msgs::Marker> ("/visualizer/edge", 1000);

  helperMeshPub = nh.advertise<visualization_msgs::Marker> (
      "/visualizer/mesh_helper", 1000);
  helperEdgePub = nh.advertise<visualization_msgs::Marker> (
      "/visualizer/edge_helper", 1000);

  initPointPub = nh.advertise<geometry_msgs::PointStamped> ("/initPoint", 0);

  goalPub
      = nh.advertise<geometry_msgs::PoseStamped> ("/global_planner/goal", 0);

  paddedLaserPub = nh.advertise<visualization_msgs::Marker> ("/paddedObs", 0);

  jpsPub = nh.advertise<nav_msgs::Path> ("/jpsPath", 0);
  jpsPubFree = nh.advertise<nav_msgs::Path> ("/jpsPathFree", 0);

  jpsPointsPub = nh.advertise<visualization_msgs::Marker> ("/jpsPoints", 0);

  corridorPub = nh.advertise<geometry_msgs::PoseArray> ("/polyCorridor", 0);

  intGoalPub
      = nh.advertise<geometry_msgs::PoseArray> ("/intermediate_goal", 0);

  cmdVelPub = nh.advertise<geometry_msgs::Twist> ("/cmd_vel", 0);

  initialPVAJPub = nh.advertise<trajectory_msgs::JointTrajectoryPoint> (
      "/initial_pvaj", 0);

  ctrlPointPub = nh.advertise<geometry_msgs::PoseArray> ("/ctrl_points", 0);

  unionCorridorPub
      = nh.advertise<visualization_msgs::Marker> ("/unionCorridor", 0);

  tubeVizPub
      = nh.advertise<visualization_msgs::MarkerArray> ("/tubeVizPub", 0);

  // Services
  estop_client = nh.serviceClient<std_srvs::Empty> ("/switch_mode");

  // Subscribers
  mapSub = nh.subscribe ("/map", 1, &PlannerROS::mapcb, this);
  goalSub = nh.subscribe ("/planner_goal", 1, &PlannerROS::goalcb, this);
  laserSub = nh.subscribe ("/front/scan", 1, &PlannerROS::lasercb, this);
  odomSub = nh.subscribe ("/odometry/filtered", 1, &PlannerROS::odomcb, this);
  occSub
      = nh.subscribe ("/occ_points", 10, &PlannerROS::occlusionPointscb, this);
  MPCHorizonSub
      = nh.subscribe ("/mpc_horizon", 1, &PlannerROS::mpcHorizoncb, this);
  clickedPointSub
      = nh.subscribe ("/clicked_point", 1, &PlannerROS::clickedPointcb, this);
  pathSub = nh.subscribe ("/global_planner/planner/plan", 1,
                          &PlannerROS::globalPathcb, this);

  // Timers
  goalTimer = nh.createTimer (ros::Duration (_dt / 2.0), &PlannerROS::goalLoop,
                              this);
  controlTimer
      = nh.createTimer (ros::Duration (_dt), &PlannerROS::controlLoop, this);
  publishTimer = nh.createTimer (ros::Duration (_dt * 2),
                                 &PlannerROS::publishOccupied, this);

  _is_occ = false;
  _is_init = false;
  _planned = false;
  _is_goal_set = false;
  _map_received = false;
  _primitive_started = false;
  _is_costmap_started = false;

  _prev_jps_path.clear ();

  _prev_jps_cost = -1;
  _curr_horizon = _max_dist_horizon;

  ROS_INFO ("Initialized planner!");

  sentTraj.points.clear ();
}

PlannerROS::~PlannerROS () {}

/**********************************************************************
  Function to start main ros loop for planning node. Also starts the
  costmap processes.

  TODOS: Add mutex's to class fields to allow for multi-threading
***********************************************************************/
void
PlannerROS::spin ()
{

  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener (tfBuffer);

  ROS_INFO ("initializing global costmap");
  global_costmap = std::make_unique<costmap_2d::Costmap2DROS> (
      "global_costmap", tfBuffer);
  ROS_INFO ("starting global costmap");
  global_costmap->start ();
  ROS_INFO ("done!");

  _is_costmap_started = true;

  ros::AsyncSpinner spinner (1);
  spinner.start ();

  ros::waitForShutdown ();
}

/**********************************************************************
  Function callback which reads goal position from a clicked point in
  RVIZ. When called, this function will set the _is_goal_set field to
  true, which is required for planning to begin.

  Inputs:
    - PointStamped message
***********************************************************************/
void
PlannerROS::clickedPointcb (const geometry_msgs::PointStamped::ConstPtr &msg)
{
  goal = Eigen::VectorXd (2);
  goal (0) = msg->point.x;
  goal (1) = msg->point.y;
  _is_goal_set = true;
  _is_goal_reset = true;
}

/**********************************************************************
  Function to project a point into the costmap for planning purposes.

  Inputs:
    - Goal coordinates

  TODOS: Implement :)
***********************************************************************/
void
PlannerROS::projectIntoMap (const Eigen::Vector2d &goal)
{
}

/**********************************************************************
  Function callback which reads a published goal pose. As with the
  clicked point callback function, this will set the _is_goal_set
  field to true, which is required for planning to begin.

  Inputs:
    - PoseStamped message
***********************************************************************/
void
PlannerROS::goalcb (const geometry_msgs::PoseStamped::ConstPtr &msg)
{
  goal = Eigen::VectorXd (2);
  goal (0) = msg->pose.position.x;
  goal (1) = msg->pose.position.y;

  _is_goal_set = true;
  _is_goal_reset = true;

  ROS_INFO ("goal received!");
}

void
PlannerROS::occlusionPointscb (const geometry_msgs::PoseArray::ConstPtr &msg)
{
  _occ_point = Eigen::MatrixXd (2, 3);

  if (msg->poses.size () > 0)
    {
      geometry_msgs::Pose p1 = msg->poses[0];
      geometry_msgs::Pose p2 = msg->poses[1];

      _occ_point.row (0)
          = Eigen::Vector3d (p1.position.x, p1.position.y, p1.position.z);
      _occ_point.row (1)
          = Eigen::Vector3d (p2.position.x, p2.position.y, p2.position.z);
      _is_occ = true;
    }
}

void
PlannerROS::mpcHorizoncb (
    const trajectory_msgs::JointTrajectory::ConstPtr &msg)
{
  mpcHorizon = *msg;
  _mpc_dt = mpcHorizon.points[1].time_from_start.toSec ()
            - mpcHorizon.points[0].time_from_start.toSec ();
}

/**********************************************************************
  Function which publishes the occupied cells of a global costmap.
***********************************************************************/
void
PlannerROS::publishOccupied (const ros::TimerEvent &)
{
  static int prev_points = -1;

  if (!_is_costmap_started)
    return;

  // costmap_2d::Costmap2D *_map = global_costmap->getCostmap();
  map_util::occupancy_grid_t occ_grid
      = map_util::costmap_to_occgrid (*global_costmap->getCostmap ());
  std::vector<Eigen::Vector2d> padded = corridor::getOccupied (occ_grid);

  visualization_msgs::Marker paddedMsg;
  paddedMsg.header.frame_id = _frame_str;
  paddedMsg.header.stamp = ros::Time::now ();
  paddedMsg.ns = "padded";
  paddedMsg.id = 420;
  paddedMsg.type = visualization_msgs::Marker::CUBE_LIST;
  paddedMsg.action = visualization_msgs::Marker::ADD;
  paddedMsg.scale.x = occ_grid.resolution;
  paddedMsg.scale.y = paddedMsg.scale.x;
  paddedMsg.scale.z = .1;
  paddedMsg.pose.orientation.w = 1;
  paddedMsg.color.r = 0.42;
  paddedMsg.color.g = 0.153;
  paddedMsg.color.b = 0.216;
  paddedMsg.color.a = .55;

  for (Eigen::Vector2d p : padded)
    {
      geometry_msgs::Point pMs;
      pMs.x = p[0];
      pMs.y = p[1];
      pMs.z = 0;
      paddedMsg.points.push_back (pMs);
    }

  paddedLaserPub.publish (paddedMsg);
}

/**********************************************************************
  Function callback which reads a published map. This map is different
  from the local and global costmap, since it doesn't contain an
  inflate layer around the obstacles.

  Inputs:
    - OccupancyGrid message

  TODOS: Decide if this map is needed or not for planning...
***********************************************************************/
void
PlannerROS::mapcb (const nav_msgs::OccupancyGrid::ConstPtr &msg)
{
  map = *msg;
  _map_received = true;
}

/**********************************************************************
  Function callback which reads in a published odometry message. To
  make arithmetic easier for planning purposes, it is converted in an
  (x,y,theta) format within an Eigen::Vector. The function also sets the
  _is_init flag to true, which is necessary for planning to begin.

  Inputs:
    - Odometry message

  TODOS: is vel field really necessary?
***********************************************************************/
void
PlannerROS::odomcb (const nav_msgs::Odometry::ConstPtr &msg)
{

  tf::Quaternion q (msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
                    msg->pose.pose.orientation.z,
                    msg->pose.pose.orientation.w);

  tf::Matrix3x3 m (q);
  double roll, pitch, yaw;
  m.getRPY (roll, pitch, yaw);

  _odom = Eigen::VectorXd (3);
  _odom (0) = msg->pose.pose.position.x;
  _odom (1) = msg->pose.pose.position.y;
  _odom (2) = yaw;

  if (_is_barn && !_is_goal_set)
    {
      goal = Eigen::VectorXd (2);
      goal (0) = _barn_goal_dist * cos (yaw) + _odom (0);
      goal (1) = _barn_goal_dist * sin (yaw) + _odom (1);
      _is_goal_set = true;

      ROS_INFO ("yaw is %.4f", yaw);
      ROS_INFO ("goal is %.4f\t%.4f", goal (0), goal (1));
    }

  _is_init = true;
}

/**********************************************************************
  Function callback which reads in a laserscan message. Only runs when
  _is_init flag is set to true in order to save on computation. In
  order to be useable by polygon generation code, the scan is saved
  into a vector of Eigen::Vector2d.

  Inputs:
    - LaserScan message

***********************************************************************/
void
PlannerROS::lasercb (const sensor_msgs::LaserScan::ConstPtr &msg)
{

  if (!_is_init)
    return;

  _obs.clear ();

  for (int i = 0; i < msg->ranges.size (); i++)
    {
      if (msg->ranges[i] > msg->range_max || msg->ranges[i] < msg->range_min)
        continue;

      double d = msg->ranges[i];

      double angle = msg->angle_min + i * msg->angle_increment + _odom (2);
      double x = d * cos (angle) + _odom (0);
      double y = d * sin (angle) + _odom (1);

      _obs.push_back (Eigen::Vector2f (x, y));
    }
}

/**********************************************************************
  Function callback which reads in a global path plan from an external
  node. This was used before JPS was implemented and will probably be
  removed in the near future if I don't find a need for it...

  Inputs:
    - Path message

  TODOS: Decide if this is a useful function now that JPS is
  implemented.
***********************************************************************/
void
PlannerROS::globalPathcb (const nav_msgs::Path::ConstPtr &msg)
{

  for (geometry_msgs::PoseStamped p : msg->poses)
    astarPath.push_back (
        Eigen::Vector2d (p.pose.position.x, p.pose.position.y));
}

/**********************************************************************
  Function which visualizes a given trajectory in RVIZ as a LINE_STRIP
  visualization_msg. Currently uses an external header library called
  tinycolormap to get the color associated with a normalized velocity.

  Inputs:
    - No inputs, works on the sentTraj field.

  TODOS:
    - Break out hard coded velocity normalization constant into a
      ros param?
    - Move this to utils.h?
***********************************************************************/
void
PlannerROS::visualizeTraj ()
{

  if (sentTraj.points.size () > 0)
    {
      if (!_use_arclen)
        {
          visualization_msgs::Marker msg;
          msg.header.frame_id = _frame_str;
          msg.header.stamp = ros::Time::now ();
          msg.ns = "planTraj";
          msg.id = 80;
          msg.action = visualization_msgs::Marker::ADD;
          msg.type = visualization_msgs::Marker::LINE_STRIP;

          msg.scale.x = .1;
          msg.pose.orientation.w = 1;

          for (trajectory_msgs::JointTrajectoryPoint p : sentTraj.points)
            {
              Eigen::Vector3d vel_vec (p.velocities[0], p.velocities[1],
                                       p.velocities[2]);
              double vel = vel_vec.norm ();

              tinycolormap::Color color = tinycolormap::GetColor (
                  vel / 1.0, tinycolormap::ColormapType::Plasma);

              std_msgs::ColorRGBA color_msg;
              color_msg.r = color.r ();
              color_msg.g = color.g ();
              color_msg.b = color.b ();
              color_msg.a = 1.0;
              msg.colors.push_back (color_msg);

              geometry_msgs::Point point_msg;
              point_msg.x = p.positions[0];
              point_msg.y = p.positions[1];
              point_msg.z = p.positions[2];
              msg.points.push_back (point_msg);
            }

          trajVizPub.publish (msg);
        }
      else
        {
          ros::Time start = ros::Time::now ();
          std::vector<state> states = _planner.get_arclen_traj ();

          if (states.size () == 0)
            {
              ROS_WARN ("arc length reparameterization failed!");
              return;
            }

          std::vector<double> ss, xs, ys;
          ss.resize (states.size ());
          xs.resize (states.size ());
          ys.resize (states.size ());

          for (int i = 0; i < states.size (); ++i)
            {
              xs[i] = states[i].pos (0);
              ys[i] = states[i].pos (1);
              ss[i] = states[i].t;
            }

          // std::vector<spline_t> d0 = _planner.get_tube();

          tk::spline splineX (ss, xs, tk::spline::cspline);
          tk::spline splineY (ss, ys, tk::spline::cspline);

          // tk::spline splineD = d0[0].spline;
          // tk::spline splineD1 = d0[1].spline;

          visualization_msgs::Marker arclenmsg;
          arclenmsg.header.frame_id = _frame_str;
          arclenmsg.header.stamp = ros::Time::now ();
          arclenmsg.ns = "planTraj";
          arclenmsg.id = 80;
          arclenmsg.action = visualization_msgs::Marker::ADD;
          arclenmsg.type = visualization_msgs::Marker::LINE_STRIP;
          arclenmsg.scale.x = .1;
          arclenmsg.pose.orientation.w = 1;

          // visualization_msgs::Marker tubemsg = arclenmsg;
          // tubemsg.ns = "tube";
          // tubemsg.id = 87;
          // tubemsg.scale.x = .05;

          // visualization_msgs::Marker tubemsg1 = tubemsg;
          // tubemsg.ns = "tube1";
          // tubemsg.id = 88;

          for (double s = 0; s < ss.back (); s += .1)
            {

              // get point and tangent to curve
              double px = splineX (s);
              double py = splineY (s);

              double tx = splineX.deriv (1, s);
              double ty = splineY.deriv (1, s);

              std_msgs::ColorRGBA color_msg;
              color_msg.r = 0.0;
              color_msg.g = 1.0;
              color_msg.b = 0.0;
              color_msg.a = 1.0;
              arclenmsg.colors.push_back (color_msg);

              geometry_msgs::Point point_msg;
              point_msg.x = px;
              point_msg.y = py;
              point_msg.z = 0.0;
              arclenmsg.points.push_back (point_msg);

              // Eigen::Vector2d point(px, py);
              // Eigen::Vector2d normal(-ty, tx);
              // normal = normal / normal.norm();

              // double d = splineD(s);
              // double d1 = splineD1(s);

              // std::cout << "d is " << d << std::endl;
              // std::cout << (point + normal*d).transpose() << std::endl;

              // geometry_msgs::Point tube_pt;
              // tube_pt.x = point(0) + normal(0) * d;
              // tube_pt.y = point(1) + normal(1) * d;
              // tube_pt.z = 1.0;
              // tubemsg.points.push_back(tube_pt);

              // geometry_msgs::Point tube_pt1;
              // tube_pt1.x = point(0) - normal(0) * d1;
              // tube_pt1.y = point(1) - normal(1) * d1;
              // tube_pt1.z = 1.0;
              // tubemsg1.points.push_back(tube_pt1);

              // color_msg.b = 1.0;
              // tubemsg.colors.push_back(color_msg);
              // tubemsg1.colors.push_back(color_msg);
            }

          // visualization_msgs::MarkerArray ma;
          // ma.markers.push_back(tubemsg);
          // ma.markers.push_back(tubemsg1);

          // tubeVizPub.publish(ma);
          trajVizPub.publish (arclenmsg);
        }
    }
}

/**********************************************************************
  Function for the main control loop of the planner, calls the plan()
  method which takes care of overall planning logic.

  Inputs:
    - Just a timer event which controls the rate at which the control
      loop is run.
***********************************************************************/
void
PlannerROS::controlLoop (const ros::TimerEvent &)
{

  static int count = 0;

  if (!_is_init || !_is_goal_set || !_is_costmap_started)
    return;

  if ((_odom (1) - goal (1)) * (_odom (1) - goal (1))
          + (_odom (0) - goal (0)) * (_odom (0) - goal (0))
      < .2)
    return;

  /*************************************
  ************* UPDATE MAP *************
  **************************************/

  // global_costmap->resetLayers();
  global_costmap->updateMap ();

  /*************************************
  **************** PLAN ****************
  **************************************/

  if (_plan_once && _planned)
    return;

  if (!plan (count >= _failsafe_count))
    {
      count++;
      if (count >= _failsafe_count)
        _curr_horizon *= .9;
    }
  else
    {
      count = 0;
      _curr_horizon /= .9;
      if (_curr_horizon > _max_dist_horizon)
        _curr_horizon = _max_dist_horizon;
    }

  // if (count >= _failsafe_count)
  // {
  //     if (plan(true))
  //     {
  //         count = 0;
  //         _curr_horizon /= .9;
  //     }
  //     else
  //         _curr_horizon *= .9;
  // }
}

/**********************************************************************
  This function is run on a timer controlled by the _dt ros parameter,
  and only runs if both the _is_init and is_goal_set flags are set to
  true. The overall process is:
    1.  Find JPS path from a point along previous trajectory to goal.
    1a. If first iteration, plan starting from robot position instead.
    2.  Expand intersecting polytopes around JPS path.
    3.  Send polytope and other physical constraints into optimizer.
    4.  Stitch resulting trajectory to previous one and publish to
        external trajectory tracker.

    Inputs:
      - is_failsafe flag which dictates if robot will stop for next
        planning phase. This is set true when planning failed
        several times in a row.
***********************************************************************/
bool
PlannerROS::plan (bool is_failsafe)
{

  if (is_failsafe)
    {
      ROS_WARN ("*******************************");
      ROS_WARN ("*** FAILSAFE MODE  ENGAGED ****");
      ROS_WARN ("*******************************");
      // GRBModel m = _planner.get_solver().getModel();
      // m.computeIIS();
      // m.write("/home/nick/Desktop/model.ilp");
      // exit(0);
    }

  // ROS_INFO("Setting up initial and final conditions");
  Eigen::MatrixXd initialPVAJ (3, 4);
  Eigen::MatrixXd finalPVAJ (3, 4);

  finalPVAJ << Eigen::Vector3d (goal (0), goal (1), 0),
      Eigen::Vector3d::Zero (), Eigen::Vector3d::Zero (),
      Eigen::Vector3d::Zero ();

  ros::Time a = ros::Time::now ();
  double p_start_t = 0.;

  if (sentTraj.points.size () == 0 || _is_teleop || _is_goal_reset
      || (mpcHorizon.points.size () == 0 && _use_arclen))
    {
      initialPVAJ << Eigen::Vector3d (_odom (0), _odom (1), 0),
          Eigen::Vector3d::Zero (), Eigen::Vector3d::Zero (),
          Eigen::Vector3d::Zero ();
    }
  else if (mpcHorizon.points.size () == 0
           && !_use_arclen) // use trajectory for init point if no horizon
    {

      double t = (a - start).toSec () + _lookahead;
      ROS_INFO ("using sentTraj for initial point t: %.4f", t);

      int trajInd
          = std::min ((int)(t / _traj_dt), (int)sentTraj.points.size () - 1);
      ROS_INFO ("got traj index %d", trajInd);

      trajectory_msgs::JointTrajectoryPoint p = sentTraj.points[trajInd];

      p_start_t = p.time_from_start.toSec ();

      geometry_msgs::PointStamped poseMsg;
      poseMsg.header.frame_id = _frame_str;
      poseMsg.header.stamp = ros::Time::now ();
      poseMsg.point.x = p.positions[0];
      poseMsg.point.y = p.positions[1];
      poseMsg.point.z = p.positions[2];
      initPointPub.publish (poseMsg);

      // position
      initialPVAJ.col (0)
          = Eigen::Vector3d (p.positions[0], p.positions[1], p.positions[2]);

      // velocity
      initialPVAJ.col (1) = Eigen::Vector3d (p.velocities[0], p.velocities[1],
                                             p.velocities[2]);

      // acceleration
      initialPVAJ.col (2) = Eigen::Vector3d (
          p.accelerations[0], p.accelerations[1], p.accelerations[2]);

      // jerk (stored in effort)
      initialPVAJ.col (3)
          = Eigen::Vector3d (p.effort[0], p.effort[1], p.effort[2]);
      ROS_INFO ("done finding initial point");
    }
  else if (mpcHorizon.points.size () > 0)
    {

      // double t = (a - start).toSec() + _lookahead;

      // ROS_INFO("t is %.4f", t);
      double t = _lookahead;

      int trajInd
          = std::min ((int)(t / _mpc_dt), (int)mpcHorizon.points.size () - 1);

      trajectory_msgs::JointTrajectoryPoint p = mpcHorizon.points[trajInd];

      p_start_t = p.time_from_start.toSec ();

      geometry_msgs::PointStamped poseMsg;
      poseMsg.header.frame_id = _frame_str;
      poseMsg.header.stamp = ros::Time::now ();
      poseMsg.point.x = p.positions[0];
      poseMsg.point.y = p.positions[1];
      poseMsg.point.z = p.positions[2];
      initPointPub.publish (poseMsg);

      // position
      initialPVAJ.col (0)
          = Eigen::Vector3d (p.positions[0], p.positions[1], p.positions[2]);

      // velocity
      initialPVAJ.col (1) = Eigen::Vector3d (p.velocities[0], p.velocities[1],
                                             p.velocities[2]);

      // acceleration
      initialPVAJ.col (2) = Eigen::Vector3d (
          p.accelerations[0], p.accelerations[1], p.accelerations[2]);

      // jerk (stored in effort)
      initialPVAJ.col (3)
          = Eigen::Vector3d (p.effort[0], p.effort[1], p.effort[2]);
    }
  else
    {
      ROS_ERROR ("Could not get initial position");
      return false;
    }

  if (is_failsafe)
    {
      initialPVAJ.col (1) = Eigen::Vector3d::Zero ();
      initialPVAJ.col (2) = Eigen::Vector3d::Zero ();
      initialPVAJ.col (3) = Eigen::Vector3d::Zero ();
    }

  /*************************************
  **************** PLAN ****************
  **************************************/

  map_util::occupancy_grid_t occ_grid
      = map_util::costmap_to_occgrid (*global_costmap->getCostmap ());

  _planner.set_start (initialPVAJ);
  _planner.set_goal (finalPVAJ);
  _planner.set_costmap (occ_grid);

  std::vector<Eigen::Vector2d> jpsPath;

  if (_jps_hysteresis && !is_failsafe)
    {
      ROS_INFO ("setting jps path as prev path with size %lu",
                _prev_jps_path.size ());
      jpsPath = _prev_jps_path;
    }

  // if (is_failsafe)
  // {
  //     ROS_INFO_STREAM("initialPVAJ:\n" << initialPVAJ);
  //     ROS_INFO_STREAM("odom:\n" << _odom.tranpose());
  // }

  std::vector<Eigen::MatrixX4d> hPolys;

  bool success = _planner.plan (_curr_horizon, jpsPath, hPolys);
  if (!success)
    {
      ROS_WARN ("Planner failed to find path");
      // check if initial velocity, acceleration, or jerk are higher than max
      double vel = initialPVAJ.col (1).norm ();
      double acc = initialPVAJ.col (2).norm ();
      double jerk = initialPVAJ.col (3).norm ();
      if (vel > _max_vel || acc > _max_acc || jerk > _max_jerk)
        {
          ROS_WARN ("initial conditions are too high: %.2f\t%.2f\t%.2f!", vel,
                    acc, jerk);
        }
    }

  // ROS_INFO("planning took %.4f", (ros::Time::now() - start).toSec());

  visualizePolytope (hPolys, meshPub, edgePub);
  // visualizeBoundary(_planner.get_corridor_boundary(), unionCorridorPub,
  // _frame_str);

  // ROS_INFO("visualized polytope %lu", hPolys.size());

  // publish initial pvaj
  trajectory_msgs::JointTrajectoryPoint p;
  p.positions = { initialPVAJ (0, 0), initialPVAJ (1, 0), initialPVAJ (2, 0) };
  p.velocities
      = { initialPVAJ (0, 1), initialPVAJ (1, 1), initialPVAJ (2, 1) };
  p.accelerations
      = { initialPVAJ (0, 2), initialPVAJ (1, 2), initialPVAJ (2, 2) };
  p.effort = { initialPVAJ (0, 3), initialPVAJ (1, 3), initialPVAJ (2, 3) };
  p.time_from_start = ros::Duration (p_start_t);
  initialPVAJPub.publish (p);

  /*************************************
  ******** PUBLISH JPS TO RVIZ *********
  **************************************/

  nav_msgs::Path jpsMsg;
  jpsMsg.header.stamp = ros::Time::now ();
  jpsMsg.header.frame_id = _frame_str;

  for (Eigen::Vector2d p : jpsPath)
    {
      geometry_msgs::PoseStamped pMsg;
      pMsg.header = jpsMsg.header;
      pMsg.pose.position.x = p (0);
      pMsg.pose.position.y = p (1);
      pMsg.pose.position.z = 0;
      pMsg.pose.orientation.w = 1;
      jpsMsg.poses.push_back (pMsg);
    }

  jpsPub.publish (jpsMsg);
  ROS_INFO ("published jps path: %lu", jpsPath.size ());

  if (!success)
    {
      mpcHorizon.points.clear ();
      return false;
    }

  _prev_jps_path = jpsPath;

  /*************************************
  ******** STITCH  TRAJECTORIES ********
  **************************************/

  std::vector<state> planned_trajectory = _planner.get_trajectory ();

  if (_use_arclen)
    {
      sentTraj.points.clear ();
      sentTraj.header.stamp = ros::Time::now ();
      sentTraj.header.frame_id = _frame_str;

      double t1 = std::round ((ros::Time::now () - a).toSec () * 10.) / 10.;
      double t2 = std::round (_lookahead * 10.) / 10.;
      int startInd
          = std::min ((int)(t1 / _mpc_dt), (int)mpcHorizon.points.size () - 1)
            + 1;
      int trajInd
          = std::min ((int)(t2 / _mpc_dt), (int)mpcHorizon.points.size () - 1);

      trajectory_msgs::JointTrajectory mpc_segment;
      for (int i = startInd; i < trajInd; ++i)
        {
          mpc_segment.points.push_back (mpcHorizon.points[i]);
        }

      std::vector<double> ss, xs, ys;
      bool reparam_status = reparam_traj (ss, xs, ys, mpc_segment);

      double s_offset = 0;
      // if (reparam_status)
      // {
      //     s_offset = ss.back();
      //     ROS_INFO("SOFFEST IS %.2f", s_offset);
      //     for (int i = 0; i < ss.size()-1; ++i)
      //     {
      //         trajectory_msgs::JointTrajectoryPoint p;
      //         p.positions.push_back(xs[i]);
      //         p.positions.push_back(ys[i]);
      //         p.time_from_start = ros::Duration(ss[i]);

      //         sentTraj.points.push_back(p);
      //     }
      // }

      std::vector<state> arclen_traj = _planner.get_arclen_traj ();
      ROS_INFO ("sending trajectory with length %.2f", arclen_traj.back ().t);

      for (state &x : arclen_traj)
        {
          // time from start is actually arc len in this case...
          trajectory_msgs::JointTrajectoryPoint p;
          p.positions.push_back (x.pos (0));
          p.positions.push_back (x.pos (1));
          p.time_from_start = ros::Duration (x.t + s_offset);

          sentTraj.points.push_back (p);
        }
    }
  else if (sentTraj.points.size () != 0)
    {

      // ROS_INFO("[%.2f] t1 is %.2f\tt2 is %.2f",
      // (ros::Time::now()-start).toSec(),t1, t2);

      int startInd, trajInd;
      trajectory_msgs::JointTrajectory aTraj, bTraj;
      bTraj = convertTrajToMsg (planned_trajectory, _traj_dt, _frame_str);

      if (mpcHorizon.points.size () == 0)
        {
          double t1
              = std::round ((ros::Time::now () - start).toSec () * 10.) / 10.;
          double t2
              = std::round (((a - start).toSec () + _lookahead) * 10.) / 10.;
          startInd = std::min ((int)(t1 / _traj_dt),
                               (int)sentTraj.points.size () - 1)
                     + 1;
          trajInd = std::min ((int)(t2 / _traj_dt),
                              (int)sentTraj.points.size () - 1);

          for (int i = startInd; i < trajInd; i++)
            {
              aTraj.points.push_back (sentTraj.points[i]);
              aTraj.points.back ().time_from_start
                  = ros::Duration ((i - startInd) * _traj_dt);
            }

          double startTime = (trajInd - startInd) * _traj_dt;

          for (int i = 0; i < bTraj.points.size (); i++)
            {
              aTraj.points.push_back (bTraj.points[i]);
              aTraj.points.back ().time_from_start
                  = ros::Duration (startTime + i * _traj_dt);
            }
        }
      else
        {
          double t1
              = std::round ((ros::Time::now () - a).toSec () * 10.) / 10.;
          double t2 = std::round (_lookahead * 10.) / 10.;
          startInd = std::min ((int)(t1 / _mpc_dt),
                               (int)mpcHorizon.points.size () - 1)
                     + 1;
          trajInd = std::min ((int)(t2 / _mpc_dt),
                              (int)mpcHorizon.points.size () - 1);

          for (int i = startInd; i < trajInd; i++)
            {
              aTraj.points.push_back (mpcHorizon.points[i]);
              aTraj.points.back ().time_from_start
                  = ros::Duration ((i - startInd) * _mpc_dt);
            }

          double startTime = (trajInd - startInd) * _mpc_dt;
          // double startTime = 0;

          for (int i = 0; i < bTraj.points.size (); i++)
            {
              aTraj.points.push_back (bTraj.points[i]);
              aTraj.points.back ().time_from_start
                  = ros::Duration (startTime + i * _traj_dt);
            }
        }

      // ROS_INFO("[%.2f] startInd is %d\ttrajInd is %d",
      // (ros::Time::now()-start).toSec(),startInd, trajInd);

      aTraj.header.frame_id = _frame_str;
      aTraj.header.stamp = ros::Time::now ();
      sentTraj = aTraj;
      start = ros::Time::now ();
      _is_goal_reset = false;
    }
  else
    {
      sentTraj = convertTrajToMsg (planned_trajectory, _traj_dt, _frame_str);
      start = ros::Time::now ();
    }

  if (!_is_teleop)
    trajPub.publish (sentTraj);

  visualizeTraj ();
  publishCPS ();

  _planned = true;
  mpcHorizon.points.clear ();

  double totalT = (ros::Time::now () - a).toSec ();
  // ROS_INFO("total time is %.4f", totalT);

  return true;
}

/**********************************************************************
  Function to publish current goal on a timer.

  Inputs:
    - Just a timer event which controls the rate at which the function
      is run.

  TODOS:
    - Decide if this function is necesary or not.
***********************************************************************/
void
PlannerROS::goalLoop (const ros::TimerEvent &)
{

  if (!_is_goal_set)
    return;

  geometry_msgs::PoseStamped msg;
  msg.header.stamp = ros::Time::now ();
  msg.header.frame_id = _frame_str;

  msg.pose.position.x = goal (0);
  msg.pose.position.y = goal (1);
  msg.pose.position.z = 0;
  msg.pose.orientation.w = 1;

  goalPub.publish (msg);
}

void
PlannerROS::publishCPS ()
{
  // publish control points as a posearray
  geometry_msgs::PoseArray msg;
  msg.header.frame_id = _frame_str;
  msg.header.stamp = ros::Time::now ();

  std::vector<Eigen::Vector3d> cps = _planner.get_cps ();

  for (Eigen::Vector3d cp : cps)
    {
      geometry_msgs::Pose p;
      p.position.x = cp (0);
      p.position.y = cp (1);
      p.position.z = cp (2);
      msg.poses.push_back (p);
    }

  ROS_INFO ("msg poses size is %lu", msg.poses.size ());
  ctrlPointPub.publish (msg);
}
