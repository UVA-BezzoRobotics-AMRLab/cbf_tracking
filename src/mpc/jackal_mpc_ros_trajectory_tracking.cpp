#include <math.h>
#include <algorithm>

#include <tf/tf.h>
#include <std_msgs/Bool.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PointStamped.h>
#include <std_msgs/Float32MultiArray.h>
#include <geometry_msgs/PolygonStamped.h>

#include <cbf_tracking/QuerySAC.h>

#include "mpc/jackal_mpc_acceleration.h"
#include "mpc/jackal_mpc_ros_trajectory_tracking.h"

#include <amrl_logging/LoggingData.h>
#include <amrl_logging/LoggingStart.h>
#include <amrl_logging/LoggingStop.h>
#include <amrl_logging/LoggingDropTable.h>
#include <amrl_logging/LoggingBufferCheck.h>

#include <amrl_logging_util/util.hpp>

#define JACKAL_WIDTH .3

JackalMPCROS::JackalMPCROS(ros::NodeHandle &nh)
{

	_nh = nh;

	_estop = false;
	_is_done = false;
	_is_init = false;
	_is_goal = false;
	_traj_reset = false;
	_is_colliding = false;
	_is_first_iter = true;

	_curr_vel = 0;
	velMsg.linear.x = 0;
	velMsg.angular.z = 0;

	_prev_rl_state = Eigen::VectorXd(9);
	_prev_rl_state << 0, 0, 0, 0, 0, 0, 0, 0, 0;

	_curr_rl_state = Eigen::VectorXd(9);
	_curr_rl_state << 0, 0, 0, 0, 0, 0, 0, 0, 0;

	_dist_grid_ptr =
		std::make_shared<distmap::DistanceMap>(distmap::DistanceMap::Dimension(5, 5),
											   1,
											   distmap::DistanceMap::Origin());

	double freq;
	std::string mpc_type_str;

	nh.param<std::string>("jackal_mpc_track/mpc_type", mpc_type_str, "track_acc_ipopt");

	if (mpc_type_str == "track_acc_ipopt")
		_mpc_type = MPC_TYPE_ACC;
	else if (mpc_type_str == "track_acc_nlopt")
		_mpc_type = MPC_TYPE_NLOPT;
	else if (mpc_type_str == "track_vel_ipopt")
		_mpc_type = MPC_TYPE_CTE;
	else if (mpc_type_str == "goto_pos_nlopt")
		_mpc_type = MPC_TYPE_NLOPT_POS;
	else
	{
		ROS_WARN("Invalid MPC type. Defaulting to track_acc_ipopt");
		_mpc_type = MPC_TYPE_ACC;
	}

	// Localization params
	nh.param("jackal_mpc_track/use_vicon", _use_vicon, false);

	// MPC params
	nh.param("jackal_mpc_track/vel_pub_freq", _vel_pub_freq, 20.0);
	nh.param("jackal_mpc_track/controller_frequency", freq, 10.0);
	nh.param("jackal_mpc_track/mpc_steps", _mpc_steps, 10.0);

	// Cost function params
	nh.param("jackal_mpc_track/w_vel", _w_vel, 1.0);
	nh.param("jackal_mpc_track/w_angvel", _w_angvel, 1.0);
	nh.param("jackal_mpc_track/w_linvel", _w_linvel, 1.0);
	nh.param("jackal_mpc_track/w_angvel_d", _w_angvel_d, 1.0);
	nh.param("jackal_mpc_track/w_linvel_d", _w_linvel_d, .5);
	nh.param("jackal_mpc_track/w_etheta", _w_etheta, 1.0);
	nh.param("jackal_mpc_track/w_cte", _w_cte, 1.0);
	nh.param("jackal_mpc_track/w_pos", _w_pos, 1.0);

	// pos_mpc cost function params
	nh.param("jackal_mpc_track/pos_mpc_w_pos", _pos_mpc_w_pos, 1.0);
	nh.param("jackal_mpc_track/pos_mpc_w_angvel", _pos_mpc_w_angvel, 1.0);
	nh.param("jackal_mpc_track/pos_mpc_w_vel", _pos_mpc_w_vel, 1.0);
	nh.param("jackal_mpc_track/pos_mpc_w_angvel_d", _pos_mpc_w_angvel_d, 1.0);
	nh.param("jackal_mpc_track/pos_mpc_w_linvel_d", _pos_mpc_w_linvel_d, .5);
	nh.param("jackal_mpc_track/pos_mpc_max_linvel", _pos_mpc_max_linvel, 2.0);
	nh.param("jackal_mpc_track/pos_mpc_max_angvel", _pos_mpc_max_angvel, 3.0);

	// Constraint params
	nh.param("jackal_mpc_track/w_max", _max_angvel, 3.0);
	nh.param("jackal_mpc_track/v_max", _max_linvel, 2.0);
	nh.param("jackal_mpc_track/a_max", _max_linacc, 3.0);
	nh.param("jackal_mpc_track/anga_max", _max_anga, 2 * M_PI);
	nh.param("jackal_mpc_track/bound_value", _bound_value, 1.0e19);

	// Goal params
	nh.param("jackal_mpc_track/x_goal", _x_goal, 0.0);
	nh.param("jackal_mpc_track/y_goal", _y_goal, 0.0);
	nh.param("jackal_mpc_track/goal_tolerance", _tol, 0.3);

	// Teleop params
	nh.param("jackal_mpc_track/teleop", _teleop, false);
	nh.param<std::string>("jackal_mpc_track/frame_id", _frame_id, "odom");

	// cbf params
	nh.param("jackal_mpc_track/use_cbf", _use_cbf, false);
	nh.param("jackal_mpc_track/cbf_alpha", _cbf_alpha, .5);
	nh.param("jackal_mpc_track/cbf_colinear", _cbf_colinear, .1);
	nh.param("jackal_mpc_track/cbf_padding", _cbf_padding, .1);
	nh.param("jackal_mpc_track/dynamic_alpha", _use_dynamic_alpha, false);

	// proportional controller params
	nh.param("jackal_mpc_track/prop_gain", _prop_gain, .5);
	nh.param("jackal_mpc_track/prop_angle_thresh", _prop_angle_thresh, 30. * M_PI / 180.);

	// learning params
	nh.param("/train/is_eval", _is_eval, false);
	nh.param("/train/logging", _logging, false);

	_logging = _logging && !_is_eval;

	if (_is_eval || _logging)
	{
		nh.param("/train/min_alpha", _min_alpha, 0.1);
		nh.param("/train/max_alpha", _max_alpha, 10.0);
	}

	_dt = 1.0 / freq;

	_mpc_params["DT"] = _dt;
	_mpc_params["STEPS"] = _mpc_steps;
	_mpc_params["W_V"] = _w_linvel;
	_mpc_params["W_ANGVEL"] = _w_angvel;
	_mpc_params["W_DA"] = _w_linvel_d;
	_mpc_params["W_DANGVEL"] = _w_angvel_d;
	_mpc_params["W_ETHETA"] = _w_etheta;
	_mpc_params["W_POS"] = _w_pos;
	_mpc_params["W_CTE"] = _w_cte;
	_mpc_params["LINVEL"] = _max_linvel;
	_mpc_params["ANGVEL"] = _max_angvel;
	_mpc_params["BOUND"] = _bound_value;
	_mpc_params["X_GOAL"] = _x_goal;
	_mpc_params["Y_GOAL"] = _y_goal;

	_mpc_params["MAX_ANGA"] = _max_anga;
	_mpc_params["MAX_LINACC"] = _max_linacc;

	_mpc_params["USE_CBF"] = _use_cbf;
	_mpc_params["CBF_ALPHA"] = _cbf_alpha;
	_mpc_params["CBF_COLINEAR"] = _cbf_colinear;
	_mpc_params["CBF_PADDING"] = _cbf_padding;
	_mpc_params["CBF_DYNAMIC_ALPHA"] = _use_dynamic_alpha;
	_mpc_params["DEBUG"] = true;

	// _mpc_params["LOGGING"] = _logging;

	_pos_mpc_params["DT"] = _dt;
	_pos_mpc_params["STEPS"] = _mpc_steps;
	_pos_mpc_params["W_V"] = _pos_mpc_w_vel;
	_pos_mpc_params["W_ANGVEL"] = _pos_mpc_w_angvel;
	_pos_mpc_params["W_DA"] = _pos_mpc_w_linvel_d;
	_pos_mpc_params["W_DANGVEL"] = _pos_mpc_w_angvel_d;
	_pos_mpc_params["LINVEL"] = _pos_mpc_max_linvel;
	_pos_mpc_params["ANGVEL"] = _pos_mpc_max_angvel;
	_pos_mpc_params["BOUND"] = _bound_value;
	_pos_mpc_params["X_GOAL"] = _x_goal;
	_pos_mpc_params["Y_GOAL"] = _y_goal;

	_mpc_core = std::make_shared<JackalMPCCore>(_mpc_type);
	ROS_INFO("loading mpc params");
	_mpc_core->load_params(_mpc_params);
	ROS_INFO("done loading params!");

	if (_use_vicon)
		_odomSub = nh.subscribe("/vicon/jackal4/jackal4", 1, &JackalMPCROS::viconcb, this);
	else
		_odomSub = nh.subscribe("/odometry/filtered", 1, &JackalMPCROS::odomcb, this);

	_collisionSub = nh.subscribe("/collision", 1, &JackalMPCROS::collisioncb, this);
	_goalSub = nh.subscribe("recoveryGoal", 1, &JackalMPCROS::goalcb, this);
	_polySub = nh.subscribe("/recoveryPoly", 1, &JackalMPCROS::polycb, this);
	_alphaSub = nh.subscribe("/alpha", 1, &JackalMPCROS::alphacb, this);
	_obsSub = nh.subscribe("/curr_obstacle", 1, &JackalMPCROS::obstaclecb, this);
	_trajSub = nh.subscribe("/reference_trajectory", 1, &JackalMPCROS::trajectorycb, this);
	_distMapSub = nh.subscribe("/distance_map_node/distance_field_obstacles", 1, &JackalMPCROS::distmapcb, this);
	_trajNoResetSub = nh.subscribe("/reference_trajectory_no_reset", 1, &JackalMPCROS::trajectoryNoResetcb, this);

	_timer = nh.createTimer(ros::Duration(_dt), &JackalMPCROS::controlLoop, this);
	// _velPubTimer = nh.createTimer(ros::Duration(1./_vel_pub_freq), &JackalMPCROS::publishVel, this);

	_donePub = nh.advertise<std_msgs::Bool>("/mpc_done", 0);
	_dist_pub = nh.advertise<std_msgs::Float64>("/mpc_dist", 0);
	_pathPub = nh.advertise<nav_msgs::Path>("/spline_path", 10);
	_velPub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
	_h_value_pub = nh.advertise<std_msgs::Float64>("/h_value", 0);
	_trajPub = nh.advertise<nav_msgs::Path>("/mpc_prediction", 10);
	_alpha_pub = nh.advertise<std_msgs::Float64>("/alpha_value", 0);
	_actualPathPub = nh.advertise<nav_msgs::Path>("/actual_path", 0);
	_solveTimePub = nh.advertise<std_msgs::Float64>("/mpc_solve_time", 0);
	_pointPub = nh.advertise<geometry_msgs::PointStamped>("traj_point", 0);
	_goalReachedPub = nh.advertise<std_msgs::Bool>("/mpc_goal_reached", 10);
	_odomPub = nh.advertise<visualization_msgs::Marker>("robot_position", 10);
	_polyPub = nh.advertise<geometry_msgs::PolygonStamped>("/convex_free", 0);
	_polyPub2 = nh.advertise<geometry_msgs::PolygonStamped>("/convex_free2", 0);
	_horizonPub = nh.advertise<trajectory_msgs::JointTrajectory>("/mpc_horizon", 0);
	_refPub = nh.advertise<trajectory_msgs::JointTrajectoryPoint>("/current_reference", 10);

	if (_logging)
	{
		_logging_table_name = "replay_buffer";
		_logging_topic_name = "/cbf_rl_learning";
		const std::vector<std::string> string_type_names({"is_done"});
		const std::vector<std::string> float_type_names(
			{"id",
			 "prev_theta",
			 "prev_vel",
			 "prev_acc",
			 "prev_angvel",
			 "prev_obs_dist",
			 "prev_obs_heading",
			 "prev_progress",
			 "prev_h",
			 "prev_alpha",
			 "action",
			 "reward",
			 "curr_theta",
			 "curr_vel",
			 "curr_acc",
			 "curr_angvel",
			 "curr_obs_dist",
			 "curr_obs_heading",
			 "curr_progress",
			 "curr_h",
			 "curr_alpha"});

		_loggingPub = nh.advertise<amrl_logging::LoggingData>(_logging_topic_name, 100);

		_sac_srv = nh.serviceClient<cbf_tracking::QuerySAC>("/query_sac");

		if (!amrl::logging_setup(nh, _logging_table_name, _logging_topic_name, string_type_names, {}, float_type_names))
		{
			ROS_ERROR("Could not setup database logging");
			exit(0);
		}
	}

	if (_is_eval)
		_sac_srv = nh.serviceClient<cbf_tracking::QuerySAC>("/query_sac");

	_eStop_srv = nh.advertiseService("/eStop", &JackalMPCROS::eStopcb, this);
	_mode_srv = nh.advertiseService("/switch_mode", &JackalMPCROS::mode_switchcb, this);

	timer_thread = std::thread(&JackalMPCROS::publishVel, this);
}

JackalMPCROS::~JackalMPCROS()
{
	if (timer_thread.joinable())
		timer_thread.join();

	if (_logging)
		amrl::logging_finish(_nh, _logging_table_name);
	// delete _mpc;
}

bool JackalMPCROS::eStopcb(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
	ROS_WARN("E-STOP RECEIVED");

	// geometry_msgs::Twist velMsg;
	velMsg.linear.x = 0;
	velMsg.angular.z = 0;

	trajectory.points.clear();

	_estop ^= true;

	return true;
}

bool JackalMPCROS::mode_switchcb(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
	ROS_WARN("SWITCHING MODE");

	_mpc_type = (_mpc_type == MPC_TYPE_CTE) ? MPC_TYPE_ACC : MPC_TYPE_CTE;

	if (_mpc_type == MPC_TYPE_CTE)
		ROS_WARN("SWITCHED TO VEL-CTE MODE");
	else
		ROS_WARN("SWITCHED TO ACC-CTE MODE");

	// geometry_msgs::Twist velMsg;
	velMsg.linear.x = 0;
	velMsg.angular.z = 0;

	_is_at_goal = false;
	_is_goal = false;

	return true;
}

void JackalMPCROS::collisioncb(const std_msgs::Bool::ConstPtr &msg)
{
	_is_colliding = msg->data;
}

void JackalMPCROS::alphacb(const std_msgs::Float64::ConstPtr &msg)
{
	_mpc_params["CBF_ALPHA"] = msg->data;
	_mpc_core->load_params(_mpc_params);
}

void JackalMPCROS::distmapcb(const distance_map_msgs::DistanceMap::ConstPtr &msg)
{
	if (msg == nullptr)
	{
		ROS_WARN("Distance map is null");
		return;
	}

	*_dist_grid_ptr = utils::distmap_from_msg(*msg);
	_mpc_core->set_dist_map(_dist_grid_ptr);
}

void JackalMPCROS::obstaclecb(const geometry_msgs::Point::ConstPtr &msg)
{
	_obstacle = Eigen::Vector3d(msg->x, msg->y, msg->z);
	_mpc_core->set_obstacle(_obstacle);
}

void JackalMPCROS::polycb(const geometry_msgs::PoseArray::ConstPtr &msg)
{

	Eigen::MatrixX4d currPoly;
	for (int i = 0; i < msg->poses.size(); ++i)
	{
		geometry_msgs::Pose p = msg->poses[i];
		if (p.orientation.x == 0 && p.orientation.y == 0 && p.orientation.z == 0 && p.orientation.w == 0)
		{
			if (currPoly.rows() > 0)
			{
				_poly = currPoly;
				break;
			}
		}
		else
		{
			Eigen::Vector4d plane(p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w);
			currPoly.conservativeResize(currPoly.rows() + 1, 4);
			currPoly.row(currPoly.rows() - 1) = plane;
		}
	}
}

void JackalMPCROS::publishVel()
{
	constexpr double pub_vel_loop_rate_hz = 50;
	const std::chrono::milliseconds pub_loop_period(static_cast<int>(1000.0 / pub_vel_loop_rate_hz));

	while (ros::ok())
	{
		if (trajectory.points.size() > 0)
		{
			_refPub.publish(current_reference);
		}

		// ROS_INFO("PUBLISHING {vel = %.2f, ang_z = %.2f}", velMsg.linear.x, velMsg.angular.z);
		
		velMsg.angular.z = std::max(-_max_angvel/2, std::min(_max_angvel/2, velMsg.angular.z));
		_velPub.publish(velMsg);
		std::this_thread::sleep_for(pub_loop_period);
	}
}

void JackalMPCROS::goalcb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
	_x_goal = msg->pose.position.x;
	_y_goal = msg->pose.position.y;

	_is_goal = true;
	_is_at_goal = false;

	_mpc_core->set_goal(Eigen::Vector2d(_x_goal, _y_goal));

	ROS_WARN("GOAL RECEIVED (%.2f, %.2f)", _x_goal, _y_goal);
}

void JackalMPCROS::viconcb(const geometry_msgs::TransformStamped::ConstPtr &msg)
{

	tf::Quaternion q(
		msg->transform.rotation.x,
		msg->transform.rotation.y,
		msg->transform.rotation.z,
		msg->transform.rotation.w);

	tf::Matrix3x3 m(q);
	double roll, pitch, yaw;
	m.getRPY(roll, pitch, yaw);

	_odom = Eigen::VectorXd(3);

	_odom(XI) = msg->transform.translation.x;
	_odom(YI) = msg->transform.translation.y;
	_odom(THETAI) = yaw;

	_is_init = true;

	visualization_msgs::Marker marker;
	marker.header.frame_id = _frame_id;
	marker.header.stamp = ros::Time();
	marker.ns = "position";
	marker.id = 0;
	marker.type = visualization_msgs::Marker::ARROW;
	marker.action = visualization_msgs::Marker::ADD;
	marker.pose.position.x = _odom(XI);
	marker.pose.position.y = _odom(YI);
	marker.pose.position.z = 0;
	marker.pose.orientation.x = msg->transform.rotation.x;
	marker.pose.orientation.y = msg->transform.rotation.y;
	marker.pose.orientation.z = msg->transform.rotation.z;
	marker.pose.orientation.w = msg->transform.rotation.w;
	marker.scale.x = 0.4;
	marker.scale.y = 0.2;
	marker.scale.z = 0.4;
	marker.color.r = 1.0;
	marker.color.g = 1.0;
	marker.color.b = 1.0;
	marker.color.a = 1.0;
	_odomPub.publish(marker);
}

void JackalMPCROS::odomcb(const nav_msgs::Odometry::ConstPtr &msg)
{

	tf::Quaternion q(
		msg->pose.pose.orientation.x,
		msg->pose.pose.orientation.y,
		msg->pose.pose.orientation.z,
		msg->pose.pose.orientation.w);

	tf::Matrix3x3 m(q);
	double roll, pitch, yaw;
	m.getRPY(roll, pitch, yaw);

	_odom = Eigen::VectorXd(3);

	_odom(XI) = msg->pose.pose.position.x;
	_odom(YI) = msg->pose.pose.position.y;
	_odom(THETAI) = yaw;

	_mpc_core->set_odom(_odom);

	if (!_is_init)
	{
		_is_init = true;
		ROS_INFO("tracker initialized");
	}
}

// TODO: Support appending trajectories
void JackalMPCROS::trajectorycb(const trajectory_msgs::JointTrajectory::ConstPtr &msg)
{
	trajectory = *msg;
	_traj_reset = true;

	std::vector<traj_point_t> traj_points;
	for (const trajectory_msgs::JointTrajectoryPoint &pt : trajectory.points)
	{
		traj_point_t traj_pt;
		traj_pt.pose = Eigen::Vector3d(pt.positions[0], pt.positions[1], pt.positions[2]);
		traj_pt.velocity = Eigen::Vector3d(pt.velocities[0], pt.velocities[1], pt.velocities[2]);
		traj_pt.acceleration = Eigen::Vector3d(pt.accelerations[0], pt.accelerations[1], pt.accelerations[2]);
		traj_pt.time_from_start = pt.time_from_start.toSec();

		traj_points.push_back(traj_pt);
	}

	_mpc_core->set_trajectory(traj_points);

	ROS_INFO("**********************************************************");
	ROS_INFO("MPC received trajectory!");
	ROS_INFO("**********************************************************");
}

void JackalMPCROS::trajectoryNoResetcb(const trajectory_msgs::JointTrajectory::ConstPtr &msg)
{
	trajectory = *msg;

	std::vector<traj_point_t> traj_points;
	for (const trajectory_msgs::JointTrajectoryPoint &pt : trajectory.points)
	{
		traj_point_t traj_pt;
		traj_pt.pose = Eigen::Vector3d(pt.positions[0], pt.positions[1], pt.positions[2]);
		traj_pt.velocity = Eigen::Vector3d(pt.velocities[0], pt.velocities[1], pt.velocities[2]);
		traj_pt.acceleration = Eigen::Vector3d(pt.accelerations[0], pt.accelerations[1], pt.accelerations[2]);
		traj_pt.time_from_start = pt.time_from_start.toSec();

		traj_points.push_back(traj_pt);
	}

	_mpc_core->set_trajectory(traj_points);

	ROS_INFO("MPC received trajectory (no time reset)!");
}

void JackalMPCROS::cte_ctrl_loop()
{
	static ros::Time start;
	static int row_id = 0;

	if (!_is_init || _estop)
		return;

	if (_traj_reset)
	{
		start = ros::Time::now();
		_traj_reset = false;
	}

	double alpha_dot = 0.;

	if (trajectory.points.size() != 0)
	{
		ros::Time now = ros::Time::now();
		double t = (now - start).toSec();
		current_reference = evalTraj(t);

		double traj_duration = trajectory.points.back().time_from_start.toSec();
		Eigen::Vector2d goal(trajectory.points.back().positions[0],
							 trajectory.points.back().positions[1]);

		// If trajectory is done, stop
		if (t > traj_duration)
		{
			ROS_INFO("trajectory done!");
			velMsg.linear.x = 0;
			velMsg.angular.z = 0;
			trajectory.points.clear();
			return;
		}

		bool exceeded_bounds = false;
		ROS_INFO("_is_eval is %d", _is_eval);
		if (_mpc_type == MPC_TYPE_NLOPT && (_logging || _is_eval))
		{
			cbf_tracking::QuerySAC req;
			Eigen::VectorXd state = _mpc_core->get_state();
			req.request.theta = state(0);
			req.request.vel = state(1);
			req.request.acc = state(2);
			req.request.ang_vel = state(3);
			req.request.obs_dist = state(4);
			req.request.heading_dist = state(5);
			req.request.progress = state(6);
			req.request.h_val = state(7);
			req.request.alpha = _mpc_params["CBF_ALPHA"];

			ROS_INFO("calling SAC service");
			// try calling _sac service
			if (_sac_srv.call(req))
			{
				if (!req.response.success)
					ROS_ERROR("SAC service failed");
				// integrate alpha_dot into CBF_ALPHA
				// clip alpha to ensure it's within bounds
				alpha_dot = req.response.alpha_dot;
				double alpha = _mpc_params["CBF_ALPHA"] + alpha_dot * _dt;

				if (alpha < _min_alpha || alpha > _max_alpha)
					exceeded_bounds = true;

				alpha = std::max(_min_alpha, std::min(_max_alpha, alpha));

				_mpc_params["CBF_ALPHA"] = alpha;
				_mpc_core->load_params(_mpc_params);
			}
			else
			{
				ROS_ERROR("Failed to call service query_sac");
				return;
			}
		}

		// Send next _mpc_steps reference points to solver
		// For feasible tracking, trajectory MUST be C2-continuous due to
		// trajectory angular velocity calculations in MPC
		Eigen::MatrixXd wpts(6, (int)_mpc_steps);
		for (int i = 0; i < _mpc_steps; ++i)
		{
			trajectory_msgs::JointTrajectoryPoint pt;

			if (i * _dt + t > traj_duration)
				pt = trajectory.points.back();
			else
				pt = evalTraj(i * _dt + t);

			// posX, velX, accX
			wpts(0, i) = pt.positions[0];
			wpts(1, i) = i * _dt + t >= traj_duration ? 0 : pt.velocities[0];
			wpts(2, i) = i * _dt + t >= traj_duration ? 0 : pt.accelerations[0];

			// posY, velY, accY
			wpts(3, i) = pt.positions[1];
			wpts(4, i) = i * _dt + t >= traj_duration ? 0 : pt.velocities[1];
			wpts(5, i) = i * _dt + t >= traj_duration ? 0 : pt.accelerations[1];
		}

		_mpc_core->set_reference(wpts);

		ros::Time start = ros::Time::now();
		mpc_results = _mpc_core->solve();
		double solve_time = (ros::Time::now() - start).toSec();

		std::vector<double> h_vals = _mpc_core->evaluateHFunction();

		// publish solve time and associated odom
		std_msgs::Float64 solveTimeMsg;
		solveTimeMsg.data = solve_time;
		_solveTimePub.publish(solveTimeMsg);

		ROS_INFO("{vel = %.2f, ang_z = %.2f}", mpc_results[0], mpc_results[1]);
		velMsg.angular.z = mpc_results[1];
		velMsg.linear.x = mpc_results[0];

		publishReference();
		publishMPCTrajectory();
		publishActualPath();

		if (_mpc_type == MPC_TYPE_NLOPT)
		{
			if (_is_first_iter && _logging)
			{
				_curr_rl_state = _mpc_core->get_state();
				_prev_rl_state = _curr_rl_state;
			}
			// we don't want to log if already reported an is_done state
			else if (_logging && !_is_done)
			{
				double reward = -100;

				_curr_rl_state = _mpc_core->get_state();

				if (!_is_colliding)
				{
					// weight distance to obstacle
					reward = 5 * _curr_rl_state(4);
				}
				else
				{
					_is_done = true;
				}

				// add penalty for not making progress
				reward -= 12 * (1 - _curr_rl_state(6));

				// add small penalty for large alpha jumps
				reward -= 0.1 * alpha_dot * alpha_dot;

				// add penalty for using higher alpha values
				// reward -= .1 * (_curr_rl_state(8)- _min_alpha);

				// if alpha value is outside bounds, penalize heavily
				if (exceeded_bounds)
					reward -= 20;

				// if h_value constraint is negative, penalize heavily
				if (h_vals[0] < 0)
					reward -= 10;

				// log to database
				amrl_logging::LoggingData row;
				std::string is_done_str = _is_done ? "true" : "false";
				std::vector<std::string> string_data = {is_done_str};
				std::vector<double> numeric_data = {
					row_id,
					_prev_rl_state(0), // theta
					_prev_rl_state(1), // velocity
					_prev_rl_state(2), // acceleration
					_prev_rl_state(3), // angular velocity
					_prev_rl_state(4), // distance to obstacle
					_prev_rl_state(5), // heading to obstacle
					_prev_rl_state(6), // progress
					_prev_rl_state(7), // h value
					_prev_rl_state(8), // alpha value
					alpha_dot,
					reward,
					_curr_rl_state(0),	// theta
					_curr_rl_state(1),	// velocity
					_curr_rl_state(2),	// acceleration
					_curr_rl_state(3),	// angular velocity
					_curr_rl_state(4),	// distance to obstacle
					_curr_rl_state(5),	// heading to obstacle
					_curr_rl_state(6),	// progress
					_curr_rl_state(7),	// h value
					_curr_rl_state(8)}; // alpha value

				row.header.seq += row_id++;
				row.header.stamp = ros::Time::now();
				row.labels = string_data;
				row.reals = numeric_data;

				_loggingPub.publish(row);

				_prev_rl_state = _curr_rl_state;
			}

			if (_use_cbf)
			{
				std_msgs::Float64 h_msg;
				h_msg.data = h_vals[0];
				_h_value_pub.publish(h_msg);

				std_msgs::Float64 alpha_msg;
				alpha_msg.data = h_vals[1];
				_alpha_pub.publish(alpha_msg);

				std_msgs::Float64 dist_msg;
				dist_msg.data = _dist_grid_ptr->atPositionSafe(_odom(0), _odom(1), true);
				_dist_pub.publish(dist_msg);
			}
		}

		if (_is_done || _is_colliding)
		{
			_is_done = true;
			std_msgs::Bool doneMsg;
			doneMsg.data = true;
			_donePub.publish(doneMsg);
		}

		// publish reference point
		geometry_msgs::PointStamped pointMsg;
		pointMsg.header.stamp = ros::Time::now();
		pointMsg.header.frame_id = _frame_id;
		pointMsg.point.x = wpts(0, 0);
		pointMsg.point.y = wpts(3, 0);
		_pointPub.publish(pointMsg);

		_is_first_iter = false;
	}
}

void JackalMPCROS::controlLoop(const ros::TimerEvent &)
{
	// don't care about aligning if trajectory is short enough (under 10 steps ~ 1 sec)
	if (trajectory.points.size() > 10 && _traj_reset)
	{

		// calculate heading error between robot and trajectory start
		// use 1st point as most times first point has 0 velocity
		double traj_heading = atan2(trajectory.points[9].velocities[1],
									trajectory.points[9].velocities[0]);

		// wrap between -pi and pi
		double e = atan2(sin(traj_heading - _odom(THETAI)), cos(traj_heading - _odom(THETAI)));

		ROS_INFO("Robot heading is %.2f and traj_heading is %.2f", _odom(THETAI), traj_heading);
		ROS_WARN("trajectory reset, checking if we need to align... error = %.2f deg", e * 180. / M_PI);

		// if error is larger than _prop_angle_thresh use proportional controller to align
		if (fabs(e) > _prop_angle_thresh)
		{
			ROS_WARN("Alignment to trajectory now!");
			trajectory_msgs::JointTrajectory traj;
			traj.header.stamp = ros::Time::now();
			traj.header.frame_id = _frame_id;
			for (int i = 0; i < _mpc_steps; ++i)
			{
				trajectory_msgs::JointTrajectoryPoint pt;
				pt.positions = {_odom(XI), _odom(YI), 0};
				pt.velocities = {0, 0, 0};
				pt.accelerations = {0, 0, 0};
				pt.effort = {0, 0, 0};
				pt.time_from_start = ros::Duration(i * _dt);
				traj.points.push_back(pt);
			}

			_horizonPub.publish(traj);
			publishReference();

			velMsg.linear.x = 0;
			velMsg.angular.z = _prop_gain * e;
			return;
		}
	}

	cte_ctrl_loop();
}

trajectory_msgs::JointTrajectoryPoint JackalMPCROS::evalTraj(double t)
{

	for (trajectory_msgs::JointTrajectoryPoint pt : trajectory.points)
	{
		if (t < pt.time_from_start.toSec())
			return pt;
	}

	return trajectory.points.back();
}

void JackalMPCROS::publishReference()
{

	nav_msgs::Path msg;
	msg.header.stamp = ros::Time::now();
	msg.header.frame_id = _frame_id;

	bool published = false;
	for (trajectory_msgs::JointTrajectoryPoint pt : trajectory.points)
	{
		if (!published)
		{
			published = true;
			_refPub.publish(pt);
		}
		geometry_msgs::PoseStamped pose;
		pose.header.stamp = ros::Time::now();
		pose.header.frame_id = _frame_id;

		pose.pose.position.x = pt.positions[0];
		pose.pose.position.y = pt.positions[1];
		pose.pose.position.z = 0;
		pose.pose.orientation.x = 0;
		pose.pose.orientation.y = 0;
		pose.pose.orientation.z = 0;
		pose.pose.orientation.w = 1;
		msg.poses.push_back(pose);
	}

	_pathPub.publish(msg);
}

void JackalMPCROS::publishActualPath()
{

	// for visualizing actual path
	poses.push_back(_odom);

	nav_msgs::Path msg;
	msg.header.stamp = ros::Time::now();
	msg.header.frame_id = _frame_id;
	for (Eigen::Vector3d p : poses)
	{
		geometry_msgs::PoseStamped pose;
		pose.pose.position.x = p[0];
		pose.pose.position.y = p[1];
		pose.pose.position.z = 0;

		pose.pose.orientation.x = 0;
		pose.pose.orientation.y = 0;
		pose.pose.orientation.z = 0;
		pose.pose.orientation.w = 1;
		msg.poses.push_back(pose);
	}

	_actualPathPub.publish(msg);
}

void JackalMPCROS::publishMPCTrajectory()
{

	geometry_msgs::PoseStamped goal;
	goal.header.stamp = ros::Time::now();
	goal.header.frame_id = _frame_id;
	goal.pose.position.x = _x_goal;
	goal.pose.position.y = _y_goal;
	goal.pose.orientation.w = 1;

	nav_msgs::Path pathMsg;
	pathMsg.header.frame_id = _frame_id;
	pathMsg.header.stamp = ros::Time::now();

	std::vector<Eigen::VectorXd> horizon = _mpc_core->getHorizon();
	for (int i = 0; i < horizon.size(); ++i)
	{
		Eigen::VectorXd state = horizon[i];
		geometry_msgs::PoseStamped tmp;
		tmp.header = pathMsg.header;
		if (state.size() == 6)
		{
			tmp.pose.position.x = state(1);
			tmp.pose.position.y = state(2);
		}
		else
		{
			tmp.pose.position.x = state(0);
			tmp.pose.position.y = state(1);
		}
		tmp.pose.position.z = .1;
		tmp.pose.orientation.w = 1;
		pathMsg.poses.push_back(tmp);
	}

	_trajPub.publish(pathMsg);

	if (horizon.size() > 1 && horizon[0].size() == 6)
	{
		// convert to JointTrajectory
		trajectory_msgs::JointTrajectory traj;
		traj.header.stamp = ros::Time::now();
		traj.header.frame_id = _frame_id;

		double dt = horizon[1](0) - horizon[0](0);

		for (int i = 0; i < horizon.size(); ++i)
		{
			Eigen::VectorXd state = horizon[i];

			double t = state(0);
			double x = state(1);
			double y = state(2);
			double theta = state(3);
			double linvel = state(4);
			double linacc = state(5);

			// compute velocity and acceleration in x and y directions
			double vel_x = linvel * cos(theta);
			double vel_y = linvel * sin(theta);

			double acc_x = linacc * cos(theta);
			double acc_y = linacc * sin(theta);

			// compute jerk in x and y directions from acceleration
			double jerk_x = 0;
			double jerk_y = 0;
			if (i < horizon.size() - 1)
			{
				double next_linacc = horizon[i + 1](5);
				double next_linacc_x = next_linacc * cos(horizon[i + 1](3));
				double next_linacc_y = next_linacc * sin(horizon[i + 1](3));
				jerk_x = (next_linacc_x - acc_x) / dt;
				jerk_y = (next_linacc_y - acc_y) / dt;

				// ROS_INFO("jerk_x = %.2f, jerk_y = %.2f", jerk_x, jerk_y);
			}
			else
			{
				jerk_x = 0;
				jerk_y = 0;

				// ROS_INFO("(in else cond) jerk_x = 0, jerk_y = 0");
			}

			trajectory_msgs::JointTrajectoryPoint pt;
			pt.time_from_start = ros::Duration(t);
			pt.positions = {x, y, 0};
			pt.velocities = {vel_x, vel_y, 0};
			pt.accelerations = {acc_x, acc_y, 0};
			pt.effort = {jerk_x, jerk_y, 0};

			traj.points.push_back(pt);
		}

		_horizonPub.publish(traj);
	}
}
