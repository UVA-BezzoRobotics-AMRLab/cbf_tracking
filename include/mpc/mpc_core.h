#pragma once

#include <memory>

#ifdef FOUND_PYBIND11
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#endif

#include <fstream>

#include "filter/filter_nlopt.h"
#include "filter/filter_nlopt_horizon.h"

#include "mpc/mpc_nlopt.h"
#include "mpc/jackal_mpc_acceleration.h"
#include "mpc/jackal_mpc_trajectory_tracking.h"


#include <distance_map_core/distance_map_converter_base.h>
#include <distance_map_core/distance_map_converter_instantiater.h>

enum mpc_type
{
    MPC_TYPE_CTE = 0,
    MPC_TYPE_ACC = 1,
    MPC_TYPE_NLOPT = 2,
    MPC_TYPE_NLOPT_POS = 3
};

struct traj_point
{
    Eigen::Vector3d pose;
    Eigen::Vector3d velocity;
    Eigen::Vector3d acceleration;
    double time_from_start;
};
typedef struct traj_point traj_point_t;

class JackalMPCCore
{
public:
    JackalMPCCore();
    JackalMPCCore(const mpc_type &type);

    ~JackalMPCCore();

    void load_params(const std::map<std::string, double> &params);

    void set_mpc_type(const mpc_type &type);
    void set_odom(const Eigen::Vector3d &odom);
    void set_goal(const Eigen::Vector2d &goal);
    void set_obstacle(const Eigen::Vector3d &obstacle);
    void set_reference(const Eigen::MatrixXd &reference);
    void set_trajectory(const std::vector<traj_point_t> &trajectory);
    void set_dist_map(const std::shared_ptr<distmap::DistanceMap> &dist_map);


    #ifdef FOUND_PYBIND11
    // #include <pybind11/pybind11.h>
    // #include <pybind11/stl.h>
    void set_odom(const std::vector<double> &odom);
    void set_goal(const std::vector<double> &goal);
    void set_obstacle(const std::vector<double> &obstacle);
    void set_reference(const std::vector<double> &reference);

    std::vector<double> get_mpc_results();
    bool set_dist_map(pybind11::dict grid_dict);
    std::vector<double> get_dist_map_data(double x, double y);
    #endif


    std::vector<Eigen::VectorXd> getHorizon();
    Eigen::VectorXd get_state();

    std::vector<double> solve();

    std::vector<double> evaluateHFunction();
    
    bool _is_colliding;

protected:
    double get_progress();
    double limit(double prev_v, double input, double max_rate);

    double _dt;
    double _max_anga;
    double _max_linacc;
    double _curr_vel;
    double _curr_ang_vel;
    double _max_vel;
    double _max_ang_vel;

    bool _use_cbf;

    std::vector<double> _mpc_results;
    std::vector<traj_point_t> _trajectory;

    std::shared_ptr<distmap::DistanceMap> _dist_grid_ptr;

    Eigen::Vector3d _odom;
    Eigen::Vector2d _goal;
    Eigen::VectorXd _state;
    Eigen::MatrixXd _reference;

    // learning states
    Eigen::VectorXd _prev_rl_state;
    Eigen::VectorXd _curr_rl_state;

    bool _is_set;

    std::unique_ptr<MPCBase> _mpc;
    mpc_type _mpc_type;

    Eigen::Vector3d _obstacle;

    CBFFilterNLOPT _filter;

    std::ofstream _file;
};
