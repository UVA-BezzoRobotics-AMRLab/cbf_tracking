#include <chrono>
#include <iostream>

#include "mpc/mpc_core.h"
#include "faster/termcolor.hpp"

JackalMPCCore::JackalMPCCore()
{
    // _mpc_type = MPC_TYPE_CTE;
    _mpc_type = MPC_TYPE_ACC;
    // _mpc_type = MPC_TYPE_NLOPT;
    // _mpc_type = MPC_TYPE_NLOPT_POS;

    _mpc = std::make_unique<MPC_ACC>();

    _curr_vel = 0;
    _curr_ang_vel = 0;

    _state = Eigen::VectorXd::Zero(6);

    _is_set = false;
    _is_colliding = false;

    _obstacle = Eigen::Vector3d(0, 0, 0);
}

JackalMPCCore::JackalMPCCore(const mpc_type &type)
{
    _mpc_type = type;

    if (_mpc_type == MPC_TYPE_CTE)
    {
        std::cout << termcolor::green << "[MPC Core] setting mpc type to track_vel_ipopt" << termcolor::reset << std::endl;
        _mpc = std::make_unique<MPC>();

        // state is x,y,theta,cte,etheta
        _state = Eigen::VectorXd::Zero(5);
    }
    else if (_mpc_type == MPC_TYPE_ACC)
    {
        std::cout << termcolor::green << "[MPC Core] setting mpc type to track_acc_ipopt" << termcolor::reset << std::endl;
        _mpc = std::make_unique<MPC_ACC>();

        // state is x,y,theta,v,cte,etheta
        _state = Eigen::VectorXd::Zero(6);
    }
    else if (_mpc_type == MPC_TYPE_NLOPT)
    {
        std::cout << termcolor::green << "[MPC Core] setting mpc type to track_acc_nlopt" << termcolor::reset << std::endl;
        _mpc = std::make_unique<CBFHorizon>();

        // state is x,y,theta,v,cte,etheta
        _state = Eigen::VectorXd::Zero(6);
    }
    else if (_mpc_type == MPC_TYPE_NLOPT_POS)
    {
        std::cout << termcolor::green << "[MPC Core] setting mpc type to goto_pos_nlopt" << termcolor::reset << std::endl;
        _mpc = std::make_unique<MPCNLOPT>();

        // state is x,y,theta,v
        _state = Eigen::VectorXd::Zero(4);
    }

    _curr_vel = 0;
    _curr_ang_vel = 0;

    _obstacle = Eigen::Vector3d(0, 0, 0);

    _is_set = false;
    _is_colliding = false;
}

JackalMPCCore::~JackalMPCCore()
{
    _file.close();
}

void JackalMPCCore::load_params(const std::map<std::string, double> &params)
{
    _dt = params.at("DT");
    _max_anga = params.at("MAX_ANGA");
    _max_linacc = params.at("MAX_LINACC");

    _max_vel = params.at("LINVEL");
    _max_ang_vel = params.at("ANGVEL");

    _use_cbf = params.at("USE_CBF");

    _mpc->LoadParams(params);
    // _filter.load_params(params);
}

void JackalMPCCore::set_obstacle(const Eigen::Vector3d &obstacle)
{
    _obstacle = obstacle;

    if (_mpc_type == MPC_TYPE_ACC)
    {
        MPC_ACC *mpc = dynamic_cast<MPC_ACC *>(_mpc.get());
        mpc->updateObstacle(obstacle);
    }
}

void JackalMPCCore::set_odom(const Eigen::Vector3d &odom)
{
    _odom = odom;
}

void JackalMPCCore::set_goal(const Eigen::Vector2d &goal)
{
    _goal = goal;
}

#ifdef FOUND_PYBIND11
void JackalMPCCore::set_odom(const std::vector<double> &odom)
{
    _odom = Eigen::Vector3d(odom[0], odom[1], odom[2]);
}

void JackalMPCCore::set_goal(const std::vector<double> &goal)
{
    _goal = Eigen::Vector2d(goal[0], goal[1]);
}

void JackalMPCCore::set_obstacle(const std::vector<double> &obstacle)
{
    _obstacle = Eigen::Vector3d(obstacle[0], obstacle[1], obstacle[2]);
}

// reference comes in flattened, so we need to reshape it to (6, mpc_steps)
void JackalMPCCore::set_reference(const std::vector<double> &reference)
{
    // divide size by 6 to get number of steps
    int steps = reference.size() / 6;
    Eigen::MatrixXd ref(6, steps);

    for (int i = 0; i < steps; i++)
    {
        ref(0, i) = reference[0 * steps + i];
        ref(1, i) = reference[1 * steps + i];
        ref(2, i) = reference[2 * steps + i];
        ref(3, i) = reference[3 * steps + i];
        ref(4, i) = reference[4 * steps + i];
        ref(5, i) = reference[5 * steps + i];
    }

    _reference = ref;
}

std::vector<double> JackalMPCCore::get_mpc_results()
{
    return _mpc_results;
}

bool JackalMPCCore::set_dist_map(pybind11::dict grid_dict)
{
    nav_msgs::OccupancyGrid grid;

    // Deserialize the header
    pybind11::dict header_dict = grid_dict["header"].cast<pybind11::dict>();
    pybind11::dict stamp_dict = header_dict["stamp"].cast<pybind11::dict>();

    grid.header.seq = header_dict["seq"].cast<uint32_t>();
    grid.header.stamp.sec = stamp_dict["secs"].cast<int32_t>();
    grid.header.stamp.nsec = stamp_dict["nsecs"].cast<int32_t>();
    grid.header.frame_id = header_dict["frame_id"].cast<std::string>();

    // Deserialize the info
    pybind11::dict info_dict = grid_dict["info"].cast<pybind11::dict>();

    grid.info.resolution = info_dict["resolution"].cast<float>();
    grid.info.width = info_dict["width"].cast<uint32_t>();
    grid.info.height = info_dict["height"].cast<uint32_t>();

    pybind11::dict origin_dict = info_dict["origin"].cast<pybind11::dict>();
    pybind11::dict position_dict = origin_dict["position"].cast<pybind11::dict>();

    grid.info.origin.position.x = position_dict["x"].cast<double>();
    grid.info.origin.position.y = position_dict["y"].cast<double>();
    grid.info.origin.position.z = position_dict["z"].cast<double>();

    pybind11::dict orientation_dict = origin_dict["orientation"].cast<pybind11::dict>();

    grid.info.origin.orientation.x = orientation_dict["x"].cast<double>();
    grid.info.origin.orientation.y = orientation_dict["y"].cast<double>();
    grid.info.origin.orientation.z = orientation_dict["z"].cast<double>();
    grid.info.origin.orientation.w = orientation_dict["w"].cast<double>();

    // Deserialize the data
    grid.data = grid_dict["data"].cast<std::vector<int8_t>>();

    boost::shared_ptr<distmap::DistanceMapConverterBase> dist_map_conv;
    dist_map_conv = distmap::make_distance_mapper("distmap/DistanceMapDeadReck");
    if (!dist_map_conv->process(boost::make_shared<const nav_msgs::OccupancyGrid>(grid)))
    {
        std::cout << "Distance map failed to be set from occupancy grid" << std::endl;
        return false;
    }

    _dist_grid_ptr = dist_map_conv->getDistanceFieldObstacle();

    // _filter.set_dist_map(_dist_grid_ptr);

    if (_mpc_type == MPC_TYPE_ACC)
    {
        MPC_ACC *mpc = dynamic_cast<MPC_ACC *>(_mpc.get());
        mpc->set_dist_map(_dist_grid_ptr);
    }
    if (_mpc_type == MPC_TYPE_NLOPT)
    {
        CBFHorizon *mpc = dynamic_cast<CBFHorizon *>(_mpc.get());
        mpc->set_dist_map(_dist_grid_ptr);
    }

    std::cout << "Distance map set from occupancy grid" << std::endl;

    return true;
}

std::vector<double> JackalMPCCore::get_dist_map_data(double x, double y)
{
    if (_dist_grid_ptr == nullptr)
    {
        return {};
    }

    double dist = _dist_grid_ptr->atPositionSafe(x, y, true);
    distmap::DistanceMap::Gradient grad = _dist_grid_ptr->gradientAtPosition(x, y, true);
    double dx = grad.dx;
    double dy = grad.dy;
    double grad_norm = sqrt(dx * dx + dy * dy);
    dx *= (-dist / grad_norm);
    dy *= (-dist / grad_norm);

    return {dist, dx, dy};
}

#endif

void JackalMPCCore::set_reference(const Eigen::MatrixXd &reference)
{
    _reference = reference;
}

void JackalMPCCore::set_mpc_type(const mpc_type &type)
{
}

void JackalMPCCore::set_trajectory(const std::vector<traj_point_t> &trajectory)
{
    _trajectory = trajectory;
}

void JackalMPCCore::set_dist_map(const std::shared_ptr<distmap::DistanceMap> &dist_map)
{
    _dist_grid_ptr = dist_map;
    // _filter.set_dist_map(dist_map);

    if (_mpc_type == MPC_TYPE_ACC)
    {
        MPC_ACC *mpc = dynamic_cast<MPC_ACC *>(_mpc.get());
        mpc->set_dist_map(dist_map);
    }
    if (_mpc_type == MPC_TYPE_NLOPT)
    {
        CBFHorizon *mpc = dynamic_cast<CBFHorizon *>(_mpc.get());
        mpc->set_dist_map(dist_map);
    }
}

double JackalMPCCore::get_progress()
{
    double min_dist = 1e10;
    double min_time = 0;

    for (const traj_point_t &pt : _trajectory)
    {
        double dist = sqrt(
            (_odom(0) - pt.pose(0)) * (_odom(0) - pt.pose(0)) +
            (_odom(1) - pt.pose(1)) * (_odom(1) - pt.pose(1)));

        if (dist < min_dist)
        {
            min_dist = dist;
            min_time = pt.time_from_start;
        }
    }

    return min_time / _trajectory.back().time_from_start;
}

std::vector<double> JackalMPCCore::solve()
{

    if (_mpc_type == MPC_TYPE_NLOPT_POS)
    {
        MPCNLOPT *mpc = dynamic_cast<MPCNLOPT *>(_mpc.get());
        std::cerr << "defining state" << std::endl;
        _state << _odom(0), _odom(1), _odom(2), _curr_vel;

        auto start = std::chrono::high_resolution_clock::now();
        _mpc_results = mpc->Solve_gtg(_state);
        auto end = std::chrono::high_resolution_clock::now();
        double time_to_solve = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();

        _curr_vel = limit(_curr_vel, _curr_vel + _mpc_results[1] * _dt, _max_linacc);
        _curr_ang_vel = limit(_curr_ang_vel, _mpc_results[0], _max_anga);

        // ensure vel is between -max and max and ang vel is between -max and max
        _curr_vel = std::max(-_max_vel, std::min(_max_vel, _curr_vel));
        _curr_ang_vel = std::max(-_max_ang_vel, std::min(_max_ang_vel, _curr_ang_vel));

        return {_curr_vel, _curr_ang_vel};
    }

    if (_reference.cols() == 0)
        return {};

    double ref_head = atan2(_reference(4, 0), _reference(1, 0));
    double cte = -1 * (_odom(0) - _reference(0, 0)) * sin(ref_head) + (_odom(1) - _reference(3, 0)) * cos(ref_head);
    double etheta = _odom(2) - ref_head;

    if (fabs(etheta) > M_PI)
    {
        int sign = (etheta >= 0) ? 1 : -1;
        if (sign == 1)
        {
            etheta = 2 * M_PI - etheta;
        }
        else
        {
            etheta = 2 * M_PI + etheta;
        }

        etheta *= -sign;
    }

    // std::cout << "odometry is " << _odom.transpose() << std::endl;
    double new_vel;
    double time_to_solve = 0.;
    if (_mpc_type == MPC_TYPE_CTE)
    {
        _state << _odom(0), _odom(1), _odom(2), cte, etheta;
        // time the solve function in milliseconds
        auto start = std::chrono::high_resolution_clock::now();
        _mpc_results = _mpc->Solve(_state, _reference);
        auto end = std::chrono::high_resolution_clock::now();
        time_to_solve = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();

        if (time_to_solve > _dt * 1000)
        {
            _mpc_results[0] = _curr_ang_vel;
            _mpc_results[1] = _curr_vel;
        }

        new_vel = _mpc_results[1];
    }
    else if (_mpc_type == MPC_TYPE_ACC)
    {
        _state << _odom(0), _odom(1), _odom(2), _curr_vel, cte, etheta;
        auto start = std::chrono::high_resolution_clock::now();
        _mpc_results = _mpc->Solve(_state, _reference);
        auto end = std::chrono::high_resolution_clock::now();
        time_to_solve = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();

        std::cout << "mpc wanted vel: " << _curr_vel + _mpc_results[1] * _dt << ", ang vel: " << _mpc_results[0] << std::endl;
        std::cout << "mpc done, starting filter" << std::endl;

        // if (_use_cbf)
        // {
        //     start = std::chrono::high_resolution_clock::now();
        //     _mpc_results = _filter.Solve(state, Eigen::Vector2d(_mpc_results[0], _mpc_results[1]), _obstacle);
        //     end = std::chrono::high_resolution_clock::now();

        //     std::cout << "Filter time: " << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() << std::endl;
        //     std::cout << "filter wanted vel: " << _curr_vel + _mpc_results[1] * _dt << ", ang vel: " << _mpc_results[0] << std::endl;
        // }

        if (time_to_solve > _dt * 1000)
        {
            _mpc_results[0] = _curr_ang_vel;
            _mpc_results[1] = 0;
        }

        new_vel = _curr_vel + _mpc_results[1] * _dt;
    }
    else if (_mpc_type == MPC_TYPE_NLOPT)
    {
        _state << _odom(0), _odom(1), _odom(2), _curr_vel, cte, etheta;
        auto start = std::chrono::high_resolution_clock::now();
        _mpc_results = _mpc->Solve(_state, _reference);
        auto end = std::chrono::high_resolution_clock::now();
        time_to_solve = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();

        if (time_to_solve > _dt * 1000)
        {
            _mpc_results[0] = _curr_ang_vel;
            _mpc_results[1] = 0;
        }

        new_vel = _curr_vel + _mpc_results[1] * _dt;
    }

    // std::cerr << "mpc_results: " << _mpc_results[0] << ", " << _mpc_results[1] << std::endl;
    std::cout << "Solve time: " << time_to_solve << std::endl;

    _curr_ang_vel = limit(_curr_ang_vel, _mpc_results[0], _max_anga);
    _curr_vel = limit(_curr_vel, new_vel, _max_linacc);

    // ensure vel is between -max and max and ang vel is between -max and max
    _curr_vel = std::max(-_max_vel, std::min(_max_vel, _curr_vel));
    _curr_ang_vel = std::max(-_max_ang_vel, std::min(_max_ang_vel, _curr_ang_vel));

    std::cerr << "curr vel: " << _curr_vel << ", curr ang vel: " << _curr_ang_vel << std::endl;

    return {_curr_vel, _curr_ang_vel};
}

Eigen::VectorXd JackalMPCCore::get_state()
{
    if (_mpc_type == MPC_TYPE_CTE)
    {
        return _state;
    }
    else if (_mpc_type == MPC_TYPE_ACC)
    {
        return _state;
    }
    else if (_mpc_type == MPC_TYPE_NLOPT)
    {
        CBFHorizon *mpc = dynamic_cast<CBFHorizon *>(_mpc.get());

        double dist = _dist_grid_ptr->atPositionSafe(_odom(0), _odom(1), true);
        distmap::DistanceMap::Gradient grad = _dist_grid_ptr->gradientAtPosition(_odom(0), _odom(1), true);
        double heading = atan2(grad.dy, grad.dx);

        Eigen::VectorXd rl_state(9);
        rl_state << _odom(2),           // theta
            _curr_vel,                  // current velocity
            mpc->mpc_linaccs[0],        // current acceleration
            mpc->mpc_angvels[0],        // current acceleration
            dist,                       // distance to obstacle
            heading,                    // heading to obstacle
            get_progress(),             // progress
            -mpc->h_value,              // h value
            mpc->alpha_value;           // alpha value

        return rl_state;
    }

    return _state;
}

std::vector<Eigen::VectorXd> JackalMPCCore::getHorizon()
{
    std::vector<Eigen::VectorXd> ret;
    if (_mpc_type == MPC_TYPE_CTE)
    {
        MPC *mpc = dynamic_cast<MPC *>(_mpc.get());

        int sz = mpc->mpc_x.size();
        for (int i = 0; i < sz; ++i)
        {
            Eigen::VectorXd state(3);
            state << mpc->mpc_x[i], mpc->mpc_y[i], mpc->mpc_theta[i];
            ret.push_back(state);
        }
    }
    else if (_mpc_type == MPC_TYPE_ACC)
    {
        MPC_ACC *mpc = dynamic_cast<MPC_ACC *>(_mpc.get());

        int sz = mpc->mpc_x.size();
        double t = 0;
        for (int i = 0; i < sz - 1; ++i)
        {
            t += _dt;
            Eigen::VectorXd state(6);
            state << t, mpc->mpc_x[i], mpc->mpc_y[i], mpc->mpc_theta[i], mpc->mpc_linvels[i], mpc->mpc_linaccs[i];
            ret.push_back(state);
        }
    }
    else if (_mpc_type == MPC_TYPE_NLOPT)
    {
        CBFHorizon *mpc = dynamic_cast<CBFHorizon *>(_mpc.get());

        int sz = mpc->mpc_x.size();
        double t = 0;
        for (int i = 0; i < sz - 1; ++i)
        {
            t += _dt;
            Eigen::VectorXd state(6);
            state << t, mpc->mpc_x[i], mpc->mpc_y[i], mpc->mpc_theta[i], mpc->mpc_linvels[i], mpc->mpc_linaccs[i];
            ret.push_back(state);
        }
    }

    return ret;
}

double JackalMPCCore::limit(double prev_v, double input, double max_rate)
{
    double ret = input;
    if (fabs(prev_v - input) / _dt > max_rate)
    {

        if (input > prev_v)
            ret = prev_v + max_rate * _dt;
        else
            ret = prev_v - max_rate * _dt;
    }

    return ret;
}

std::vector<double> JackalMPCCore::evaluateHFunction()
{

    if (_mpc_type == MPC_TYPE_ACC)
    {
        MPC_ACC *mpc = dynamic_cast<MPC_ACC *>(_mpc.get());
        return mpc->evaluateHFunction();
    }
    else if (_mpc_type == MPC_TYPE_NLOPT)
    {
        CBFHorizon *mpc = dynamic_cast<CBFHorizon *>(_mpc.get());
        return {mpc->h_value, mpc->alpha_value};
    }

    return {0.0, 0.0};
}
