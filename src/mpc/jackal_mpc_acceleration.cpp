#include <string>
#include <math.h>
#include <iostream>
#include <ros/ros.h>
#include <cppad/ipopt/solve.hpp>

#include "faster/termcolor.hpp"
#include "mpc/jackal_mpc_acceleration.h"

// ROS IPOPT-based MPC for differential drive/skid-steering vehicles
// Code modifies the car-based MPC tracking github below:
// https://github.com/Danziger/CarND-T2-P5-Model-Predictive-Control-MPC

namespace
{
    using CppAD::AD;

    class FG_eval
    {
    public:
        double _dt, _ref_cte, _ref_etheta, _ref_vel;
        double _w_cte, _w_etheta, _w_vel, _w_angvel, _w_angvel_d, _w_linvel_d, _w_pos;
        int _mpc_steps;

        AD<double> cost_cte, cost_etheta, cost_vel;

        FG_eval()
        {

            // Set default value
            _dt = 0.1; // in sec
            _ref_cte = 0;
            _ref_etheta = 0;
            _ref_vel = 0.5; // m/s

            // Cost function weights
            _w_cte = 100;
            _w_etheta = .5;
            _w_vel = 0;
            _w_angvel = 0;
            _w_angvel_d = 0;
            _w_linvel_d = 0;
            _w_pos = 1;

            use_cbf = false;
            alpha = 1.0;
            colinear = 0.01;
            padding = .05;

            _mpc_steps = 10;
            _x_start = 0;
            _y_start = _x_start + _mpc_steps;
            _theta_start = _y_start + _mpc_steps;
            _v_start = _theta_start + _mpc_steps;
            _cte_start = _v_start + _mpc_steps;
            _etheta_start = _cte_start + _mpc_steps;
            _angvel_start = _etheta_start + _mpc_steps;
            _linacc_start = _angvel_start + _mpc_steps - 1;
        }

        // Load parameters for constraints
        void LoadParams(const std::map<std::string, double> &params)
        {
            _dt = params.find("DT") != params.end() ? params.at("DT") : _dt;
            _mpc_steps = params.find("STEPS") != params.end() ? params.at("STEPS") : _mpc_steps;
            _ref_cte = params.find("REF_CTE") != params.end() ? params.at("REF_CTE") : _ref_cte;
            _ref_etheta = params.find("REF_ETHETA") != params.end() ? params.at("REF_ETHETA") : _ref_etheta;
            _ref_vel = params.find("REF_V") != params.end() ? params.at("REF_V") : _ref_vel;

            _w_pos = params.find("W_POS") != params.end() ? params.at("W_POS") : _w_pos;
            _w_cte = params.find("W_CTE") != params.end() ? params.at("W_CTE") : _w_cte;
            _w_etheta = params.find("W_ETHETA") != params.end() ? params.at("W_ETHETA") : _w_etheta;
            _w_vel = params.find("W_V") != params.end() ? params.at("W_V") : _w_vel;
            _w_angvel = params.find("W_ANGVEL") != params.end() ? params.at("W_ANGVEL") : _w_angvel;
            _w_angvel_d = params.find("W_DANGVEL") != params.end() ? params.at("W_DANGVEL") : _w_angvel_d;
            _w_linvel_d = params.find("W_DA") != params.end() ? params.at("W_DA") : _w_linvel_d;

            use_cbf = params.find("USE_CBF") != params.end() ? params.at("USE_CBF") : use_cbf;
            alpha = params.find("CBF_ALPHA") != params.end() ? params.at("CBF_ALPHA") : alpha;
            colinear = params.find("CBF_COLINEAR") != params.end() ? params.at("CBF_COLINEAR") : colinear;
            padding = params.find("CBF_PADDING") != params.end() ? params.at("CBF_PADDING") : padding;

            // print all parameters
            // std::cout << "DT: " << _dt << std::endl;
            // std::cout << "STEPS: " << _mpc_steps << std::endl;
            // std::cout << "REF_CTE: " << _ref_cte << std::endl;
            // std::cout << "REF_ETHETA: " << _ref_etheta << std::endl;
            // std::cout << "REF_V: " << _ref_vel << std::endl;
            // std::cout << "W_POS: " << _w_pos << std::endl;
            // std::cout << "W_CTE: " << _w_cte << std::endl;
            // std::cout << "W_ETHETA: " << _w_etheta << std::endl;
            // std::cout << "W_V: " << _w_vel << std::endl;
            // std::cout << "W_ANGVEL: " << _w_angvel << std::endl;
            // std::cout << "W_DANGVEL: " << _w_angvel_d << std::endl;
            // std::cout << "W_DA: " << _w_linvel_d << std::endl;

            _x_start = 0;
            _y_start = _x_start + _mpc_steps;
            _theta_start = _y_start + _mpc_steps;
            _v_start = _theta_start + _mpc_steps;
            _cte_start = _v_start + _mpc_steps;
            _etheta_start = _cte_start + _mpc_steps;
            _angvel_start = _etheta_start + _mpc_steps;
            _linacc_start = _angvel_start + _mpc_steps - 1;

            // cout << "\n!! FG_eval Obj parameters updated !! " << _mpc_steps << endl;
        }

        // void setCBF(bool use_cbf, double alpha, double colinear, double padding)
        // {
        //     this->use_cbf = use_cbf;
        //     this->alpha = alpha;
        //     this->colinear = colinear;
        //     this->padding = padding;
        // }

        void setDistMap(const std::shared_ptr<distmap::DistanceMap> &dist_map)
        {
            _dist_grid_ptr = dist_map;
        }

        void setObstacle(const Eigen::Vector3d &obstacle)
        {
            _obstacle = obstacle;
        }

        void setWpts(const Eigen::MatrixXd &wpts)
        {
            this->wpts = wpts;

            traj_omgs = Eigen::VectorXd::Zero(_mpc_steps);
            for (int i = 0; i < _mpc_steps; ++i)
            {
                double ref_vel_x = wpts(1, i);
                double ref_acc_x = wpts(2, i);
                double ref_vel_y = wpts(4, i);
                double ref_acc_y = wpts(5, i);
                if (ref_vel_x * ref_vel_x + ref_vel_y * ref_vel_y < 1e-6)
                    traj_omgs(i) = 0;
                else
                    traj_omgs(i) = (ref_acc_y * ref_vel_x - ref_acc_x * ref_vel_y) /
                                   (pow(ref_vel_x, 2) + pow(ref_vel_y, 2));
            }
        }

        void setGoal(const Eigen::Vector3d &goalPose)
        {
        }

        // MPC implementation (cost func & constraints)
        typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
        void operator()(ADvector &fg, const ADvector &vars)
        {
            // cost function is fg[0]
            fg[0] = 0;

            for (int i = 0; i < _mpc_steps; i++)
            {

                AD<double> ref_x = wpts(0, i);
                AD<double> ref_y = wpts(3, i);

                fg[0] += _w_pos * CppAD::pow(vars[_x_start + i] - ref_x, 2);
                fg[0] += _w_pos * CppAD::pow(vars[_y_start + i] - ref_y, 2);
                fg[0] += _w_cte * CppAD::pow(vars[_cte_start + i], 2);
                fg[0] += _w_etheta * CppAD::pow(vars[_etheta_start + i], 2);
                // fg[0] += _w_vel * CppAD::pow(vars[_linacc_start+i]-ref_acc, 2);
            }

            // Minimize the use of actuators.
            for (int i = 0; i < _mpc_steps - 1; i++)
            {
                AD<double> ref_acc_x = wpts(2, i);
                AD<double> ref_acc_y = wpts(5, i);
                AD<double> ref_acc = CppAD::sqrt(CppAD::pow(ref_acc_x, 2) + CppAD::pow(ref_acc_y, 2));

                fg[0] += _w_angvel * CppAD::pow(vars[_angvel_start + i], 2);
                fg[0] += _w_vel * CppAD::pow(vars[_linacc_start + i] - ref_acc, 2);
                //   fg[0] += _w_vel * CppAD::pow(vars[_linacc_start + i],2);
            }

            // // Minimize the value gap between sequential actuations.
            for (int i = 0; i < _mpc_steps - 2; i++)
            {
                fg[0] += _w_angvel_d * CppAD::pow(vars[_angvel_start + i + 1] - vars[_angvel_start + i], 2);
                fg[0] += _w_linvel_d * CppAD::pow(vars[_linacc_start + i + 1] - vars[_linacc_start + i], 2);
            }

            // fg[x] for constraints
            // Initial constraints
            fg[1 + _x_start] = vars[_x_start];
            fg[1 + _y_start] = vars[_y_start];
            fg[1 + _theta_start] = vars[_theta_start];
            fg[1 + _v_start] = vars[_v_start];
            fg[1 + _cte_start] = vars[_cte_start];
            fg[1 + _etheta_start] = vars[_etheta_start];

            // Add system dynamic model constraint
            for (int i = 0; i < _mpc_steps - 1; i++)
            {
                // The state at time t+1 .
                AD<double> x1 = vars[_x_start + i + 1];
                AD<double> y1 = vars[_y_start + i + 1];
                AD<double> theta1 = vars[_theta_start + i + 1];
                AD<double> v1 = vars[_v_start + i + 1];
                AD<double> cte1 = vars[_cte_start + i + 1];
                AD<double> etheta1 = vars[_etheta_start + i + 1];

                // The state at time t.
                AD<double> x0 = vars[_x_start + i];
                AD<double> y0 = vars[_y_start + i];
                AD<double> theta0 = vars[_theta_start + i];
                AD<double> v0 = vars[_v_start + i];
                AD<double> cte0 = vars[_cte_start + i];
                AD<double> etheta0 = vars[_etheta_start + i];

                // trajectory angular vel (omega)
                // AD<double> ref_vel_x = wpts(1,i);
                // AD<double> ref_acc_x = wpts(2,i);
                // AD<double> ref_vel_y = wpts(4,i);
                // AD<double> ref_acc_y = wpts(5,i);

                AD<double> traj_omg = traj_omgs(i);
                // if (ref_vel_x*ref_vel_x + ref_vel_y*ref_vel_y < 1e-6)
                //     traj_omg = 0;
                // else{
                //     traj_omg = (ref_acc_y*ref_vel_x - ref_acc_x*ref_vel_y)/
                //     (CppAD::pow(ref_vel_x,2) + CppAD::pow(ref_vel_y,2));
                // }

                // Only consider the actuation at time t.
                // AD<double> angvel0 = vars[_angvel_start + i];
                AD<double> w0 = vars[_angvel_start + i];
                AD<double> a0 = vars[_linacc_start + i];

                // model equations
                fg[2 + _x_start + i] = x1 - (x0 + v0 * CppAD::cos(theta0) * _dt);
                fg[2 + _y_start + i] = y1 - (y0 + v0 * CppAD::sin(theta0) * _dt);
                fg[2 + _theta_start + i] = theta1 - (theta0 + w0 * _dt);
                fg[2 + _v_start + i] = v1 - (v0 + a0 * _dt);
                fg[2 + _cte_start + i] = cte1 - (cte0 + v0 * CppAD::sin(etheta0) * _dt);
                fg[2 + _etheta_start + i] = etheta1 - (etheta0 + (w0 - traj_omg) * _dt);

                if (use_cbf)
                {
                    AD<double> alpha = this->alpha;
                    AD<double> d2 = this->colinear;
                    AD<double> padding = this->padding;

                    AD<double> xdot = v0 * cos(theta0);
                    AD<double> ydot = v0 * sin(theta0);

                    AD<double> dist = _dist_grid_ptr->atPositionSafe(
                                          CppAD::Value(CppAD::Var2Par(x0)), CppAD::Value(CppAD::Var2Par(y0)), true) +
                                      1e-6;
                    AD<double> D = dist - padding - 1e-6;

                    distmap::DistanceMap::Gradient grad = _dist_grid_ptr->gradientAtPosition(
                        CppAD::Value(CppAD::Var2Par(x0)), CppAD::Value(CppAD::Var2Par(y0)), true);
                    AD<double> dx = grad.dx;
                    AD<double> dy = grad.dy;
                    AD<double> grad_norm = CppAD::sqrt(dx * dx + dy * dy);
                    dx *= (-dist / grad_norm);
                    dy *= (-dist / grad_norm);

                    AD<double> p0 = (dx * CppAD::cos(theta0) + dy * CppAD::sin(theta0)) / dist;
                    AD<double> pp = (-dy * CppAD::cos(theta0) + dx * CppAD::sin(theta0)) / dist;
                    AD<double> P = p0 + v0 * d2;

                    AD<double> Lfh1 = (CppAD::exp(-P) * xdot / dist) * (-dx + D * (CppAD::cos(theta0) - dx * p0 / dist));
                    AD<double> Lfh2 = (CppAD::exp(-P) * ydot / dist) * (-dy + D * (CppAD::sin(theta0) - dy * p0 / dist));
                    AD<double> Lfh = Lfh1 + Lfh2;

                    AD<double> Lgh1 = -D * d2 * CppAD::exp(-P);
                    AD<double> Lgh2 = D * pp * CppAD::exp(-P);

                    AD<double> h = D * CppAD::exp(-P);

                    fg[2 + _etheta_start + _mpc_steps + i] = Lfh + Lgh1 * a0 + Lgh2 * w0 + alpha * h;
                }

            }
        }

    private:
        int _x_start, _y_start, _theta_start, _v_start, _cte_start, _etheta_start, _angvel_start, _linacc_start;
        Eigen::MatrixXd wpts;
        Eigen::VectorXd traj_omgs;
        Eigen::Vector3d _obstacle;

        bool use_cbf;
        double alpha;
        double padding;
        double colinear;

        std::shared_ptr<distmap::DistanceMap> _dist_grid_ptr;
    };
}

MPC_ACC::MPC_ACC()
{
    // Set default value
    _mpc_steps = 10;
    _max_angvel = 3.0;    // Maximal angvel radian (~30 deg)
    _max_linvel = 2.0;    // Maximal linvel accel
    _max_linacc = 4.0;    // Maximal linacc accel
    _bound_value = 1.0e3; // Bound value for other variables

    use_cbf = false;
    alpha = 1.0;
    colinear = 0.01;
    padding = .05;

    _x_start = 0;
    _y_start = _x_start + _mpc_steps;
    _theta_start = _y_start + _mpc_steps;
    _v_start = _theta_start + _mpc_steps;
    _cte_start = _v_start + _mpc_steps;
    _etheta_start = _cte_start + _mpc_steps;
    _angvel_start = _etheta_start + _mpc_steps;
    _linacc_start = _angvel_start + _mpc_steps - 1;
}

void MPC_ACC::LoadParams(const std::map<std::string, double> &params)
{
    _params = params;
    // Init parameters for MPC object
    _mpc_steps = _params.find("STEPS") != _params.end() ? _params.at("STEPS") : _mpc_steps;
    _max_angvel = _params.find("ANGVEL") != _params.end() ? _params.at("ANGVEL") : _max_angvel;
    _max_linvel = _params.find("LINVEL") != _params.end() ? _params.at("LINVEL") : _max_linvel;
    _max_linacc = _params.find("MAX_LINACC") != _params.end() ? _params.at("MAX_LINACC") : _max_linacc;
    _bound_value = _params.find("BOUND") != _params.end() ? _params.at("BOUND") : _bound_value;

    use_cbf = params.find("USE_CBF") != params.end() ? params.at("USE_CBF") : use_cbf;
    alpha = params.find("CBF_ALPHA") != params.end() ? params.at("CBF_ALPHA") : alpha;
    colinear = params.find("CBF_COLINEAR") != params.end() ? params.at("CBF_COLINEAR") : colinear;
    padding = params.find("CBF_PADDING") != params.end() ? params.at("CBF_PADDING") : padding;

    // std::cerr << "max linvel is " << _max_linvel << std::endl;

    _x_start = 0;
    _y_start = _x_start + _mpc_steps;
    _theta_start = _y_start + _mpc_steps;
    _v_start = _theta_start + _mpc_steps;
    _cte_start = _v_start + _mpc_steps;
    _etheta_start = _cte_start + _mpc_steps;
    _angvel_start = _etheta_start + _mpc_steps;
    _linacc_start = _angvel_start + _mpc_steps - 1;

    if (use_cbf){
        std::cout << termcolor::green << "USING CBF" << std::endl;
        std::cout << "alpha is " << alpha << std::endl;
    }
    else
        std::cout << termcolor::red << "NOT USING CBF" << termcolor::reset << std::endl;

    std::cout << "\n!! MPC Obj parameters updated !! " << std::endl;

}

void MPC_ACC::updateObstacle(const Eigen::Vector3d &obstacle)
{
    _obstacle = obstacle;
}

void MPC_ACC::set_dist_map(const std::shared_ptr<distmap::DistanceMap> &dist_map)
{
    _dist_grid_ptr = dist_map;
    std::cout << "[MPC_ACC] Distance map set!" << std::endl;
}

std::vector<double> MPC_ACC::Solve(const Eigen::VectorXd &state, const Eigen::MatrixXd &wpts)
{
    bool ok = true;

    // if (_dist_grid_ptr.get() == nullptr)
    // {
    //     std::cerr << "Distance map not set" << std::endl;
    //     return {0, 0};
    // }

    typedef CPPAD_TESTVECTOR(double) Dvector;
    const double x = state[0];
    const double y = state[1];
    const double theta = state[2];
    const double v = state[3];
    const double cte = state[4];
    const double etheta = state[5];

    // Set the number of model variables (includes both states and inputs).
    // For example: If the state is a 4 element vector, the actuators is a 2
    // element vector and there are 10 timesteps. The number of variables is:
    // 4 * 10 + 2 * 9

    size_t n_vars = _mpc_steps * state.size() + (_mpc_steps - 1) * 2;

    // Set the number of constraints
    size_t n_constraints = _mpc_steps * state.size();

    if (use_cbf)
        n_constraints += _mpc_steps;

    // std::cerr << "N_VARS: " << n_vars << std::endl;
    // std::cerr << "N_CONSTRAINTS: " << n_constraints << std::endl;

    // Initial value of the independent variables.
    // SHOULD BE 0 besides initial state.
    Dvector vars(n_vars);
    for (int i = 0; i < n_vars; i++)
    {
        vars[i] = 0;
    }

    // Set the initial variable values
    vars[_x_start] = x;
    vars[_y_start] = y;
    vars[_theta_start] = theta;
    vars[_v_start] = v;
    vars[_cte_start] = cte;
    vars[_etheta_start] = etheta;

    // warm start with previous solution
    if (mpc_x.size() > 0)
    {
        for (int i = 1; i < _mpc_steps; ++i)
        {
            vars[_x_start + i] = mpc_x[i];
            vars[_y_start + i] = mpc_y[i];
            vars[_theta_start + i] = mpc_theta[i];
            vars[_v_start + i] = mpc_linvels[i];
            vars[_cte_start + i] = mpc_ctes[i];
            vars[_etheta_start + i] = mpc_ethetas[i];
        }

        for (int i = 0; i < _mpc_steps - 1; ++i)
        {
            vars[_angvel_start + i] = mpc_angvels[i];
            vars[_linacc_start + i] = mpc_linaccs[i];
        }
    }

    // Set lower and upper limits for variables.
    Dvector vars_lowerbound(n_vars);
    Dvector vars_upperbound(n_vars);

    // Set all non-actuators upper and lowerlimits
    // to the max negative and positive values.
    for (int i = 0; i < _angvel_start; i++)
    {
        vars_lowerbound[i] = -_bound_value;
        vars_upperbound[i] = _bound_value;
    }

    for (int i = _v_start; i < _cte_start; i++)
    {
        // vars_lowerbound[i] = 0; //-_max_linvel;
        vars_lowerbound[i] = -_max_linvel;
        vars_upperbound[i] = _max_linvel;
    }

    // The upper and lower limits of angvel are set to -25 and 25
    // degrees (values in radians).
    for (int i = _angvel_start; i < _linacc_start; i++)
    {
        vars_lowerbound[i] = -_max_angvel;
        vars_upperbound[i] = _max_angvel;
    }

    // Acceleration/decceleration upper and lower limits
    for (int i = _linacc_start; i < n_vars; i++)
    {
        vars_lowerbound[i] = -_max_linacc;
        vars_upperbound[i] = _max_linacc;
    }

    // Lower and upper limits for the constraints
    // Should be 0 besides initial state.
    Dvector constraints_lowerbound(n_constraints);
    Dvector constraints_upperbound(n_constraints);

    for (int i = 0; i < n_constraints; i++)
    {
        constraints_lowerbound[i] = 0;
        constraints_upperbound[i] = 0;
    }

    if (use_cbf)
    {
        for (int i = n_constraints - _mpc_steps; i < n_constraints; i++)
        {
            constraints_lowerbound[i] = 0.0;
            constraints_upperbound[i] = 1.0e19;
        }
    }

    constraints_lowerbound[_x_start] = x;
    constraints_lowerbound[_y_start] = y;
    constraints_lowerbound[_theta_start] = theta;
    constraints_lowerbound[_v_start] = v;
    constraints_lowerbound[_cte_start] = cte;
    constraints_lowerbound[_etheta_start] = etheta;

    constraints_upperbound[_x_start] = x;
    constraints_upperbound[_y_start] = y;
    constraints_upperbound[_theta_start] = theta;
    constraints_upperbound[_v_start] = v;
    constraints_upperbound[_cte_start] = cte;
    constraints_upperbound[_etheta_start] = etheta;

    // object that computes objective and constraints
    FG_eval fg_eval;
    fg_eval.LoadParams(_params);
    fg_eval.setWpts(wpts);
    fg_eval.setObstacle(_obstacle);
    fg_eval.setDistMap(_dist_grid_ptr);
    // fg_eval.setCBF(use_cbf, alpha, colinear, padding);

    // options for IPOPT solver
    std::string options;
    // Uncomment this if you'd like more print information
    options += "Integer print_level  0\n";
    // NOTE: Setting sparse to true allows the solver to take advantage
    // of sparse routines, this makes the computation MUCH FASTER. If you
    // can uncomment 1 of these and see if it makes a difference or not but
    // if you uncomment both the computation time should go up in orders of
    // magnitude.
    options += "Sparse  true        forward\n";
    options += "Sparse  true        reverse\n";
    // NOTE: Currently the solver has a maximum time limit of 0.5 seconds.
    // Change this as you see fit.
    options += "Numeric max_cpu_time          .5\n";

    // place to return solution
    CppAD::ipopt::solve_result<Dvector> solution;

    // solve the problem
    // std::cout << "starting solver" << std::endl;
    CppAD::ipopt::solve<Dvector, FG_eval>(
        options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound,
        constraints_upperbound, fg_eval, solution);

    // Check some of the solution values
    ok &= (solution.status == CppAD::ipopt::solve_result<Dvector>::success || solution.status == CppAD::ipopt::solve_result<Dvector>::stop_at_acceptable_point || solution.status == CppAD::ipopt::solve_result<Dvector>::feasible_point_found);

    if (!ok)
    {
        std::cerr << "Error occured during solve (" << solution.status << ")" << std::endl;
        // print out state
        std::cout << "x: " << x << ", y: " << y << ", theta: " << theta << ", linvel: " << v << ", cte: " << cte << ", etheta: " << etheta << std::endl;
        solution.x[_angvel_start] = 0;
        solution.x[_linacc_start] = 0;
    }

    // Cost
    auto cost = solution.obj_value;
    // std::cout << "------------ Total Cost(solution): " << cost << " ------------" << std::endl;
    // std::cout << "max_angvel:" << _max_angvel <<std::endl;
    // std::cout << "max_linvel:" << _max_linvel <<std::endl;

    // std::cout << "-----------------------------------------------" <<std::endl;

    mpc_x = {};
    mpc_y = {};
    mpc_theta = {};
    mpc_linvels = {};
    mpc_ctes = {};
    mpc_ethetas = {};

    for (int i = 0; i < _mpc_steps; i++)
    {
        mpc_x.push_back(solution.x[_x_start + i]);
        mpc_y.push_back(solution.x[_y_start + i]);
        mpc_theta.push_back(solution.x[_theta_start + i]);
        mpc_linvels.push_back(solution.x[_v_start + i]);
        mpc_ctes.push_back(solution.x[_cte_start + i]);
        mpc_ethetas.push_back(solution.x[_etheta_start + i]);
    }

    mpc_angvels = {};
    mpc_linaccs = {};
    for (int i = 0; i < _mpc_steps - 1; i++)
    {
        mpc_angvels.push_back(solution.x[_angvel_start + i]);
        mpc_linaccs.push_back(solution.x[_linacc_start + i]);
    }

    // if heading is near -pi or pi, print out horizon and cost
    // if (fabs(mpc_theta[0]) > M_PI-1e-1 || fabs(mpc_theta[0]) < -M_PI+1e-1)
    // {
    //     std::cout << "------------ Horizon ------------" << std::endl;
    //     for (int i = 0; i < _mpc_steps; i++)
    //     {
    //         std::cout << "x: " << mpc_x[i] << ", y: " << mpc_y[i] << ", theta: " << mpc_theta[i] << ", linvel: " << mpc_linvels[i] << ", cte: " << mpc_ctes[i] << ", etheta: " << mpc_ethetas[i] << std::endl;
    //     }
    //     // std::cout << "------------ Total Cost(solution): " << cost << " ------------" << std::endl;
    //     // exit(0);
    // }

    // evaluateHFunction();

    std::vector<double> result;
    result.push_back(solution.x[_angvel_start]);
    result.push_back(solution.x[_linacc_start]);

    return result;
}

std::vector<double> MPC_ACC::evaluateHFunction()
{

    double alpha = .4;
    double d2 = .1;
    double obs_x = _obstacle(0);
    double obs_y = _obstacle(1);
    double obs_r = _obstacle(2);

    // for each state in horizon, evaluate h function
    for (int i = 0; i < _mpc_steps - 1; ++i)
    {
        double x0 = mpc_x[i];
        double y0 = mpc_y[i];
        double theta0 = mpc_theta[i];
        double v0 = mpc_linvels[i];
        double w0 = mpc_angvels[i];
        double a0 = mpc_linaccs[i];

        double xdot0 = v0 * cos(theta0);
        double ydot0 = v0 * sin(theta0);

        double dist = sqrt((x0 - obs_x) * (x0 - obs_x) + (y0 - obs_y) * (y0 - obs_y));
        double D = dist - obs_r;

        double p0 = ((obs_x - x0) * cos(theta0) + (obs_y - y0) * sin(theta0)) / dist;
        double pp = (-(obs_y - y0) * cos(theta0) + (obs_x - x0) * sin(theta0)) / dist;
        double P = p0 + v0 * d2;

        // get point on boundary of obstacle
        double x1 = obs_x + obs_r * cos(atan2(y0 - obs_y, x0 - obs_x));
        double y1 = obs_y + obs_r * sin(atan2(y0 - obs_y, x0 - obs_x));

        double Lfh1 = (exp(-P) * xdot0 / dist) * ((x0 - obs_x) + D * (cos(theta0) + (x0 - obs_x) * p0 / dist));
        double Lfh2 = (exp(-P) * ydot0 / dist) * ((y0 - obs_y) + D * (sin(theta0) + (y0 - obs_y) * p0 / dist));
        double Lfh = Lfh1 + Lfh2;

        double Lgh1 = -D * d2 * exp(-P);
        double Lgh2 = D * pp * exp(-P);

        double h = D * exp(-P);

        // std::cout << "Lfh: " << Lfh << std::endl;
        // std::cout << "Lgh1: " << Lgh1 << std::endl;
        // std::cout << "Lgh2: " << Lgh2 << std::endl;
        // std::cout << "h: " << h << std::endl;
        // std::cout << "constraint: " << Lfh + Lgh1*a0 + Lgh2*w0 + alpha*h << std::endl;
        // if (Lfh + Lgh1 * a0 + Lgh2 * w0 + alpha * h <= 0.)
        // {
        //     std::cout << "constraint violated @ index " << i << std::endl;
        //     // print out state and control
        //     std::cout << "------ state -------" << std::endl;
        //     std::cout << "x0: " << x0 << std::endl;
        //     std::cout << "y0: " << y0 << std::endl;
        //     std::cout << "theta0: " << theta0 << std::endl;
        //     std::cout << "v0: " << v0 << std::endl;
        //     std::cout << "------ control -------" << std::endl;
        //     std::cout << "w0: " << w0 << std::endl;
        //     std::cout << "a0: " << a0 << std::endl;

        //     std::cout << "------ h function -------" << std::endl;
        //     std::cout << "constraint: " << Lfh + Lgh1 * a0 + Lgh2 * w0 + alpha * h << std::endl;
        //     std::cout << "Lfh: " << Lfh << std::endl;
        //     std::cout << "Lgh1: " << Lgh1 * a0 << std::endl;
        //     std::cout << "Lgh2: " << Lgh2 * w0 << std::endl;
        //     std::cout << "h: " << h << std::endl;
        // }
        return {h, Lfh + Lgh1 * a0 + Lgh2 * w0 + alpha * h};
    }

    return {0.0, 0.0};
}

void MPC_ACC::updateGoal(const Eigen::Vector3d &goalPose)
{
}

std::vector<double> MPC_ACC::Solve(const Eigen::VectorXd &state)
{
    return {};
}
