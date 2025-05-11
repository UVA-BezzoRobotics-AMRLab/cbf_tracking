#include <iostream>
#include "filter/safety_filter_unicycle.h"

namespace
{
    using CppAD::AD;

    class FG_eval
    {
    public:
        FG_eval() : _w_idx(0), _dt(1./15.) {}

        void set_obstacle(const std::vector<double> &obstacle)
        {
            _obs_x = obstacle[0];
            _obs_y = obstacle[1];
            _obs_r = obstacle[2];
        }

        void set_desired_input(const std::vector<double> &u)
        {
            _ws = u;
            _horizon_length = u.size()+1;

            x_start = 0;
            y_start = x_start + _horizon_length;
            theta_start = y_start + _horizon_length;
            w_start = theta_start + _horizon_length;
        }

        void set_state(const std::vector<double> &x)
        {
            _x = x[0];
            _y = x[1];
            _theta = x[2];
        }

        void set_vel(const std::vector<double> &v){
            _vs = v;
        }

        typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
        void operator()(ADvector &fg, const ADvector &vars)
        {
            fg[0] = 0;

            for (int i = 0; i < _horizon_length-1; ++i)
                fg[0] += CppAD::pow(vars[w_start+i] - _ws[i], 2);

            // std::cerr << "made it passed fg[0]" << std::endl;

            // initial state constraint
            fg[1+x_start] = vars[x_start];
            fg[1+y_start] = vars[y_start];
            fg[1+theta_start] = vars[theta_start];

            // std::cerr << "made it passed initial constraints" << std::endl;

            // std::cerr << _horizon_length << std::endl;
            for(int i = 0; i < _horizon_length-1; ++i){
                AD<double> x0 = vars[x_start + i];
                AD<double> y0 = vars[y_start + i];
                AD<double> theta0 = vars[theta_start + i];

                AD<double> x1 = vars[x_start + i + 1];
                AD<double> y1 = vars[y_start + i + 1];
                AD<double> theta1 = vars[theta_start + i + 1];

                AD<double> obs_x = _obs_x;
                AD<double> obs_y = _obs_y;
                AD<double> obs_r = _obs_r;
                AD<double> dx = x0 - obs_x;
                AD<double> dy = y0 - obs_y;
                AD<double> d2 = .15;
                AD<double> a3 = .8;

                AD<double> dist = CppAD::sqrt(dx * dx + dy * dy);
                AD<double> D = dist - obs_r;
                AD<double> P = (_theta - CppAD::atan((obs_y - y0) / (obs_x - x0))) * d2;

                AD<double> divisor = D * D * CppAD::exp(P);

                // control barrier function constraint (obstacle avoidance)
                fg[1+w_start+i] = ((-1 * _vs[i] * dx / dist) * CppAD::cos(theta0) - (_vs[i] * dy / dist) * CppAD::sin(theta0) - dist * d2 * vars[w_start+i]) / divisor - a3 * D * CppAD::exp(P);
                
                fg[1+x_start+i+1] = x1 - (x0 + _vs[i] * CppAD::cos(theta0) * _dt);
                fg[1+y_start+i+1] = y1 - (y0 + _vs[i] * CppAD::sin(theta0) * _dt);
                fg[1+theta_start+i+1] = theta1 - (theta0 + vars[w_start + i] * _dt);

            }

            return;
        }

    protected:
        int _w_idx, _horizon_length, x_start, y_start, theta_start, w_start;
        double _x, _y, _theta, _obs_x, _obs_y, _obs_r, _vel, _dt;
        std::vector<double> _ws, _vs;
    };
}

SafetyFilterUnicycle::SafetyFilterUnicycle()
{
    _w_idx = 0;
}

std::vector<double> SafetyFilterUnicycle::filter(
    const std::vector<double> &x,
    const std::vector<double> &v,
    const std::vector<double> &u,
    const std::vector<double> &obstacle)
{
    bool ok = true;

    typedef CPPAD_TESTVECTOR(double) Dvector;

    // horizon length
    double horizon_length = u.size() + 1;

    int x_start = 0;
    int y_start = x_start + horizon_length;
    int theta_start = y_start + horizon_length;
    int w_start = theta_start + horizon_length;
    
    // number of independent variables
    size_t n_vars = horizon_length-1 + (horizon_length)*x.size();
    // number of constraints
    size_t n_constraints = horizon_length * x.size() + horizon_length-1;


    // std::cerr << "hello0" << std::endl;
    // initial value of the independent variables
    Dvector vars(n_vars);
    for (size_t i = 0; i < n_vars; ++i)
    {
        vars[i] = 0;
    }

    // std::cerr << "hello1" << std::endl;
    
    // set initial values for variables
    vars[x_start] = x[0];
    vars[y_start] = x[1];
    vars[theta_start] = x[2];
    for (int i = w_start; i < n_vars; ++i)
    {
        vars[i] = u[i];
    }
    // std::cerr << "hello2" << std::endl;
    
    // lower and upper limits for x
    Dvector vars_lowerbound(n_vars), vars_upperbound(n_vars);
    for (size_t i = 0; i < w_start; ++i)
    {
        vars_lowerbound[i] = -1e19;
        vars_upperbound[i] = 1e19;
    }

    // std::cerr << "hello3" << std::endl;

    for (size_t i = w_start; i < n_vars; ++i)
    {
        vars_lowerbound[i] = -M_PI / 3;
        vars_upperbound[i] = M_PI / 3;
    }
    // std::cerr << "hello4" << std::endl;

    // lower and upper limits for constraints
    Dvector constraints_lowerbound(n_constraints);
    Dvector constraints_upperbound(n_constraints);

    for(int i = 0; i < n_constraints-(horizon_length-1); ++i){
        constraints_lowerbound[i] = 0;
        constraints_upperbound[i] = 0;
    }
    // std::cerr << "hello5" << std::endl;

    constraints_lowerbound[x_start] = x[0];
    constraints_lowerbound[y_start] = x[1];
    constraints_lowerbound[theta_start] = x[2];
    
    constraints_upperbound[x_start] = x[0];
    constraints_upperbound[y_start] = x[1];
    constraints_upperbound[theta_start] = x[2];

    // std::cerr << "test" << std::endl;
    for(int i = n_constraints-(horizon_length-1); i < n_constraints; ++i){
        constraints_lowerbound[i] = -1e19;
        constraints_upperbound[i] = 0;
    }
    // std::cerr << "test1" << std::endl;

    // object that computes objective and constraints
    FG_eval fg_eval;
    // std::cerr << "??" << std::endl;
    fg_eval.set_state(x);
    fg_eval.set_vel(v);
    fg_eval.set_desired_input(u);
    fg_eval.set_obstacle(obstacle);

    // std::cerr << "wtf is going on?" << std::endl;

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

    // std::cerr << "solving now" << std::endl;
    // solve the problem
    CppAD::ipopt::solve<Dvector, FG_eval>(
        options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound,
        constraints_upperbound, fg_eval, solution);

    // Check some of the solution values
    ok &= (solution.status == CppAD::ipopt::solve_result<Dvector>::success || solution.status == CppAD::ipopt::solve_result<Dvector>::stop_at_acceptable_point || solution.status == CppAD::ipopt::solve_result<Dvector>::feasible_point_found);

    filter_x = {};
    filter_y = {};
    for(int i = 0; i < horizon_length; ++i){
        filter_x.push_back(solution.x[x_start+i]);
        filter_y.push_back(solution.x[y_start+i]);
    }

    return {solution.x[w_start]};
}
