#include <iostream>
#include "filter/safety_filter_extended_unicycle.h"

namespace
{
    using CppAD::AD;

    class FG_eval
    {
    public:
        FG_eval() : _w_idx(0), _dt(1. / 15.) {}

        void set_obstacle(const std::vector<double> &obstacle)
        {
            _obs_x = obstacle[0];
            _obs_y = obstacle[1];
            _obs_r = obstacle[2];
        }

        void set_desired_input(const std::vector<double> &a, const std::vector<double> &w)
        {
            _as = a;
            _ws = w;
            _horizon_length = _as.size() + 1;

            x_start = 0;
            y_start = x_start + _horizon_length;
            v_start = y_start + _horizon_length;
            x_dot_start = v_start + _horizon_length;
            y_dot_start = x_dot_start + _horizon_length;
            a_start = y_dot_start + _horizon_length;
            w_start = a_start + _horizon_length - 1;

            // h_start is a constraint index that isn't actually a variable
            h_start = a_start;

        }

        void set_state(const std::vector<double> &x)
        {
            _x = x[0];
            _y = x[1];
            _theta = x[2];
        }

        typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
        void operator()(ADvector &fg, const ADvector &vars)
        {
            fg[0] = 0;

            for (int i = 0; i < _horizon_length - 1; ++i){
                fg[0] += CppAD::pow(vars[a_start + i] - _as[i], 2);
                fg[0] += CppAD::pow(vars[w_start + i] - _ws[i], 2);
            }

            // initial state constraint
            fg[1 + x_start] = vars[x_start];
            fg[1 + y_start] = vars[y_start];
            fg[1 + v_start] = vars[v_start];
            fg[1 + x_dot_start] = vars[x_dot_start];
            fg[1 + y_dot_start] = vars[y_dot_start];

            // std::cerr << "made it passed initial constraints" << std::endl;

            // std::cerr << _horizon_length << std::endl;
            for (int i = 0; i < _horizon_length - 1; ++i)
            {
                AD<double> x0 = vars[x_start + i];
                AD<double> y0 = vars[y_start + i];
                AD<double> v0 = vars[v_start + i];
                AD<double> xdot0 = vars[x_dot_start + i];
                AD<double> ydot0 = vars[y_dot_start + i];

                AD<double> x1 = vars[x_start + i + 1];
                AD<double> y1 = vars[y_start + i + 1];
                AD<double> v1 = vars[v_start + i + 1];
                AD<double> xdot1 = vars[x_dot_start + i + 1];
                AD<double> ydot1 = vars[y_dot_start + i + 1];

                AD<double> a = vars[a_start + i];
                AD<double> w = vars[w_start + i];

                AD<double> obs_x = _obs_x;
                AD<double> obs_y = _obs_y;
                AD<double> obs_r = _obs_r;
                // AD<double> dx = x0 - obs_x;
                // AD<double> dy = y0 - obs_y;

                // AD<double> d2 = .15;
                // AD<double> a3 = .8;

                // AD<double> dist = CppAD::sqrt(dx * dx + dy * dy);
                // AD<double> D = dist - obs_r;
                // AD<double> P = (_theta - CppAD::atan((obs_y - y0) / (obs_x - x0))) * d2;

                // AD<double> divisor = D * D * CppAD::exp(P);

                // AD<double> theta = CppAD::atan(ydot0/xdot0);
                AD<double> alpha = .4;
                AD<double> d2 = 1;
                AD<double> dist = CppAD::sqrt((x0-obs_x)*(x0-obs_x) + (y0-obs_y)*(y0-obs_y));

                AD<double> D = dist - obs_r;
                AD<double> p0 = ((obs_x-x0)*CppAD::cos(_theta) + (obs_y-y0)*CppAD::sin(_theta))/dist;
                AD<double> pp = (-(obs_y-y0)*CppAD::cos(_theta) + (obs_x-x0)*CppAD::sin(_theta))/dist;
                AD<double> P =  p0 + v0*d2;

                AD<double> Lfh1 = (CppAD::exp(-P)*xdot0/dist)*((x0-obs_x) + D*(CppAD::cos(_theta) + (x0-obs_x)*p0/dist));
                AD<double> Lfh2 = (CppAD::exp(-P)*ydot0/dist)*((y0-obs_y) + D*(CppAD::sin(_theta) + (y0-obs_y)*p0/dist));
                AD<double> Lfh = Lfh1 + Lfh2;

                AD<double> Lgh1 = -D*d2*CppAD::exp(-P);
                AD<double> Lgh2 = D*pp*CppAD::exp(-P);

                AD<double> h = D*CppAD::exp(-P);

                // control barrier function constraint (obstacle avoidance)
                fg[1 + h_start + i] = Lfh + Lgh1*a + Lgh2*w + alpha*h;

                fg[1 + x_start + i + 1] = x1 - (x0 + _dt * xdot0);
                fg[1 + y_start + i + 1] = y1 - (y0 + _dt * ydot0);
                fg[1 + v_start + i + 1] = v1 - (v0 + _dt * a);

                // TODO: Find way to remove this if statement
                // if (fabs(v0) > 1e-8){
                    fg[1 + x_dot_start + i + 1] = xdot1 - (a*CppAD::cos(_theta) - ydot0)*_dt;
                    fg[1 + y_dot_start + i + 1] = ydot1 - (a*CppAD::sin(_theta) + xdot0)*_dt;
                // }
            }

            return;
        }

    protected:
        int _w_idx, _horizon_length, x_start, y_start, v_start, x_dot_start, 
            y_dot_start, a_start, w_start, h_start;
        double _x, _y, _theta, _obs_x, _obs_y, _obs_r, _vel, _dt;
        std::vector<double> _ws, _as;
    };
}

SafetyFilterUnicycle::SafetyFilterUnicycle()
{
    _w_idx = 0;
}

std::vector<double> SafetyFilterUnicycle::filter(
    const std::vector<double> &x,
    const std::vector<double> &a,
    const std::vector<double> &w,
    const std::vector<double> &obstacle)
{
    bool ok = true;

    typedef CPPAD_TESTVECTOR(double) Dvector;

    // horizon length
    double horizon_length = a.size() + 1;

    int x_start = 0;
    int y_start = x_start + horizon_length;
    int v_start = y_start + horizon_length;
    int x_dot_start = v_start + horizon_length;
    int y_dot_start = x_dot_start + horizon_length;
    int a_start = y_dot_start + horizon_length;
    int w_start = a_start + horizon_length - 1;

    // number of independent variables
    size_t n_vars = (horizon_length - 1)*2 + (horizon_length)*x.size();
    // number of constraints
    size_t n_constraints = horizon_length * x.size() + horizon_length - 1;

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
    vars[v_start] = x[3];
    vars[x_dot_start] = x[4];
    vars[y_dot_start] = x[5];

    for (int i = a_start; i < w_start; ++i)
    {
        vars[i] = a[i];
    }

    for (int i = w_start; i < n_vars; ++i)
    {
        vars[i] = w[i];
    }
    // std::cerr << "hello2" << std::endl;

    // lower and upper limits for x
    Dvector vars_lowerbound(n_vars), vars_upperbound(n_vars);
    for (size_t i = 0; i < v_start; ++i)
    {
        vars_lowerbound[i] = -1e19;
        vars_upperbound[i] = 1e19;
    }

    // limit velocity
    for(size_t i = v_start; i < a_start; ++i){
        vars_lowerbound[i] = -1.5;
        vars_upperbound[i] = 1.5;
    }

    // std::cerr << "hello3" << std::endl;

    for (size_t i = a_start; i < w_start; ++i)
    {
        vars_lowerbound[i] = -3.0;
        vars_upperbound[i] = 3.0;
    }

    for (size_t i = w_start; i < n_vars; ++i)
    {
        vars_lowerbound[i] = -M_PI / 2;
        vars_upperbound[i] = M_PI / 2;
    }
    // std::cerr << "hello4" << std::endl;

    // lower and upper limits for constraints
    Dvector constraints_lowerbound(n_constraints);
    Dvector constraints_upperbound(n_constraints);

    for (int i = 0; i < n_constraints - (horizon_length - 1); ++i)
    {
        constraints_lowerbound[i] = 0;
        constraints_upperbound[i] = 0;
    }
    // std::cerr << "hello5" << std::endl;

    constraints_lowerbound[x_start] = x[0];
    constraints_lowerbound[y_start] = x[1];
    constraints_lowerbound[v_start] = x[3];
    constraints_lowerbound[x_dot_start] = x[4];
    constraints_lowerbound[y_dot_start] = x[5];

    constraints_upperbound[x_start] = x[0];
    constraints_upperbound[y_start] = x[1];
    constraints_upperbound[v_start] = x[3];
    constraints_upperbound[x_dot_start] = x[4];
    constraints_upperbound[y_dot_start] = x[5];

    // std::cerr << "test" << std::endl;
    for (int i = n_constraints - (horizon_length - 1); i < n_constraints; ++i)
    {
        constraints_lowerbound[i] = 0;
        constraints_upperbound[i] = 1e19;
    }
    // std::cerr << "test1" << std::endl;

    // object that computes objective and constraints
    FG_eval fg_eval;
    // std::cerr << "??" << std::endl;
    fg_eval.set_state({x[0], x[1], x[2]});
    fg_eval.set_desired_input(a, w);
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
    for (int i = 0; i < horizon_length; ++i)
    {
        filter_x.push_back(solution.x[x_start + i]);
        filter_y.push_back(solution.x[y_start + i]);
    }

    return {solution.x[a_start], solution.x[w_start]};
}
