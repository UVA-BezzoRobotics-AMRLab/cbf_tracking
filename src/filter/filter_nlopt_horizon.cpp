#include <cmath>
#include <iostream>
#include <fstream>

#include "faster/termcolor.hpp"
#include "filter/filter_nlopt_horizon.h"

CBFHorizon::CBFHorizon()
{
    _dt = .1;
    _mpc_steps = 10;
    max_linacc = 4.0;
    max_angvel = 2.0;

    use_cbf = false;
    use_dynamic_alpha = false;
    alpha = .5;
    colinear = .1;
    padding = .1;

    h_value = 0.;

    debug = false;

    // _x_start = 0;
    // _y_start = _x_start + _mpc_steps;
    // _theta_start = _y_start + _mpc_steps;
    // _v_start = _theta_start + _mpc_steps;
    // _cte_start = _v_start + _mpc_steps;
    // _etheta_start = _cte_start + _mpc_steps;
    // _angvel_start = _etheta_start + _mpc_steps;
    // _linacc_start = _angvel_start + _mpc_steps - 1;

    // managing the indices in this way is far more cache friendly
    _x_start = 0;
    _y_start = 1;
    _theta_start = 2;
    _v_start = 3;
    _cte_start = 4;
    _etheta_start = 5;
    _angvel_start = 6;
    _linacc_start = 7;
    _ind_inc = 8;

    mpc_x.resize(_mpc_steps);
    std::fill(mpc_x.begin(), mpc_x.end(), 0);
    mpc_y.resize(_mpc_steps);
    std::fill(mpc_y.begin(), mpc_y.end(), 0);
    mpc_theta.resize(_mpc_steps);
    std::fill(mpc_theta.begin(), mpc_theta.end(), 0);
    mpc_linvels.resize(_mpc_steps);
    std::fill(mpc_linvels.begin(), mpc_linvels.end(), 0);
    mpc_angvels.resize(_mpc_steps - 1);
    std::fill(mpc_angvels.begin(), mpc_angvels.end(), 0);
    mpc_linaccs.resize(_mpc_steps - 1);
    std::fill(mpc_linaccs.begin(), mpc_linaccs.end(), 0);
}

CBFHorizon::~CBFHorizon()
{
}

void CBFHorizon::set_dist_map(const std::shared_ptr<distmap::DistanceMap> &dist_map)
{
    dist_grid_ptr = dist_map;
    std::cout << "[Filter] Distance map set" << std::endl;
}

void CBFHorizon::LoadParams(const std::map<std::string, double> &params)
{
    _dt = params.at("DT");
    _mpc_steps = params.find("STEPS") != params.end() ? params.at("STEPS") : _mpc_steps;

    use_cbf = params.find("USE_CBF") != params.end() ? params.at("USE_CBF") : use_cbf;

    _w_pos = params.find("W_POS") != params.end() ? params.at("W_POS") : _w_pos;
    _w_vel = params.find("W_V") != params.end() ? params.at("W_V") : _w_vel;
    _w_cte = params.find("W_CTE") != params.end() ? params.at("W_CTE") : _w_cte;
    _w_etheta = params.find("W_ETHETA") != params.end() ? params.at("W_ETHETA") : _w_etheta;
    _w_angvel = params.find("W_ANGVEL") != params.end() ? params.at("W_ANGVEL") : _w_angvel;
    _w_angvel_d = params.find("W_DANGVEL") != params.end() ? params.at("W_DANGVEL") : _w_angvel_d;
    _w_linvel_d = params.find("W_DA") != params.end() ? params.at("W_DA") : _w_linvel_d;

    max_linacc = params.find("MAX_LINACC") != params.end() ? params.at("MAX_LINACC") : max_linacc;
    max_angvel = params.find("ANGVEL") != params.end() ? params.at("ANGVEL") : max_angvel;
    max_linvel = params.find("LINVEL") != params.end() ? params.at("LINVEL") : max_linvel;

    alpha = params.find("CBF_ALPHA") != params.end() ? params.at("CBF_ALPHA") : alpha;
    colinear = params.find("CBF_COLINEAR") != params.end() ? params.at("CBF_COLINEAR") : colinear;
    padding = params.find("CBF_PADDING") != params.end() ? params.at("CBF_PADDING") : padding;
    use_dynamic_alpha = params.find("CBF_DYNAMIC_ALPHA") != params.end() ? params.at("CBF_DYNAMIC_ALPHA") : use_dynamic_alpha;

    alpha_value = alpha;

    debug = params.find("DEBUG") != params.end() ? params.at("DEBUG") : debug;

    use_dynamic_alpha &= use_cbf;

    std::cout << "[Filter] Parameters loaded" << std::endl;

    // print out all _w_ params
    std::cout << "[Filter] W_POS: " << _w_pos << std::endl;
    std::cout << "[Filter] W_V: " << _w_vel << std::endl;
    std::cout << "[Filter] W_CTE: " << _w_cte << std::endl;
    std::cout << "[Filter] W_ETHETA: " << _w_etheta << std::endl;
    std::cout << "[Filter] W_ANGVEL: " << _w_angvel << std::endl;
    std::cout << "[Filter] W_DANGVEL: " << _w_angvel_d << std::endl;
    std::cout << "[Filter] W_DLINVEL: " << _w_linvel_d << std::endl;
    std::cout << "[Filter] MAX_LINACC: " << max_linacc << std::endl;
    std::cout << "[Filter] MAX_ANGVEL: " << max_angvel << std::endl;
    std::cout << "[Filter] MAX_LINVEL: " << max_linvel << std::endl;
    std::cout << "[Filter] CBF_ALPHA: " << alpha << std::endl;
    std::cout << "[Filter] CBF_COLINEAR: " << colinear << std::endl;
    std::cout << "[Filter] CBF_PADDING: " << padding << std::endl;
    std::cout << "[Filter] CBF_DYNAMIC_ALPHA: " << use_dynamic_alpha << std::endl;

    if (use_cbf)
        std::cout << termcolor::green << "[Filter] Using CBF" << termcolor::reset << std::endl;
    else
        std::cout << termcolor::red << "[Filter] Not using CBF" << termcolor::reset << std::endl;

    // _x_start = 0;
    // _y_start = _x_start + _mpc_steps;
    // _theta_start = _y_start + _mpc_steps;
    // _v_start = _theta_start + _mpc_steps;
    // _cte_start = _v_start + _mpc_steps;
    // _etheta_start = _cte_start + _mpc_steps;
    // _angvel_start = _etheta_start + _mpc_steps;
    // _linacc_start = _angvel_start + _mpc_steps - 1;
    _x_start = 0;
    _y_start = 1;
    _theta_start = 2;
    _v_start = 3;
    _cte_start = 4;
    _etheta_start = 5;
    _angvel_start = 6;
    _linacc_start = 7;
    _ind_inc = 8;

    mpc_x.resize(_mpc_steps);
    std::fill(mpc_x.begin(), mpc_x.end(), 0);
    mpc_y.resize(_mpc_steps);
    std::fill(mpc_y.begin(), mpc_y.end(), 0);
    mpc_theta.resize(_mpc_steps);
    std::fill(mpc_theta.begin(), mpc_theta.end(), 0);
    mpc_linvels.resize(_mpc_steps);
    std::fill(mpc_linvels.begin(), mpc_linvels.end(), 0);
    mpc_angvels.resize(_mpc_steps - 1);
    std::fill(mpc_angvels.begin(), mpc_angvels.end(), 0);
    mpc_linaccs.resize(_mpc_steps - 1);
    std::fill(mpc_linaccs.begin(), mpc_linaccs.end(), 0);

    if (use_dynamic_alpha)
    {
        _alpha_start = 8;
        _ind_inc = 9;
    }
}

autodiff::real CBFHorizon::eval_objective(const autodiff::ArrayXreal &x, void *data)
{
    CBFHorizon obj = *(CBFHorizon *)data;

    autodiff::real cost = 0.0;

    // std::cout << "OBJECTIVE EVALUATION" << std::endl;

    for (int i = 0; i < obj._mpc_steps; ++i)
    {
        autodiff::real ref_x = obj.reference(0, i);
        autodiff::real ref_y = obj.reference(3, i);

        // cost += obj._w_pos * (x[obj._x_start + i] - ref_x) * (x[obj._x_start + i] - ref_x);
        // cost += obj._w_pos * (x[obj._y_start + i] - ref_y) * (x[obj._y_start + i] - ref_y);

        // cost += obj._w_cte * x[obj._cte_start + i] * x[obj._cte_start + i];
        // cost += obj._w_etheta * x[obj._etheta_start + i] * x[obj._etheta_start + i];

        cost += obj._w_pos * (x[obj._x_start + obj._ind_inc * i] - ref_x) * (x[obj._x_start + obj._ind_inc * i] - ref_x);
        cost += obj._w_pos * (x[obj._y_start + obj._ind_inc * i] - ref_y) * (x[obj._y_start + obj._ind_inc * i] - ref_y);

        cost += obj._w_cte * x[obj._cte_start + obj._ind_inc * i] * x[obj._cte_start + obj._ind_inc * i];
        cost += obj._w_etheta * x[obj._etheta_start + obj._ind_inc * i] * x[obj._etheta_start + obj._ind_inc * i];
    }
    // std::cout << "OBJECTIVE EVALUATION 1" << std::endl;

    for (int i = 0; i < obj._mpc_steps - 1; ++i)
    {
        autodiff::real ref_acc_x = obj.reference(2, i);
        autodiff::real ref_acc_y = obj.reference(5, i);
        autodiff::real ref_acc = sqrt(ref_acc_x * ref_acc_x + ref_acc_y * ref_acc_y);

        cost += obj._w_angvel * x[obj._angvel_start + obj._ind_inc * i] * x[obj._angvel_start + obj._ind_inc * i];
        cost += obj._w_vel * (x[obj._linacc_start + obj._ind_inc * i] - ref_acc) * (x[obj._linacc_start + obj._ind_inc * i] - ref_acc);
    }
    // std::cout << "OBJECTIVE EVALUATION 2" << std::endl;

    for (int i = 0; i < obj._mpc_steps - 2; ++i)
    {
        cost += obj._w_angvel_d * (x[obj._angvel_start + obj._ind_inc * (i + 1)] - x[obj._angvel_start + obj._ind_inc * i]) * (x[obj._angvel_start + obj._ind_inc * (i + 1)] - x[obj._angvel_start + obj._ind_inc * i]);
        cost += obj._w_linvel_d * (x[obj._linacc_start + obj._ind_inc * (i + 1)] - x[obj._linacc_start + obj._ind_inc * i]) * (x[obj._linacc_start + obj._ind_inc * (i + 1)] - x[obj._linacc_start + obj._ind_inc * i]);
    }
    // std::cout << "OBJECTIVE EVALUATION 3" << std::endl;

    if (obj.use_dynamic_alpha)
    {
        cost += (x[obj._alpha_start] - obj.alpha) * (x[obj._alpha_start] - obj.alpha);
    }

    ((CBFHorizon *)data)->iterations++;

    return cost;
}

double CBFHorizon::objective(const std::vector<double> &x, std::vector<double> &grad, void *data)
{

    CBFHorizon obj = *(CBFHorizon *)data;
    autodiff::VectorXreal x_var(x.size());
    for (size_t i = 0; i < x.size(); ++i)
    {
        x_var[i] = x[i];
    }

    autodiff::real cost;

    if (!grad.empty())
    {
        Eigen::Map<Eigen::VectorXd> grad_map(grad.data(), grad.size());
        grad_map = autodiff::gradient(obj.eval_objective,
                                      autodiff::detail::wrt(x_var),
                                      autodiff::at(x_var, data),
                                      cost);
    }
    else
    {
        cost = obj.eval_objective(x_var, data);
    }

    return autodiff::val(cost);
}

autodiff::VectorXreal CBFHorizon::eval_cbf_constraint(const autodiff::VectorXreal &vars, void *data)
{
    CBFHorizon obj = *(CBFHorizon *)data;

    std::shared_ptr<distmap::DistanceMap> dist_grid_ptr = obj.dist_grid_ptr;

    autodiff::VectorXreal result(obj._mpc_steps - 1);

    autodiff::real alpha = obj.alpha;
    autodiff::real colinear = obj.colinear;
    autodiff::real padding = obj.padding;
    autodiff::real eps = 1e-6;

    // std::cout << "CBF EVALUATION" << std::endl;

    for (int i = 0; i < obj._mpc_steps - 1; ++i)
    {
        autodiff::real x = vars[obj._x_start + obj._ind_inc * i];
        autodiff::real y = vars[obj._y_start + obj._ind_inc * i];
        autodiff::real theta = vars[obj._theta_start + obj._ind_inc * i];
        autodiff::real v = vars[obj._v_start + obj._ind_inc * i];

        autodiff::real desired_a = vars[obj._linacc_start + obj._ind_inc * i];
        autodiff::real desired_w = vars[obj._angvel_start + obj._ind_inc * i];

        autodiff::real xdot = v * cos(theta);
        autodiff::real ydot = v * sin(theta);

        // add small value to avoid division by zero
        autodiff::real dist =
            dist_grid_ptr->atPositionSafe(autodiff::val(x), autodiff::val(y), true) + eps;

        autodiff::real D = dist - padding - eps;

        distmap::DistanceMap::Gradient grad =
            dist_grid_ptr->gradientAtPosition(autodiff::val(x), autodiff::val(y), true);

        autodiff::real dx = grad.dx;
        autodiff::real dy = grad.dy;

        autodiff::real grad_norm = sqrt(dx * dx + dy * dy);
        dx *= (-dist / grad_norm);
        dy *= (-dist / grad_norm);

        autodiff::real p0 = (dx * cos(theta) + dy * sin(theta)) / dist;
        autodiff::real pp = (-dy * cos(theta) + dx * sin(theta)) / dist;
        autodiff::real P = p0 + v * colinear;

        autodiff::real Lfh1 = (exp(-P) * xdot / dist) * (-dx + D * (cos(theta) - dx * p0 / dist));
        autodiff::real Lfh2 = (exp(-P) * ydot / dist) * (-dy + D * (sin(theta) - dy * p0 / dist));
        autodiff::real Lfh = Lfh1 + Lfh2;

        autodiff::real Lgh1 = -D * colinear * exp(-P);
        autodiff::real Lgh2 = D * pp * exp(-P);

        autodiff::real h = D * exp(-P);

        if (obj.use_dynamic_alpha)
        {
            // result[i] = -Lfh - Lgh1 * desired_a - Lgh2 * desired_w - vars[obj->_alpha_start + i] * h;
            result[i] = -Lfh - Lgh1 * desired_a - Lgh2 * desired_w - vars[obj._alpha_start] * h;
        }
        else
        {
            result[i] = -Lfh - Lgh1 * desired_a - Lgh2 * desired_w - alpha * h;
        }
    }

    // std::cout << "DONE CBF" << std::endl;

    return result;
}

void CBFHorizon::inequality_constraint(unsigned m, double *result, unsigned n, const double *x, double *grad, void *f_data)
{
    CBFHorizon obj = *(CBFHorizon *)f_data;

    autodiff::VectorXreal x_real(n);
    for (size_t i = 0; i < n; ++i)
    {
        x_real(i) = x[i];
    }

    autodiff::VectorXreal F;
    if (grad)
    {
        Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> Jx(grad, m, n);
        autodiff::jacobian(eval_cbf_constraint,
                           autodiff::detail::wrt(x_real),
                           autodiff::detail::at(x_real, f_data),
                           F, Jx);
    }
    else
        F = eval_cbf_constraint(x_real, f_data);

    for (size_t i = 0; i < m; ++i)
    {
        result[i] = autodiff::val(F(i));
    }

}

autodiff::VectorXreal CBFHorizon::eval_constraint(const autodiff::VectorXreal &x, void *data)
{
    CBFHorizon obj = *(CBFHorizon *)data;

    autodiff::VectorXreal result((int)(obj._mpc_steps * obj.state.size()));

    result[obj._x_start] = x[obj._x_start] - obj.state(0);
    result[obj._y_start] = x[obj._y_start] - obj.state(1);
    result[obj._theta_start] = x[obj._theta_start] - obj.state(2);
    result[obj._v_start] = x[obj._v_start] - obj.state(3);
    result[obj._cte_start] = x[obj._cte_start] - obj.state(4);
    result[obj._etheta_start] = x[obj._etheta_start] - obj.state(5);

    // std::cout << "DYNAMIC CONSTRAINT EVAL" << std::endl;
    for (int i = 0; i < obj._mpc_steps - 1; ++i)
    {
        autodiff::real x1 = x[obj._x_start + obj._ind_inc * (i + 1)];
        autodiff::real y1 = x[obj._y_start + obj._ind_inc * (i + 1)];
        autodiff::real theta1 = x[obj._theta_start + obj._ind_inc * (i + 1)];
        autodiff::real v1 = x[obj._v_start + obj._ind_inc * (i + 1)];
        autodiff::real cte1 = x[obj._cte_start + obj._ind_inc * (i + 1)];
        autodiff::real etheta1 = x[obj._etheta_start + obj._ind_inc * (i + 1)];

        autodiff::real x0 = x[obj._x_start + obj._ind_inc * i];
        autodiff::real y0 = x[obj._y_start + obj._ind_inc * i];
        autodiff::real theta0 = x[obj._theta_start + obj._ind_inc * i];
        autodiff::real v0 = x[obj._v_start + obj._ind_inc * i];
        autodiff::real cte0 = x[obj._cte_start + obj._ind_inc * i];
        autodiff::real etheta0 = x[obj._etheta_start + obj._ind_inc * i];

        // std::cout << "trying to get inputs" << i << std::endl;
        autodiff::real w0 = x[obj._angvel_start + obj._ind_inc * i];
        autodiff::real a0 = x[obj._linacc_start + obj._ind_inc * i];
        // std::cout << "done trying to get inputs" << i << std::endl;

        // model equations
        // std::cout << "doing results " << obj._etheta_start + (obj._ind_inc - 2) * (i + 1) << " / " << obj._mpc_steps * obj.state.size() << std::endl;
        result[obj._x_start + (obj._ind_inc - 2) * (i + 1)] = x1 - (x0 + v0 * cos(theta0) * obj._dt);
        result[obj._y_start + (obj._ind_inc - 2) * (i + 1)] = y1 - (y0 + v0 * sin(theta0) * obj._dt);
        result[obj._theta_start + (obj._ind_inc - 2) * (i + 1)] = atan2(sin(theta1 - (theta0 + w0 * obj._dt)), cos(theta1 - (theta0 + w0 * obj._dt)));
        result[obj._v_start + (obj._ind_inc - 2) * (i + 1)] = v1 - (v0 + a0 * obj._dt);
        result[obj._cte_start + (obj._ind_inc - 2) * (i + 1)] = cte1 - (cte0 + v0 * sin(etheta0) * obj._dt);
        result[obj._etheta_start + (obj._ind_inc - 2) * (i + 1)] = etheta1 - (etheta0 + (w0 - obj.traj_omgs(i)) * obj._dt);
        // std::cout << "done doing results i" << std::endl;
    }

    // std::cout << "DONE DYNAMIC CONSTRAINT" << std::endl;
    return result;
}

void CBFHorizon::multi_constraint(unsigned m, double *result, unsigned n, const double *x, double *grad, void *f_data)
{
    CBFHorizon obj = *(CBFHorizon *)f_data;

    autodiff::VectorXreal x_real(n);
    for (size_t i = 0; i < n; ++i)
    {
        x_real(i) = x[i];
    }

    autodiff::VectorXreal F;
    if (grad)
    {
        Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> Jx(grad, m, n);
        autodiff::jacobian(obj.eval_constraint,
                           autodiff::detail::wrt(x_real),
                           autodiff::detail::at(x_real, f_data),
                           F, Jx);
    }
    else
        F = obj.eval_constraint(x_real, f_data);

    for (size_t i = 0; i < m; ++i)
    {
        result[i] = autodiff::val(F(i));
    }
}

void CBFHorizon::vector_finite_difference(const double *x,
                                          double *grad,
                                          void *data,
                                          std::vector<double> (*func)(const std::vector<double> &x, void *data),
                                          unsigned m,
                                          unsigned n,
                                          double h)
{

    std::vector<double> x_copy(n);
    for (size_t i = 0; i < n; ++i)
    {
        x_copy[i] = x[i];
    }

    std::vector<double> f1;
    std::vector<double> f2;

    for (size_t i = 0; i < n; ++i)
    {
        double x1 = x_copy[i];
        x_copy[i] = x1 + h;
        f1 = func(x_copy, data);
        x_copy[i] = x1 - h;
        f2 = func(x_copy, data);
        for (size_t j = 0; j < m; j++)
        {
            grad[j * n + i] = (f1[j] - f2[j]) / (2 * h);
        }

        x_copy[i] = x1;
    }
}

void CBFHorizon::finite_difference(const std::vector<double> &x,
                                   std::vector<double> &grad,
                                   void *data,
                                   double (*func)(const std::vector<double> &x, void *data),
                                   double h)
{
    // copy the input vector
    std::vector<double> x_copy = x;
    for (size_t i = 0; i < x.size(); i++)
    {
        double x1 = x_copy[i];
        x_copy[i] = x1 + h;
        double f1 = func(x_copy, data);
        x_copy[i] = x1 - h;
        double f2 = func(x_copy, data);
        grad[i] = (f1 - f2) / (2 * h);
        x_copy[i] = x1;
    }
}

std::vector<double> CBFHorizon::Solve(const Eigen::VectorXd &state, const Eigen::MatrixXd &wpts)
{

    size_t n_vars = _mpc_steps * state.size() + 2 * (_mpc_steps - 1);

    if (use_dynamic_alpha && dist_grid_ptr.get() != nullptr)
    {
        // n_vars += _mpc_steps - 1;
        n_vars += 1;
    }

    size_t n_barrier_constraints = _mpc_steps - 1;
    // size_t n_barrier_constraints = 1;
    size_t n_dynamic_constraints = _mpc_steps * state.size();

    double x = state[0];
    double y = state[1];
    double theta = state[2];
    double v = state[3];
    double cte = state[4];
    double etheta = state[5];

    this->state = state;
    reference = wpts;

    // traj_omgs = Eigen::VectorXd::Zero(_mpc_steps);
    traj_omgs = autodiff::VectorXreal(_mpc_steps);
    for (int i = 0; i < _mpc_steps; ++i)
    {
        autodiff::real ref_vel_x = reference(1, i);
        autodiff::real ref_acc_x = reference(2, i);
        autodiff::real ref_vel_y = reference(4, i);
        autodiff::real ref_acc_y = reference(5, i);
        if (ref_vel_x * ref_vel_x + ref_vel_y * ref_vel_y < 1e-6)
            traj_omgs(i) = 0;
        else
            traj_omgs(i) = (ref_acc_y * ref_vel_x - ref_acc_x * ref_vel_y) /
                           (ref_vel_x * ref_vel_x + ref_vel_y * ref_vel_y);
    }

    desired_w = 0;
    desired_a = 0;

    nlopt::opt opt(nlopt::algorithm::LD_SLSQP, n_vars);

    std::vector<double> vars_lower_bound(n_vars);
    std::vector<double> vars_upper_bound(n_vars);

    // state bounds
    for (size_t i = 0; i < _mpc_steps; ++i)
    {
        vars_lower_bound[_x_start + _ind_inc * i] = -HUGE_VAL;
        vars_lower_bound[_y_start + _ind_inc * i] = -HUGE_VAL;
        vars_lower_bound[_theta_start + _ind_inc * i] = -HUGE_VAL;
        vars_lower_bound[_v_start + _ind_inc * i] = -max_linvel;
        vars_lower_bound[_cte_start + _ind_inc * i] = -HUGE_VAL;
        vars_lower_bound[_etheta_start + _ind_inc * i] = -HUGE_VAL;

        vars_upper_bound[_x_start + _ind_inc * i] = HUGE_VAL;
        vars_upper_bound[_y_start + _ind_inc * i] = HUGE_VAL;
        vars_upper_bound[_theta_start + _ind_inc * i] = HUGE_VAL;
        vars_upper_bound[_v_start + _ind_inc * i] = max_linvel;
        vars_upper_bound[_cte_start + _ind_inc * i] = HUGE_VAL;
        vars_upper_bound[_etheta_start + _ind_inc * i] = HUGE_VAL;
    }

    for (size_t i = 0; i < _mpc_steps - 1; ++i)
    {
        vars_lower_bound[_angvel_start + _ind_inc * i] = -max_angvel;
        vars_lower_bound[_linacc_start + _ind_inc * i] = -max_linacc;

        vars_upper_bound[_angvel_start + _ind_inc * i] = max_angvel;
        vars_upper_bound[_linacc_start + _ind_inc * i] = max_linacc;
    }

    // for (size_t i = 0; i < _angvel_start; ++i)
    // {
    //     vars_lower_bound[i] = -HUGE_VAL;
    //     vars_upper_bound[i] = HUGE_VAL;
    // }

    // for (size_t i = _theta_start; i < _v_start; ++i)
    // {
    //     vars_lower_bound[i] = -M_PI - .1;
    //     vars_upper_bound[i] = M_PI + .1;
    // }

    // velocity bounds
    // for (size_t i = _v_start; i < _cte_start; ++i)
    // {
    //     vars_lower_bound[i] = -max_linvel;
    //     vars_upper_bound[i] = max_linvel;
    // }

    // for (size_t i = _angvel_start; i < _linacc_start; ++i)
    // {
    //     vars_lower_bound[i] = -max_angvel;
    //     vars_upper_bound[i] = max_angvel;
    // }

    // for (size_t i = _linacc_start; i < n_vars; ++i)
    // {
    //     vars_lower_bound[i] = -max_linacc;
    //     vars_upper_bound[i] = max_linacc;
    // }

    if (use_dynamic_alpha && dist_grid_ptr.get() != nullptr)
    {
        // for (int i = 0; i < _mpc_steps - 1; ++i)
        // {
        //     vars_lower_bound[_alpha_start + i] = 1e-3;
        //     vars_upper_bound[_alpha_start + i] = 10;
        // }

        vars_lower_bound[_alpha_start] = 1e-3;
        vars_upper_bound[_alpha_start] = 10;
    }

    opt.set_lower_bounds(vars_lower_bound);
    opt.set_upper_bounds(vars_upper_bound);

    opt.set_min_objective(objective, this);

    // no use using cbf if distance is large
    double d = dist_grid_ptr->atPositionSafe(x, y, true);
    if (use_cbf && dist_grid_ptr.get() != nullptr && d < 100)
    {
        std::vector<double> barrier_tolerances(n_barrier_constraints, 1e-8);
        opt.add_inequality_mconstraint(inequality_constraint, this, barrier_tolerances);
    }

    std::vector<double> dynamics_tolerances(n_dynamic_constraints, 1e-8);
    opt.add_equality_mconstraint(multi_constraint, this, dynamics_tolerances);

    opt.set_xtol_rel(1e-2);
    // opt.set_ftol_abs(1e-3);
    opt.set_maxtime(autodiff::val(_dt) * .9);

    std::vector<double> x0(n_vars);

    // warm start with previous solution if available
    if (prev_x0.size() != 0)
    {
        for (size_t i = 0; i < n_vars; ++i)
        {
            x0[i] = prev_x0[i];
        }
    }
    else
    {
        // warm start with reference trajectory

        for (int i = 0; i < n_vars; ++i)
        {
            x0[i] = 0;
        }

        for (size_t i = 0; i < _mpc_steps; ++i)
        {
            x0[_x_start + _ind_inc * i] = reference(0, i);
            x0[_y_start + _ind_inc * i] = reference(3, i);
            x0[_theta_start + _ind_inc * i] = atan2(reference(4, i), reference(1, i));
            x0[_v_start + _ind_inc * i] = sqrt(pow(reference(1, i), 2) + pow(reference(4, i), 2));
        }

        for (size_t i = 0; i < _mpc_steps - 1; ++i)
        {
            x0[_linacc_start + _ind_inc * i] = sqrt(pow(reference(2, i), 2) + pow(reference(5, i), 2));
        }
    }

    iterations = 0;

    x0[_x_start] = x;
    x0[_y_start] = y;
    x0[_theta_start] = theta;
    x0[_v_start] = v;
    x0[_cte_start] = cte;
    x0[_etheta_start] = etheta;

    if (use_dynamic_alpha && dist_grid_ptr.get() != nullptr)
    {
        // for (int i = 0; i < _mpc_steps - 1; ++i)
        // {
        //     x0[_alpha_start + i] = alpha;
        // }
        x0[_alpha_start] = alpha;
    }

    // variable initialization printout
    // std::cout << "warm start:\n";
    // for (int i = 0; i < _mpc_steps; ++i)
    // {
    //     std::cout << i << "\tx: " << x0[_x_start + i] << "\ty: " << x0[_y_start + i] << "\ttheta: " << x0[_theta_start + i] << "\tv: " << x0[_v_start + i] << std::endl;
    // }

    double minf;
    std::vector<double> x0_cp = x0;
    nlopt::result result;
    try
    {

        result = opt.optimize(x0, minf);
        std::cout << "nlopt succeeded in " << iterations << " iterations" << std::endl;
        // std::cout << "cost is " << minf << std::endl;
        // std::cout << "states" << std::endl;
        // for (int i = 0; i < _mpc_steps; ++i)
        // {
        //     std::cout << i << "\tx: " << x0[_x_start + i] << "\ty: " << x0[_y_start + i] << "\ttheta: " << x0[_theta_start + i] << "\tv: " << x0[_v_start + i] << std::endl;
        // }
    }
    catch (std::exception &e)
    {

        std::cerr << termcolor::red << "nlopt failed: " << e.what() << termcolor::reset << std::endl;
        // exit(0);
        return {0, 0};
    }

    std::cout << "[Filter] done with solve" << std::endl;

    for (int i = 0; i < _mpc_steps; ++i)
    {
        mpc_x[i] = x0[_x_start + _ind_inc * i];
        mpc_y[i] = x0[_y_start + _ind_inc * i];
        mpc_theta[i] = x0[_theta_start + _ind_inc * i];
        mpc_linvels[i] = x0[_v_start + _ind_inc * i];
    }

    for (int i = 0; i < _mpc_steps - 1; ++i)
    {
        mpc_angvels[i] = x0[_angvel_start + _ind_inc * i];
        mpc_linaccs[i] = x0[_linacc_start + _ind_inc * i];
    }

    prev_x0 = x0;

    if (dist_grid_ptr.get() != nullptr)
    {
        autodiff::VectorXreal x_real(x0.size());
        for (size_t i = 0; i < x0.size(); ++i)
        {
            x_real[i] = x0[i];
        }
        h_value = autodiff::val(eval_cbf_constraint(x_real, this)[0]);
        if (h_value > 1e-6)
            std::cout << termcolor::red << "H CONSTRAINT VIOLATED" << termcolor::reset << std::endl;
    }

    if (use_dynamic_alpha)
        alpha_value = x0[_alpha_start];
    else
        alpha_value = alpha;

    return {x0[_angvel_start], x0[_linacc_start]};
}
