#include <chrono>
#include <iomanip>
#include <iostream>

#include "mpc/mpc_nlopt.h"

MPCNLOPT::MPCNLOPT()
{
    _is_pos = false;
    _dt = 0.1;
    _mpc_steps = 10;

    _x_start = 0;
    _y_start = _x_start + _mpc_steps;
    _theta_start = _y_start + _mpc_steps;
    _v_start = _theta_start + _mpc_steps;
    _cte_start = _v_start + _mpc_steps;
    _etheta_start = _cte_start + _mpc_steps;
    _angvel_start = _etheta_start + _mpc_steps;
    _linacc_start = _angvel_start + _mpc_steps - 1;

    _w_pos = 0;
    _w_vel = 0;
    _w_cte = 0;
    _w_etheta = 0;
    _w_angvel = 0;
    _w_angvel_d = 0;
    _w_linvel_d = 0;

    _ref_cte = 0;
    _ref_etheta = 0;
    _ref_vel = 0;
}

MPCNLOPT::MPCNLOPT(bool _is_pos) : _is_pos(_is_pos)
{
    _dt = 0.1;
    _mpc_steps = 10;

    _x_start = 0;
    _y_start = _x_start + _mpc_steps;
    _theta_start = _y_start + _mpc_steps;
    _v_start = _theta_start + _mpc_steps;

    if (!_is_pos)
    {
        _cte_start = _v_start + _mpc_steps;
        _etheta_start = _cte_start + _mpc_steps;
        _angvel_start = _etheta_start + _mpc_steps;
    }
    else
    {
        _angvel_start = _v_start + _mpc_steps;
    }

    _linacc_start = _angvel_start + _mpc_steps - 1;

    _w_pos = 0;
    _w_vel = 0;
    _w_cte = 0;
    _w_etheta = 0;
    _w_angvel = 0;
    _w_angvel_d = 0;
    _w_linvel_d = 0;

    _ref_cte = 0;
    _ref_etheta = 0;
    _ref_vel = 0;
}

MPCNLOPT::~MPCNLOPT()
{
}

double MPCNLOPT::evaluate_objective_gtg(const std::vector<double> &x)
{
    double cost = 0.0;
    for (int i = 0; i < _mpc_steps; ++i)
    {
        cost += _w_pos * (x[_x_start + i] - 4.0) * (x[_x_start + i] - 4.0);
        cost += _w_pos * (x[_y_start + i] - 4.0) * (x[_y_start + i] - 4.0);
    }

    for (int i = 0; i < _mpc_steps - 1; ++i)
    {
        cost += _w_angvel * x[_angvel_start + i] * x[_angvel_start + i];
        cost += _w_vel * x[_linacc_start + i] * x[_linacc_start + i];
    }

    for (int i = 0; i < _mpc_steps - 2; ++i)
    {
        cost += _w_angvel_d * (x[_angvel_start + i + 1] - x[_angvel_start + i]) * (x[_angvel_start + i + 1] - x[_angvel_start + i]);
        cost += _w_linvel_d * (x[_linacc_start + i + 1] - x[_linacc_start + i]) * (x[_linacc_start + i + 1] - x[_linacc_start + i]);
    }

    return cost;
}

void MPCNLOPT::evaluate_constraints_gtg(std::vector<double> &constraint_vals, const std::vector<double> &x)
{
    double n = 4 * _mpc_steps + (_mpc_steps - 1) * 2;
    double m = 4 * (_mpc_steps - 1);

    // initial state constraints
    constraint_vals[(_mpc_steps - 1) * 0] = x[_x_start] - _state(0);
    constraint_vals[(_mpc_steps - 1) * 1] = x[_y_start] - _state(1);
    constraint_vals[(_mpc_steps - 1) * 2] = x[_theta_start] - _state(2);
    constraint_vals[(_mpc_steps - 1) * 3] = x[_v_start] - _state(3);

    for (int i = 1; i < _mpc_steps; ++i)
    {
        double x1 = x[_x_start + i];
        double y1 = x[_y_start + i];
        double theta1 = x[_theta_start + i];
        double v1 = x[_v_start + i];

        double x0 = x[_x_start + i - 1];
        double y0 = x[_y_start + i - 1];
        double theta0 = x[_theta_start + i - 1];
        double v0 = x[_v_start + i - 1];

        double w0 = x[_angvel_start + i - 1];
        double a0 = x[_linacc_start + i - 1];

        constraint_vals[(_mpc_steps - 1) * 0 + i] = x1 - (x0 + v0 * cos(theta0) * _dt);
        constraint_vals[(_mpc_steps - 1) * 1 + i] = y1 - (y0 + v0 * sin(theta0) * _dt);
        constraint_vals[(_mpc_steps - 1) * 2 + i] = theta1 - (theta0 + w0 * _dt);
        constraint_vals[(_mpc_steps - 1) * 3 + i] = v1 - (v0 + a0 * _dt);

        // std::cout << (_v_start + i - 3) * 4 + i << " / "
        //           << constraint_vals.size() << std::endl;
    }
}

void MPCNLOPT::finite_difference_constraints_gtg(double *grad, const double *x_arr, unsigned int sz)
{
    // convert to std::vector
    std::vector<double> x(sz);
    for (int i = 0; i < sz; ++i)
    {
        x[i] = x_arr[i];
    }

    double h = 1e-6;
    double n = sz;
    double m = 4 * (_mpc_steps - 1);

    // std::vector<double> xph = x;
    // std::vector<double> xmh = x;
    std::vector<double> fph(m, 0);
    std::vector<double> fmh(m, 0);

    memset(grad, 0, m * n * sizeof(double));

    for (int i = 0; i < n; ++i)
    {
        // xph[i] += h;
        // xmh[i] -= h;
        double x_i = x[i];

        x[i] = x_i + h;
        evaluate_constraints_gtg(fph, x);
        x[i] = x_i - h;
        evaluate_constraints_gtg(fmh, x);

        for (int j = 0; j < m; ++j)
        {
            grad[(int)(j * n + i)] = (fph[j] - fmh[j]) / (2 * h);
        }

        // xph[i] = x[i];
        // xmh[i] = x[i];
        x[i] = x_i;
    }

    // return grad;
}

std::vector<double> MPCNLOPT::finite_difference_obj_gtg(const std::vector<double> &x)
{
    double h = 1e-6;
    std::vector<double> grad(x.size(), 0);
    std::vector<double> x_mod = x;

    for (int i = 0; i < x.size(); ++i)
    {
        double x_i = x[i];

        x_mod[i] = x_i + h;
        double fph = evaluate_objective_gtg(x_mod);
        x_mod[i] = x_i - h;
        double fmh = evaluate_objective_gtg(x_mod);

        grad[i] = (fph - fmh) / (2 * h);

        x_mod[i] = x_i;
    }

    return grad;
}

double MPCNLOPT::objective_gtg(const std::vector<double> &x, std::vector<double> &grad, void *f_data)
{
    MPCNLOPT obj = *(reinterpret_cast<MPCNLOPT *>(f_data));

    double cost = 0.0;
    for (int i = 0; i < obj._mpc_steps; ++i)
    {
        cost += obj._w_pos * (x[obj._x_start + i] - 4.0) * (x[obj._x_start + i] - 4.0);
        cost += obj._w_pos * (x[obj._y_start + i] - 4.0) * (x[obj._y_start + i] - 4.0);
    }

    for (int i = 0; i < obj._mpc_steps - 1; ++i)
    {
        cost += obj._w_angvel * x[obj._angvel_start + i] * x[obj._angvel_start + i];
        cost += obj._w_vel * x[obj._linacc_start + i] * x[obj._linacc_start + i];
    }

    for (int i = 0; i < obj._mpc_steps - 2; ++i)
    {
        cost += obj._w_angvel_d * (x[obj._angvel_start + i + 1] - x[obj._angvel_start + i]) * (x[obj._angvel_start + i + 1] - x[obj._angvel_start + i]);
        cost += obj._w_linvel_d * (x[obj._linacc_start + i + 1] - x[obj._linacc_start + i]) * (x[obj._linacc_start + i + 1] - x[obj._linacc_start + i]);
    }

    if (!grad.empty())
    {
        // fill with zeros
        std::fill(grad.begin(), grad.end(), 0);

        // for (int i = 0; i < obj._mpc_steps; ++i)
        // {
        //     grad[obj._x_start + i] = 2 * obj._w_pos * (x[obj._x_start + i] - 4.0);
        //     grad[obj._y_start + i] = 2 * obj._w_pos * (x[obj._y_start + i] - 4.0);
        // }

        // for (int i = 0; i < obj._mpc_steps - 1; ++i)
        // {
        //     grad[obj._angvel_start + i] = 2 * obj._w_angvel * x[obj._angvel_start + i];
        //     grad[obj._linacc_start + i] = 2 * obj._w_vel * x[obj._linacc_start + i];
        // }

        // grad[obj._angvel_start] -= 2 * obj._w_angvel_d * (x[obj._angvel_start + 1] - x[obj._angvel_start]);
        // grad[obj._linacc_start] -= 2 * obj._w_linvel_d * (x[obj._linacc_start + 1] - x[obj._linacc_start]);
        // for (int i = 1; i < obj._mpc_steps - 2; ++i)
        // {
        //     grad[obj._angvel_start + i] += 2 * obj._w_angvel_d * (x[obj._angvel_start + i] - x[obj._angvel_start + i - 1]) - 2 * obj._w_angvel_d * (x[obj._angvel_start + i + 1] - x[obj._angvel_start + i]);
        //     grad[obj._linacc_start + i] += 2 * obj._w_linvel_d * (x[obj._linacc_start + i] - x[obj._linacc_start + i - 1]) - 2 * obj._w_linvel_d * (x[obj._linacc_start + i + 1] - x[obj._linacc_start + i]);
        // }
        // grad[obj._angvel_start + obj._mpc_steps - 2] += 2 * obj._w_angvel_d * (x[obj._angvel_start + obj._mpc_steps - 2] - x[obj._angvel_start + obj._mpc_steps - 3]);
        // grad[obj._linacc_start + obj._mpc_steps - 2] += 2 * obj._w_linvel_d * (x[obj._linacc_start + obj._mpc_steps - 2] - x[obj._linacc_start + obj._mpc_steps - 3]);

        // std::vector<double> fd_grad = obj.finite_difference_obj(x);

        // for (int i = 0; i < grad.size(); ++i)
        // {
        //     assert(fabs(grad[i] - fd_grad[i]) < 1e-3);
        // }

        // time the finite difference routine
        auto start = std::chrono::high_resolution_clock::now();
        grad = obj.finite_difference_obj_gtg(x);
        auto end = std::chrono::high_resolution_clock::now();
        double time_to_solve = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();

        // std::cout << "Finite difference objective time: " << time_to_solve << std::endl;
    }

    // std::cout << "gradient size is " << grad.size() << "\n";
    // std::cout << "COST IS " << cost << "\n";

    return cost;
}

void MPCNLOPT::dynamics_constraints_gtg(unsigned int m, double *result, unsigned int n, const double *x, double *grad, void *f_data)
{
    MPCNLOPT obj = *(reinterpret_cast<MPCNLOPT *>(f_data));

    double dt = obj._dt;

    double x_tol = 1e-3;

    // initial state constraints
    result[(int)((obj._mpc_steps - 1) * 0)] = x[obj._x_start] - obj._state(0);
    result[(int)((obj._mpc_steps - 1) * 1)] = x[obj._y_start] - obj._state(1);
    result[(int)((obj._mpc_steps - 1) * 2)] = x[obj._theta_start] - obj._state(2);
    result[(int)((obj._mpc_steps - 1) * 3)] = x[obj._v_start] - obj._state(3);

    // dynamics constraints across the horizon
    for (int i = 1; i < obj._mpc_steps; ++i)
    {
        double x1 = x[obj._x_start + i];
        double y1 = x[obj._y_start + i];
        double theta1 = x[obj._theta_start + i];
        double v1 = x[obj._v_start + i];

        double x0 = x[obj._x_start + i - 1];
        double y0 = x[obj._y_start + i - 1];
        double theta0 = x[obj._theta_start + i - 1];
        double v0 = x[obj._v_start + i - 1];

        double w0 = x[obj._angvel_start + i - 1];
        double a0 = x[obj._linacc_start + i - 1];

        result[(int)((obj._mpc_steps - 1) * 0 + i)] = x1 - (x0 + v0 * cos(theta0) * dt);
        result[(int)((obj._mpc_steps - 1) * 1 + i)] = y1 - (y0 + v0 * sin(theta0) * dt);
        result[(int)((obj._mpc_steps - 1) * 2 + i)] = theta1 - (theta0 + w0 * dt);
        result[(int)((obj._mpc_steps - 1) * 3 + i)] = v1 - (v0 + a0 * dt);

        if (grad)
        {
            // x constraint
            // grad[(obj._x_start + i) * n + obj._x_start + i] = 1;
            // grad[(obj._x_start + i) * n + obj._theta_start + i] = -v0 * sin(theta0) * dt;
            // grad[(obj._x_start + i) * n + obj._v_start + i] = cos(theta0) * dt;
            // grad[(obj._x_start + i) * n + obj._x_start + i + 1] = -1;

            // // y constraint
            // grad[(obj._y_start + i - 1) * n + obj._y_start + i] = 1;
            // grad[(obj._y_start + i - 1) * n + obj._theta_start + i] = v0 * cos(theta0) * dt;
            // grad[(obj._y_start + i - 1) * n + obj._v_start + i] = sin(theta0) * dt;
            // grad[(obj._y_start + i - 1) * n + obj._y_start + i + 1] = -1;

            // // theta constraint
            // grad[(obj._theta_start + i - 2) * n + obj._theta_start + i] = 1;
            // grad[(obj._theta_start + i - 2) * n + obj._angvel_start + i] = dt;
            // grad[(obj._theta_start + i - 2) * n + obj._theta_start + i + 1] = -1;

            // // v constraint
            // grad[(obj._v_start + i - 3) * n + obj._v_start + i] = 1;
            // grad[(obj._v_start + i - 3) * n + obj._linacc_start + i] = dt;
            // grad[(obj._v_start + i - 3) * n + obj._v_start + i + 1] = -1;

            // std::vector<double> fd_constraint = obj.finite_difference_constraints(x, n);
            // std::cout << "size is " << fd_constraint.size() << "\n";
            // fd_constraint.clear();
        }
    }

    if (grad)
    {
        auto start = std::chrono::high_resolution_clock::now();
        obj.finite_difference_constraints_gtg(grad, x, n);
        auto end = std::chrono::high_resolution_clock::now();
        double time_to_solve = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();

        // std::cout << "Finite difference constraints time: " << time_to_solve << std::endl;
    }

    return;
}

double MPCNLOPT::objective(const std::vector<double> &x, std::vector<double> &grad, void *f_data)
{

    MPCNLOPT obj = *(reinterpret_cast<MPCNLOPT *>(f_data));

    double cost = 0.0;
    double term1 = 0.0;
    for (int i = 0; i < obj._mpc_steps; ++i)
    {
        double ref_x = obj._wpts(0, i);
        double ref_y = obj._wpts(3, i);

        // cost += obj._w_pos * (x[obj._x_start + i] - ref_x) * (x[obj._x_start + i] - ref_x);
        // cost += obj._w_pos * (x[obj._y_start + i] - ref_y) * (x[obj._y_start + i] - ref_y);

        // cost += obj._w_cte * x[obj._cte_start + i] * x[obj._cte_start + i];
        // cost += obj._w_etheta * x[obj._etheta_start + i] * x[obj._etheta_start + i];

        term1 += obj._w_pos * (x[obj._x_start + i] - ref_x) * (x[obj._x_start + i] - ref_x);
        term1 += obj._w_pos * (x[obj._y_start + i] - ref_y) * (x[obj._y_start + i] - ref_y);

        term1 += obj._w_cte * x[obj._cte_start + i] * x[obj._cte_start + i];
        term1 += obj._w_etheta * x[obj._etheta_start + i] * x[obj._etheta_start + i];
    }

    cost += term1;
    std::cout << "term1 is " << term1 << "\n";

    double term2 = 0.0;

    // minimize actuation use
    for (int i = 0; i < obj._mpc_steps - 1; ++i)
    {
        double ref_acc_x = obj._wpts(2, i);
        double ref_acc_y = obj._wpts(5, i);
        double ref_acc = sqrt(ref_acc_x * ref_acc_x + ref_acc_y * ref_acc_y);

        // cost += obj._w_angvel * x[obj._angvel_start + i] * x[obj._angvel_start + i];
        // cost += obj._w_vel * (x[obj._linacc_start + i] - ref_acc) * (x[obj._linacc_start + i] - ref_acc);

        term2 += obj._w_angvel * x[obj._angvel_start + i] * x[obj._angvel_start + i];
        term2 += obj._w_vel * (x[obj._linacc_start + i] - ref_acc) * (x[obj._linacc_start + i] - ref_acc);
    }

    cost += term2;
    std::cout << "term2 is " << term2 << "\n";

    double term3 = 0.0;
    // minimize value gap between sequential actuations
    for (int i = 0; i < obj._mpc_steps - 2; ++i)
    {
        // cost += obj._w_angvel_d * (x[obj._angvel_start + i + 1] - x[obj._angvel_start + i]) * (x[obj._angvel_start + i + 1] - x[obj._angvel_start + i]);
        // cost += obj._w_linvel_d * (x[obj._linacc_start + i + 1] - x[obj._linacc_start + i]) * (x[obj._linacc_start + i + 1] - x[obj._linacc_start + i]);

        term3 += obj._w_angvel_d * (x[obj._angvel_start + i + 1] - x[obj._angvel_start + i]) * (x[obj._angvel_start + i + 1] - x[obj._angvel_start + i]);
        term3 += obj._w_linvel_d * (x[obj._linacc_start + i + 1] - x[obj._linacc_start + i]) * (x[obj._linacc_start + i + 1] - x[obj._linacc_start + i]);
    }

    cost += term3;
    std::cout << "term3 is " << term3 << "\n";

    if (!grad.empty())
        grad = obj.finite_difference_obj(x);


    // print the x vector for debugging if cost is NAN
    // if (std::isnan(cost))
    // {
    //     std::cout << "x vector is: ";
    //     for (int i = 0; i < x.size(); ++i)
    //     {
    //         std::cout << x[i] << " ";
    //     }
    // }
    std::cout << "COST IS " << cost << "\n";
    if (std::isnan(cost))
        exit(-1);

    return cost;
}

double MPCNLOPT::evaluate_objective(const std::vector<double> &x)
{
    double cost = 0.0;
    for (int i = 0; i < _mpc_steps; ++i)
    {
        double ref_x = _wpts(0, i);
        double ref_y = _wpts(3, i);

        cost += _w_pos * (x[_x_start + i] - ref_x) * (x[_x_start + i] - ref_x);
        cost += _w_pos * (x[_y_start + i] - ref_y) * (x[_y_start + i] - ref_y);

        cost += _w_cte * x[_cte_start + i] * x[_cte_start + i];
        cost += _w_etheta * x[_etheta_start + i] * x[_etheta_start + i];
    }

    // minimize actuation use
    for (int i = 0; i < _mpc_steps - 1; ++i)
    {
        double ref_acc_x = _wpts(2, i);
        double ref_acc_y = _wpts(5, i);
        double ref_acc = sqrt(ref_acc_x * ref_acc_x + ref_acc_y * ref_acc_y);

        cost += _w_angvel * x[_angvel_start + i] * x[_angvel_start + i];
        cost += _w_vel * (x[_linacc_start + i] - ref_acc) * (x[_linacc_start + i] - ref_acc);
    }

    // minimize value gap between sequential actuations
    for (int i = 0; i < _mpc_steps - 2; ++i)
    {
        cost += _w_angvel_d * (x[_angvel_start + i + 1] - x[_angvel_start + i]) * (x[_angvel_start + i + 1] - x[_angvel_start + i]);
        cost += _w_linvel_d * (x[_linacc_start + i + 1] - x[_linacc_start + i]) * (x[_linacc_start + i + 1] - x[_linacc_start + i]);
    }

    return cost;
}

std::vector<double> MPCNLOPT::finite_difference_obj(const std::vector<double> &x)
{
    double h = 1e-6;
    std::vector<double> grad(x.size(), 0);
    std::vector<double> x_mod = x;

    for (int i = 0; i < x.size(); ++i)
    {
        double x_i = x[i];

        x_mod[i] = x_i + h;
        double fph = evaluate_objective(x_mod);
        x_mod[i] = x_i - h;
        double fmh = evaluate_objective(x_mod);

        grad[i] = (fph - fmh) / (2 * h);

        x_mod[i] = x_i;
    }

    return grad;
}

// using add_equality_mconstraint requires double* instead of std::vector<double>
void MPCNLOPT::dynamics_constraints(unsigned int m, double *result, unsigned int n, const double *x, double *grad, void *f_data)
{
    MPCNLOPT obj = *(reinterpret_cast<MPCNLOPT *>(f_data));

    double dt = obj._dt;

    // initial state constraints
    result[(int)((obj._mpc_steps) * 0)] = x[obj._x_start] - obj._state(0);
    result[(int)((obj._mpc_steps) * 1)] = x[obj._y_start] - obj._state(1);
    result[(int)((obj._mpc_steps) * 2)] = x[obj._theta_start] - obj._state(2);
    result[(int)((obj._mpc_steps) * 3)] = x[obj._v_start] - obj._state(3);
    result[(int)((obj._mpc_steps) * 4)] = x[obj._cte_start] - obj._state(4);
    result[(int)((obj._mpc_steps) * 5)] = x[obj._etheta_start] - obj._state(5);

    // dynamics constraints across the horizon
    for (int i = 0; i < obj._mpc_steps - 1; ++i)
    {
        double x1 = x[obj._x_start + i + 1];
        double y1 = x[obj._y_start + i + 1];
        double theta1 = x[obj._theta_start + i + 1];
        double v1 = x[obj._v_start + i + 1];
        double cte1 = x[obj._cte_start + i + 1];
        double etheta1 = x[obj._etheta_start + i + 1];

        double x0 = x[obj._x_start + i];
        double y0 = x[obj._y_start + i];
        double theta0 = x[obj._theta_start + i];
        double v0 = x[obj._v_start + i];
        double cte0 = x[obj._cte_start + i];
        double etheta0 = x[obj._etheta_start + i];

        double w0 = x[obj._angvel_start + i];
        double a0 = x[obj._linacc_start + i];

        result[(int)((obj._mpc_steps) * 0 + i + 1)] = x1 - (x0 + v0 * cos(theta0) * dt);
        result[(int)((obj._mpc_steps) * 1 + i + 1)] = y1 - (y0 + v0 * sin(theta0) * dt);
        result[(int)((obj._mpc_steps) * 2 + i + 1)] = theta1 - (theta0 + w0 * dt);
        result[(int)((obj._mpc_steps) * 3 + i + 1)] = v1 - (v0 + a0 * dt);
        result[(int)((obj._mpc_steps) * 4 + i + 1)] = cte1 - (cte0 + v0 * sin(etheta0) * dt);
        result[(int)((obj._mpc_steps) * 5 + i + 1)] = etheta1 - (etheta0 + (w0 - obj._traj_omgs(i)) * dt);
    }

    if (grad)
    {
        std::cout << "Calculating constraint gradients\n";
        obj.finite_difference_constraints(grad, x, n);

        // check if any of the gradients are NAN
        for (int i = 0; i < n * m; ++i)
        {
            if (std::isnan(grad[i]))
            {
                std::cout << "Constraint gradient is NAN at index " << i << "\n";
                break;
            }
        }
    }

    return;
}

void MPCNLOPT::evaluate_constraints(std::vector<double> &constraint_vals, const std::vector<double> &x)
{
    double n = 6 * _mpc_steps + (_mpc_steps - 1) * 2;
    double m = 6 * (_mpc_steps);

    // initial state constraints
    constraint_vals[(_mpc_steps) * 0] = x[_x_start] - _state(0);
    constraint_vals[(_mpc_steps) * 1] = x[_y_start] - _state(1);
    constraint_vals[(_mpc_steps) * 2] = x[_theta_start] - _state(2);
    constraint_vals[(_mpc_steps) * 3] = x[_v_start] - _state(3);
    constraint_vals[(_mpc_steps) * 4] = x[_cte_start] - _state(4);
    constraint_vals[(_mpc_steps) * 5] = x[_etheta_start] - _state(5);

    for (int i = 0; i < _mpc_steps - 1; ++i)
    {
        double x1 = x[_x_start + i + 1];
        double y1 = x[_y_start + i + 1];
        double theta1 = x[_theta_start + i + 1];
        double v1 = x[_v_start + i + 1];
        double cte1 = x[_cte_start + i + 1];
        double etheta1 = x[_etheta_start + i + 1];

        double x0 = x[_x_start + i];
        double y0 = x[_y_start + i];
        double theta0 = x[_theta_start + i];
        double v0 = x[_v_start + i];
        double cte0 = x[_cte_start + i];
        double etheta0 = x[_etheta_start + i];

        double w0 = x[_angvel_start + i];
        double a0 = x[_linacc_start + i];

        constraint_vals[(_mpc_steps) * 0 + i + 1] = x1 - (x0 + v0 * cos(theta0) * _dt);
        constraint_vals[(_mpc_steps) * 1 + i + 1] = y1 - (y0 + v0 * sin(theta0) * _dt);
        constraint_vals[(_mpc_steps) * 2 + i + 1] = theta1 - (theta0 + w0 * _dt);
        constraint_vals[(_mpc_steps) * 3 + i + 1] = v1 - (v0 + a0 * _dt);
        constraint_vals[(_mpc_steps) * 4 + i + 1] = cte1 - (cte0 + v0 * sin(etheta0) * _dt);
        constraint_vals[(_mpc_steps) * 5 + i + 1] = etheta1 - (etheta0 + (w0 - _traj_omgs(i)) * _dt);

    }
    
    return;
}

void MPCNLOPT::finite_difference_constraints(double *grad, const double *x_arr, unsigned int sz)
{
    // convert to std::vector
    std::vector<double> x(sz);
    for (int i = 0; i < sz; ++i)
    {
        x[i] = x_arr[i];
    }

    double h = 1e-6;
    double n = sz;
    double m = 6 * (_mpc_steps);

    std::vector<double> fph(m, 0);
    std::vector<double> fmh(m, 0);

    // memset(grad, 0, m * n * sizeof(double));

    for (int i = 0; i < n; ++i)
    {
        double x_i = x[i];

        x[i] = x_i + h;
        evaluate_constraints(fph, x);
        x[i] = x_i - h;
        evaluate_constraints(fmh, x);

        for (int j = 0; j < m; ++j)
        {
            grad[(int)(j * n + i)] = (fph[j] - fmh[j]) / (2 * h);
        }

        x[i] = x_i;
    }
}

void MPCNLOPT::updateGoal(const Eigen::Vector3d &goalPose)
{
}

std::vector<double> MPCNLOPT::Solve(const Eigen::VectorXd &state)
{
    return {};
}

void MPCNLOPT::LoadParams(const std::map<std::string, double> &params)
{
    _params = params;

    // Init parameters for MPCNLOPT object
    _dt = params.find("DT") != params.end() ? params.at("DT") : _dt;
    _max_angvel = _params.find("ANGVEL") != _params.end() ? _params.at("ANGVEL") : _max_angvel;
    _max_linvel = _params.find("LINVEL") != _params.end() ? _params.at("LINVEL") : _max_linvel;
    _bound_value = _params.find("BOUND") != _params.end() ? _params.at("BOUND") : _bound_value;
    _mpc_steps = params.find("STEPS") != params.end() ? params.at("STEPS") : _mpc_steps;

    _w_pos = params.find("W_POS") != params.end() ? params.at("W_POS") : _w_pos;
    _w_cte = params.find("W_CTE") != params.end() ? params.at("W_CTE") : _w_cte;
    _w_etheta = params.find("W_ETHETA") != params.end() ? params.at("W_ETHETA") : _w_etheta;
    _w_vel = params.find("W_V") != params.end() ? params.at("W_V") : _w_vel;
    _w_angvel = params.find("W_ANGVEL") != params.end() ? params.at("W_ANGVEL") : _w_angvel;
    _w_angvel_d = params.find("W_DANGVEL") != params.end() ? params.at("W_DANGVEL") : _w_angvel_d;
    _w_linvel_d = params.find("W_DA") != params.end() ? params.at("W_DA") : _w_linvel_d;

    // print all parameters
    std::cout << "DT: " << _dt << std::endl;
    std::cout << "STEPS: " << _mpc_steps << std::endl;
    std::cout << "W_POS: " << _w_pos << std::endl;
    std::cout << "W_CTE: " << _w_cte << std::endl;
    std::cout << "W_ETHETA: " << _w_etheta << std::endl;
    std::cout << "W_V: " << _w_vel << std::endl;
    std::cout << "W_ANGVEL: " << _w_angvel << std::endl;
    std::cout << "W_DANGVEL: " << _w_angvel_d << std::endl;
    std::cout << "W_DA: " << _w_linvel_d << std::endl;

    // std::cerr << "max linvel is " << _max_linvel << std::endl;

    _x_start = 0;
    _y_start = _x_start + _mpc_steps;
    _theta_start = _y_start + _mpc_steps;
    _v_start = _theta_start + _mpc_steps;

    if (!_is_pos)
    {
        _cte_start = _v_start + _mpc_steps;
        _etheta_start = _cte_start + _mpc_steps;
        _angvel_start = _etheta_start + _mpc_steps;
    }
    else
    {
        _angvel_start = _v_start + _mpc_steps;
    }

    _linacc_start = _angvel_start + _mpc_steps - 1;

    // print out the start indices
    std::cout << "x start: " << _x_start << std::endl;
    std::cout << "y start: " << _y_start << std::endl;
    std::cout << "theta start: " << _theta_start << std::endl;
    std::cout << "v start: " << _v_start << std::endl;
    std::cout << "cte start: " << _cte_start << std::endl;
    std::cout << "etheta start: " << _etheta_start << std::endl;
    std::cout << "angvel start: " << _angvel_start << std::endl;
    std::cout << "linacc start: " << _linacc_start << std::endl;

    std::cout << "\n!! MPCNLOPT Obj parameters updated !! " << std::endl;
}

std::vector<double> MPCNLOPT::Solve(const Eigen::VectorXd &state, const Eigen::MatrixXd &wpts)
{
    size_t n_vars = _mpc_steps * state.size() + (_mpc_steps - 1) * 2;
    size_t n_constraints = _mpc_steps * state.size();

    double x = state(0);
    double y = state(1);
    double theta = state(2);
    double v = std::min(_max_linvel, std::max(0.0, state(3)));
    double cte = state(4);
    double etheta = state(5);

    _wpts = wpts;
    _state = state;
    _traj_omgs = Eigen::VectorXd::Zero(_mpc_steps);
    for (int i = 0; i < _mpc_steps; ++i)
    {
        double ref_vel_x = wpts(1, i);
        double ref_acc_x = wpts(2, i);
        double ref_vel_y = wpts(4, i);
        double ref_acc_y = wpts(5, i);
        if (ref_vel_x * ref_vel_x + ref_vel_y * ref_vel_y < 1e-8)
            _traj_omgs(i) = 0;
        else
            _traj_omgs(i) = (ref_acc_y * ref_vel_x - ref_acc_x * ref_vel_y) /
                            (pow(ref_vel_x, 2) + pow(ref_vel_y, 2));
    }

    // Requires gradients
    nlopt::opt opt(nlopt::algorithm::LD_SLSQP, n_vars);

    std::vector<double> vars_lower_bound(n_vars);
    std::vector<double> vars_upper_bound(n_vars);

    for (int i = 0; i < _angvel_start; ++i)
    {
        vars_lower_bound[i] = -_bound_value;
        vars_upper_bound[i] = _bound_value;
    }

    for (int i = _v_start; i < _cte_start; ++i)
    {
        vars_lower_bound[i] = 0;
        vars_upper_bound[i] = _max_linvel + .01;
    }

    for (int i = _angvel_start; i < _linacc_start; ++i)
    {
        vars_lower_bound[i] = -_max_angvel;
        vars_upper_bound[i] = _max_angvel;
    }

    for (int i = _linacc_start; i < n_vars; ++i)
    {
        vars_lower_bound[i] = -2 * _max_linvel;
        vars_upper_bound[i] = 2 * _max_linvel;
    }

    opt.set_lower_bounds(vars_lower_bound);
    opt.set_upper_bounds(vars_upper_bound);

    opt.set_min_objective(MPCNLOPT::objective, this);

    std::vector<double> tolerances(state.size(), 1e-4);
    opt.add_equality_mconstraint(MPCNLOPT::dynamics_constraints, this, tolerances);
    opt.set_xtol_rel(1e-4);
    opt.set_maxtime(.5);

    std::vector<double> xi(n_vars, 0);
    xi[_x_start] = x;
    xi[_y_start] = y;
    xi[_theta_start] = theta;
    xi[_v_start] = v;
    xi[_cte_start] = cte;
    xi[_etheta_start] = etheta;

    double minf;

    try
    {
        nlopt::result result = opt.optimize(xi, minf);
        std::cout << "found minimum at f(x)= "
                  << std::setprecision(10) << minf << std::endl;
    }
    catch (std::exception &e)
    {
        std::cerr << "nlopt failed: " << e.what() << std::endl;
    }

    std::cout << "finished making optimizer" << std::endl;

    mpc_x = {};
    mpc_y = {};
    mpc_theta = {};
    mpc_linvels = {};

    for (int i = 0; i < _mpc_steps; ++i)
    {
        mpc_x.push_back(xi[_x_start + i]);
        mpc_y.push_back(xi[_y_start + i]);
        mpc_theta.push_back(xi[_theta_start + i]);
        mpc_linvels.push_back(xi[_v_start + i]);
    }

    mpc_angvels = {};
    mpc_linaccs = {};

    for (int i = 0; i < _mpc_steps - 1; ++i)
    {
        mpc_angvels.push_back(xi[_angvel_start + i]);
        mpc_linaccs.push_back(xi[_linacc_start + i]);
    }

    std::vector<double> ret;
    ret.push_back(xi[_angvel_start]);
    ret.push_back(xi[_linacc_start]);

    return ret;
}

std::vector<double> MPCNLOPT::Solve_gtg(const Eigen::VectorXd &state)
{
    size_t n_vars = _mpc_steps * state.size() + (_mpc_steps - 1) * 2;
    size_t n_constraints = (_mpc_steps) * state.size();

    double x = state(0);
    double y = state(1);
    double theta = state(2);
    double v = std::min(_max_linvel, std::max(0.0, state(3)));

    _state = state;

    // Requires gradients
    nlopt::opt opt(nlopt::algorithm::LD_SLSQP, n_vars);

    std::vector<double> vars_lower_bound(n_vars);
    std::vector<double> vars_upper_bound(n_vars);

    for (int i = 0; i < _angvel_start; ++i)
    {
        vars_lower_bound[i] = -HUGE_VAL;
        vars_upper_bound[i] = HUGE_VAL;
    }

    for (int i = _v_start; i < _v_start + _mpc_steps; ++i)
    {
        vars_lower_bound[i] = 0;
        vars_upper_bound[i] = _max_linvel + .01;
    }

    for (int i = _angvel_start; i < _linacc_start; ++i)
    {
        vars_lower_bound[i] = -_max_angvel;
        vars_upper_bound[i] = _max_angvel;
    }

    for (int i = _linacc_start; i < n_vars; ++i)
    {
        vars_lower_bound[i] = -2 * _max_linvel;
        vars_upper_bound[i] = 2 * _max_linvel;
    }

    opt.set_lower_bounds(vars_lower_bound);
    opt.set_upper_bounds(vars_upper_bound);

    opt.set_min_objective(MPCNLOPT::objective_gtg, this);

    std::vector<double> tolerances(n_constraints, 1e-4);
    opt.add_equality_mconstraint(MPCNLOPT::dynamics_constraints_gtg, this, tolerances);
    opt.set_ftol_rel(1e-4);
    opt.set_xtol_rel(1e-6);
    opt.set_maxtime(.5);

    std::vector<double> xi(n_vars, 0);
    xi[_x_start] = x;
    xi[_y_start] = y;
    xi[_theta_start] = theta;
    xi[_v_start] = v;

    double minf;

    try
    {
        nlopt::result result = opt.optimize(xi, minf);
        std::cout << "found minimum at f(x)= "
                  << std::setprecision(10) << minf << std::endl;

        std::cout << "result is " << result << std::endl;
    }
    catch (std::exception &e)
    {
        std::cerr << "nlopt failed: " << e.what() << std::endl;
    }

    mpc_x = {};
    mpc_y = {};
    mpc_theta = {};
    mpc_linvels = {};

    for (int i = 0; i < _mpc_steps; ++i)
    {
        mpc_x.push_back(xi[_x_start + i]);
        mpc_y.push_back(xi[_y_start + i]);
        mpc_theta.push_back(xi[_theta_start + i]);
        mpc_linvels.push_back(xi[_v_start + i]);
    }

    mpc_angvels = {};
    mpc_linaccs = {};

    for (int i = 0; i < _mpc_steps - 1; ++i)
    {
        mpc_angvels.push_back(xi[_angvel_start + i]);
        mpc_linaccs.push_back(xi[_linacc_start + i]);
    }

    // print positions in horizon in readable format
    // std::cout << "Horizon values: " << std::endl;
    // for (int i = 0; i < _mpc_steps; ++i)
    // {
    //     std::cout << "x: " << mpc_x[i] << ", y: " << mpc_y[i] << ", theta: " << mpc_theta[i] << ", v: " << mpc_linvels[i] << std::endl;
    //     std::cout << "angvel: " << mpc_angvels[i] << ", linacc: " << mpc_linaccs[i] << std::endl;
    // }

    exit(0);

    std::vector<double> ret;
    ret.push_back(xi[_angvel_start]);
    ret.push_back(xi[_linacc_start]);

    return ret;
}
