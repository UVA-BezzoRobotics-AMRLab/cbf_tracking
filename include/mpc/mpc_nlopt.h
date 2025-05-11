#pragma once

#include <map>
#include <vector>
#include <nlopt.hpp>
#include <Eigen/Dense>

#include "mpc/mpc_base.h"

class MPCNLOPT : public MPCBase
{
public:
    MPCNLOPT();
    MPCNLOPT(bool is_pos);
    ~MPCNLOPT();

    void updateGoal(const Eigen::Vector3d &goalPose) override;
    std::vector<double> Solve(const Eigen::VectorXd &state) override;
    std::vector<double> Solve(const Eigen::VectorXd &state, const Eigen::MatrixXd &wpts) override;
    std::vector<double> Solve_gtg(const Eigen::VectorXd &state);
    void LoadParams(const std::map<std::string, double> &params) override;

    std::vector<double> finite_difference_obj_gtg(const std::vector<double> &x);
    void finite_difference_constraints_gtg(double *grad, const double *x_arr, unsigned int sz);

    std::vector<double> finite_difference_obj(const std::vector<double> &x);
    void finite_difference_constraints(double *grad, const double *x_arr, unsigned int sz);

protected:
    double _dt;
    double _mpc_steps;
    double _max_angvel;
    double _max_linvel;
    double _bound_value;

    double _w_pos;
    double _w_vel;
    double _w_cte;
    double _w_etheta;
    double _w_angvel;
    double _w_angvel_d;
    double _w_linvel_d;

    double _ref_cte;
    double _ref_etheta;
    double _ref_vel;

    int _x_start;
    int _y_start;
    int _theta_start;
    int _v_start;
    int _cte_start;
    int _etheta_start;
    int _angvel_start;
    int _linacc_start;

    bool _is_pos;

    std::map<std::string, double> _params;

    Eigen::MatrixXd _wpts;
    Eigen::VectorXd _state;
    Eigen::VectorXd _traj_omgs;

    std::vector<double> mpc_x;
    std::vector<double> mpc_y;
    std::vector<double> mpc_theta;
    std::vector<double> mpc_linvels;

    std::vector<double> mpc_angvels;
    std::vector<double> mpc_linaccs;

    static double objective(const std::vector<double> &x, std::vector<double> &grad, void *f_data);
    static void dynamics_constraints(unsigned int m, double *result, unsigned int n, const double *x, double *grad, void *f_data);

    static double objective_gtg(const std::vector<double> &x, std::vector<double> &grad, void *f_data);
    static void dynamics_constraints_gtg(unsigned int m, double *result, unsigned int n, const double *x, double *grad, void *f_data);

    double evaluate_objective_gtg(const std::vector<double> &x);
    void evaluate_constraints_gtg(std::vector<double> &constraint_vals, const std::vector<double> &x);

    double evaluate_objective(const std::vector<double> &x);
    void evaluate_constraints(std::vector<double> &constraint_vals, const std::vector<double> &x);
    
};
