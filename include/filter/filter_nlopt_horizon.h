#pragma once

#include <vector>
#include <nlopt.hpp>
#include <Eigen/Dense>

#include "mpc/mpc_base.h"

// backward
#include <autodiff/reverse/var.hpp>
#include <autodiff/reverse/var/eigen.hpp>

// forward
#include <autodiff/forward/real.hpp>
#include <autodiff/forward/real/eigen.hpp>

#include <distance_map_core/distance_map_converter_base.h>
#include <distance_map_core/distance_map_converter_instantiater.h>

class CBFHorizon : public MPCBase
{
public:
    CBFHorizon();
    ~CBFHorizon();

    std::vector<double> Solve(const Eigen::VectorXd &state, const Eigen::MatrixXd &wpts);
    std::vector<double> Solve(const Eigen::VectorXd &state) {return {};}
    void updateGoal(const Eigen::Vector3d &goalPose) {}

    void set_dist_map(const std::shared_ptr<distmap::DistanceMap> &dist_map);
    void LoadParams(const std::map<std::string, double> &params);
    
    std::vector<double> mpc_linaccs;

    double h_value;
    double alpha_value;

protected:
    static double objective(const std::vector<double> &x, std::vector<double> &grad, void *data);
    static double constraint(const std::vector<double> &x, std::vector<double> &grad, void *data);

    static void multi_constraint(unsigned m, double *result, unsigned n, const double *x, double *grad, void *f_data);
    static void inequality_constraint(unsigned m, double *result, unsigned n, const double *x, double *grad, void *f_data);

    // objective
    static autodiff::real eval_objective(const autodiff::ArrayXreal& x, void *data);

    // constraints
    // static double eval_h_func(const std::vector<double> &x, void *data);
    // static std::vector<double> eval_cbf_constraint(const std::vector<double> &vars, void *data);
    static autodiff::VectorXreal eval_cbf_constraint(const autodiff::VectorXreal &vars, void *data);
    static autodiff::VectorXreal eval_constraint(const autodiff::VectorXreal &x, void *data);

    void vector_finite_difference(const double *x,
                                  double *grad,
                                  void *data,
                                  std::vector<double> (*func)(const std::vector<double> &x, void *data),
                                  unsigned m,
                                  unsigned n,
                                  double h = 1e-5);

    void finite_difference(const std::vector<double> &x,
                           std::vector<double> &grad,
                           void *data,
                           double (*func)(const std::vector<double> &x, void *data),
                           double h = 1e-5);

    double desired_w;
    double desired_a;

    int _mpc_steps;

    int _ind_inc;

    // params
    autodiff::real _dt;

    double max_linacc;
    double max_angvel;
    double max_linvel;

    double alpha;
    double colinear;
    double padding;

    double _x_start;
    double _y_start;
    double _theta_start;
    double _v_start;
    double _cte_start;
    double _etheta_start;
    double _angvel_start;
    double _linacc_start;
    double _alpha_start;

    autodiff::real _w_pos;
    autodiff::real _w_vel;
    autodiff::real _w_cte;
    autodiff::real _w_etheta;
    autodiff::real _w_angvel;
    autodiff::real _w_angvel_d;
    autodiff::real _w_linvel_d;

    Eigen::VectorXd state;
    Eigen::MatrixXd reference;

    autodiff::VectorXreal traj_omgs;

    int iterations;

    bool debug;
    bool use_cbf;
    bool use_dynamic_alpha;

    std::vector<double> prev_x0;


    std::shared_ptr<distmap::DistanceMap> dist_grid_ptr;
};
