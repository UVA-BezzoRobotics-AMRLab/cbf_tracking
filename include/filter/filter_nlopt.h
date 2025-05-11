#pragma once

#ifdef FOUND_PYBIND11
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#endif

#include <vector>
#include <nlopt.hpp>
#include <Eigen/Dense>

#include <distance_map_core/distance_map_converter_base.h>
#include <distance_map_core/distance_map_converter_instantiater.h>

class CBFFilterNLOPT
{
public:
    CBFFilterNLOPT();
    ~CBFFilterNLOPT();

    std::vector<double> Solve(const Eigen::Ref<Eigen::VectorXd> &state, const Eigen::Ref<Eigen::VectorXd> &desired_input);
    void set_dist_map(const std::shared_ptr<distmap::DistanceMap> &dist_map);
    void load_params(const std::map<std::string, double> &params);

    #ifdef FOUND_PYBIND11
    bool set_dist_map(pybind11::dict grid_dict);
    #endif

protected:
    static double objective(const std::vector<double> &x, std::vector<double> &grad, void *data);
    static double constraint(const std::vector<double> &x, std::vector<double> &grad, void *data);

    static void multi_constraint(unsigned m, double *result, unsigned n, const double *x, double* grad, void* f_data);

    static double eval_objective(const std::vector<double> &x, void *data);
    static double eval_constraint(const std::vector<double> &x, void *data);

    void finite_difference(const std::vector<double> &x,
                           std::vector<double> &grad,
                           void *data,
                           double (*func)(const std::vector<double> &x, void *data),
                           double h = 1e-5);

    double desired_w;
    double desired_a;

    // params
    double dt;

    double max_linacc;
    double max_angvel;

    double alpha;
    double colinear;
    double padding;

    Eigen::VectorXd state;
    Eigen::VectorXd obstacle;

    std::shared_ptr<distmap::DistanceMap> dist_grid_ptr;
};
