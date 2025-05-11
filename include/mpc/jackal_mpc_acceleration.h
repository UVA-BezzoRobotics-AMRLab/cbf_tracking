#pragma once

#include "mpc/mpc_base.h"

#include <distance_map_core/distance_map_converter_base.h>
#include <distance_map_core/distance_map_converter_instantiater.h>

class MPC_ACC : public MPCBase
{
public:
    MPC_ACC();

    std::vector<double> mpc_x;
    std::vector<double> mpc_y;
    std::vector<double> mpc_theta;
    std::vector<double> mpc_linvels;
    std::vector<double> mpc_ctes;
    std::vector<double> mpc_ethetas;
    std::vector<double> mpc_linaccs;
    std::vector<double> mpc_angvels;

    void LoadParams(const std::map<std::string, double> &params) override;
    std::vector<double> Solve(const Eigen::VectorXd &state) override;
    std::vector<double> Solve(const Eigen::VectorXd &state, const Eigen::MatrixXd &wpts) override;
    void updateGoal(const Eigen::Vector3d &goalPose) override;
    void updateObstacle(const Eigen::Vector3d &obstacle);

    std::vector<double> evaluateHFunction();

    void set_dist_map(const std::shared_ptr<distmap::DistanceMap> &dist_map);

protected:
    // Parameters for mpc solver
    int _cte_start, _etheta_start, _linacc_start;

    Eigen::Vector3d _obstacle;

    double alpha;
    double padding;
    double colinear;
    
    bool use_cbf;

    std::shared_ptr<distmap::DistanceMap> _dist_grid_ptr;
};
