#pragma once

#include "mpc_base.h"

class MPC : public MPCBase{
public:
	MPC();

    std::vector<double> mpc_x;
    std::vector<double> mpc_y;
    std::vector<double> mpc_theta;
    std::vector<double> mpc_linvels;
    std::vector<double> mpc_angvels;

    void LoadParams(const std::map<std::string, double> &params) override;
	std::vector<double> Solve(const Eigen::VectorXd &state) override;
    std::vector<double> Solve(const Eigen::VectorXd &state, const Eigen::MatrixXd &wpts) override;
    void updateGoal(const Eigen::Vector3d& goalPose) override;

protected:
	// Parameters for mpc solver
    int _cte_start, _etheta_start;
};
