#include <ros/ros.h>
#include <Eigen/Core>
#include <boost/bind.hpp>

#include <trajectory_msgs/JointTrajectory.h>

#include <faster/solver.hpp>

#include <robust_fast_navigation/utils.h>
#include <robust_fast_navigation/corridor.h>
#include <cbf_tracking/SolveFromState.h>

class solver_service
{
public:
    solver_service(ros::NodeHandle &nh)
    {

        double limits[3] = {1.2, 1.2, 4};

        solver.setN(6);
        solver.createVars();
        solver.setDC(.05);
        solver.setBounds(limits);
        solver.setForceFinalConstraint(true);
        solver.setFactorInitialAndFinalAndIncrement(1, 10, 1.0);
        solver.setThreads(0);
        solver.setWMax(4.0);
        solver.setVerbose(0);

        service = nh.advertiseService("solve_from_state", &solver_service::solveFromSolverStateService, this);
    }

    ~solver_service()
    {
    }


    void msgToCorridor(
        std::vector<Eigen::MatrixX4d> &hPolys,
        const geometry_msgs::PoseArray &msg)
    {

        hPolys.clear();
        Eigen::MatrixX4d currPoly;
        for (int i = 0; i < msg.poses.size(); ++i)
        {
            geometry_msgs::Pose p = msg.poses[i];
            if (p.orientation.x == 0 && p.orientation.y == 0 && p.orientation.z == 0 && p.orientation.w == 0)
            {
                if (currPoly.rows() > 0)
                {
                    hPolys.push_back(currPoly);
                    currPoly.resize(0, 4);
                }
            }
            else
            {
                Eigen::Vector4d plane(p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w);
                currPoly.conservativeResize(currPoly.rows() + 1, 4);
                currPoly.row(currPoly.rows() - 1) = plane;
            }
        }
    }
    bool solveFromSolverStateService(
        cbf_tracking::SolveFromState::Request &req,
        cbf_tracking::SolveFromState::Response &res)
    {

        ROS_INFO("solving from solver state");

        cbf_tracking::SolverState solver_state = req.state;

        std::vector<Eigen::MatrixX4d> polys;
        msgToCorridor(polys, solver_state.polys);

        // ROS_INFO_STREAM(polys[0]);
        state initialState;
        state finalState;

        initialState.setPos(solver_state.initialPVA.positions[0], solver_state.initialPVA.positions[1], solver_state.initialPVA.positions[2]);
        initialState.setVel(solver_state.initialPVA.velocities[0], solver_state.initialPVA.velocities[1], solver_state.initialPVA.velocities[2]);
        initialState.setAccel(solver_state.initialPVA.accelerations[0], solver_state.initialPVA.accelerations[1], solver_state.initialPVA.accelerations[2]);
        initialState.setJerk(0, 0, 0);

        finalState.setPos(solver_state.finalPVA.positions[0], solver_state.finalPVA.positions[1], solver_state.finalPVA.positions[2]);
        finalState.setVel(solver_state.finalPVA.velocities[0], solver_state.finalPVA.velocities[1], solver_state.finalPVA.velocities[2]);
        finalState.setAccel(solver_state.finalPVA.accelerations[0], solver_state.finalPVA.accelerations[1], solver_state.initialPVA.accelerations[2]);
        finalState.setJerk(0, 0, 0);

        solver.setX0(initialState);
        solver.setXf(finalState);
        solver.setPolytopes(polys);

        res.success = solver.genNewTraj();

        if (res.success == 1)
        {
            solver.fillX();

            trajectory_msgs::JointTrajectory traj = convertTrajToMsg(solver.X_temp_, .05, "");

            for (int interval = 0; interval < solver.N_; ++interval)
            {
                std::vector<GRBLinExpr> cp0 = solver.getCP0(interval);
                std::vector<GRBLinExpr> cp1 = solver.getCP1(interval);
                std::vector<GRBLinExpr> cp2 = solver.getCP2(interval);
                std::vector<GRBLinExpr> cp3 = solver.getCP3(interval);

                geometry_msgs::Point p;
                p.x = cp0[0].getValue();
                p.y = cp0[1].getValue();
                p.z = cp0[2].getValue();
                res.control_pts.push_back(p);

                p.x = cp1[0].getValue();
                p.y = cp1[1].getValue();
                p.z = cp1[2].getValue();
                res.control_pts.push_back(p);

                p.x = cp2[0].getValue();
                p.y = cp2[1].getValue();
                p.z = cp2[2].getValue();
                res.control_pts.push_back(p);

                p.x = cp3[0].getValue();
                p.y = cp3[1].getValue();
                p.z = cp3[2].getValue();
                res.control_pts.push_back(p);
            }
        }

        if (res.success == 0)
        {
            bool is_in = corridor::isInPoly(polys[0],
                                  Eigen::Vector2d(
                                      solver_state.initialPVA.positions[0],
                                      solver_state.initialPVA.positions[1]));

            ROS_INFO("init state is in poly: %d", is_in);
        }

        return true;
    }

private:
    SolverGurobi solver;
    ros::ServiceServer service;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "solve_from_state_server");
    ros::NodeHandle nh;

    solver_service solver_service(nh);

    ros::spin();
    return 0;
}
