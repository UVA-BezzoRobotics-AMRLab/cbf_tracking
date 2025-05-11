#ifndef GUROBI_SOLVER_H
#define GUROBI_SOLVER_H

#include <vector>
#include <Eigen/Core>
#include "gurobi_c++.h"

class SolverCallback : public GRBCallback {
public:
    bool _terminate;
    SolverCallback(){
        _terminate = false;
    }

protected:
    void cb(){
        if(_terminate)
            GRBCallback::abort();
    }
};

class TrajectorySolver{
public:
    TrajectorySolver();
    ~TrajectorySolver();

    bool solve();

    void setVariables();
    void setObjective();
    void generateTrajectory();
    void setDynamicConstraints();
    void setBasisConverter();
    void setInterpTime(double interp_time);
    void setFinalState(const Eigen::Matrix3d& PVA);
    void setInitialState(const Eigen::Matrix3d& PVA);
    void setPolys(const std::vector<Eigen::MatrixX4d>& hPolys);


private:

    // To save computation on pointer dereferences I will initialize here to keep m static
    GRBEnv* _env = new GRBEnv();
    GRBModel _m = GRBModel(*_env);

    std::vector<Eigen::MatrixX4d> _hPolys;
    std::vector<Eigen::Matrix4d> bs2mv_matrices, bs2be_matrices;

    std::vector<std::vector<GRBVar> > _xs;

    // number of segments for trajectory. For now it is just the number of polygons
    int _N;

    bool forceFinalVA, _use_minvo;

    double _dt; // time allocation per segment
    double _v_max;
    double _a_max;
    double _j_max;
    double _interp_time;

    std::vector<GRBConstr> _final_conditions;
    std::vector<GRBConstr> _max_constraints;
    std::vector<GRBConstr> _poly_constraints;
    std::vector<GRBConstr> _initial_conditions;
    std::vector<GRBConstr> _dynamic_constraints;


    GRBLinExpr getPos(int segment, double t, int axis);
    GRBLinExpr getVel(int segment, double t, int axis);
    GRBLinExpr getAcc(int segment, double t, int axis);
    GRBLinExpr getJerk(int segment, double t, int axis);

    GRBLinExpr getA(int segment, int axis);
    GRBLinExpr getB(int segment, int axis);
    GRBLinExpr getC(int segment, int axis);
    GRBLinExpr getD(int segment, int axis);

    GRBLinExpr getAn(int segment, int axis);
    GRBLinExpr getBn(int segment, int axis);
    GRBLinExpr getCn(int segment, int axis);
    GRBLinExpr getDn(int segment, int axis);

    std::vector<GRBLinExpr> getCP0(int segment);
    std::vector<GRBLinExpr> getCP1(int segment);
    std::vector<GRBLinExpr> getCP2(int segment);
    std::vector<GRBLinExpr> getCP3(int segment);

    Eigen::Matrix4d M_pos_bs2mv_seg0, M_pos_bs2mv_seg1, M_pos_bs2mv_rest, M_pos_bs2mv_seg_last2,
    M_pos_bs2mv_seg_last;
    Eigen::Matrix4d M_pos_bs2be_seg0, M_pos_bs2be_seg1, M_pos_bs2be_rest, M_pos_bs2be_seg_last2,
    M_pos_bs2be_seg_last;

    Eigen::Matrix4d A_be, A_mv;

};

#endif
