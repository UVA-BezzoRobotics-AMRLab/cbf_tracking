/* ----------------------------------------------------------------------------
 * Copyright 2020, Jesus Tordesillas Torres, Aerospace Controls Laboratory
 * Massachusetts Institute of Technology
 * All Rights Reserved
 * Authors: Jesus Tordesillas, et al.
 * See LICENSE file for the license information
 * -------------------------------------------------------------------------- */

#ifndef SOLVER_GUROBI_HPP
#define SOLVER_GUROBI_HPP
#include <Eigen/Dense>
#include "gurobi_c++.h"
#include <sstream>
#include <Eigen/Dense>
#include <type_traits>
#include <fstream>
#include <faster/termcolor.hpp>

#include <unsupported/Eigen/Polynomials>
#include "faster_types.hpp"
#include <ginac/ginac.h>

using namespace termcolor;
using namespace GiNaC;

// TODO: This function is the same as solvePolyOrder2 but with other name (weird conflicts...)
inline double solvePolynomialOrder2(Eigen::Vector3f& coeff)
{
  // std::cout << "solving\n" << coeff.transpose() << std::endl;
  double a = coeff(0);
  double b = coeff(1);
  double c = coeff(2);
  double dis = b * b - 4 * a * c;
  if (dis >= 0)
  {
    double x1 = (-b - sqrt(dis)) / (2 * a);  // x1 will always be smaller than x2
    double x2 = (-b + sqrt(dis)) / (2 * a);

    if (x1 >= 0)
    {
      return x1;
    }
    if (x2 >= 0)
    {
      return x2;
    }
  }
  printf("No solution found to the equation\n");
  return std::numeric_limits<float>::max();
}

class mycallback : public GRBCallback
{
public:
  bool should_terminate_;
  mycallback()
  {
    should_terminate_ = false;
  }

  mycallback(double timeout_time)
  {
    this->timeout_time = timeout_time;
    should_terminate_ = false;
  }
  // void abortar();
  void set_timeout(double timeout_time){
    this->timeout_time = timeout_time;
    should_terminate_ = false;
  }

protected:
  void callback() override
  {  // This function is called periodically along the optimization process.
    //  It is called several times more after terminating the program
    if (where == GRB_CB_MIPNODE)
    {
      double elapsed_time = this->getDoubleInfo(GRB_CB_RUNTIME);
      if (elapsed_time > timeout_time)
      {
        printf("Timeout reached\n");
        should_terminate_ = true;
        GRBCallback::abort();  // This function only does effect when inside the function callback() of this class
      }
    }
  }
  

  double timeout_time;
};

class BasisConverter{
public:
    BasisConverter(){
        // Digits = 20;

        // initialize the minvo position basis matrix on the interval [-1,1]
        A_minvo_pos = Eigen::MatrixXd(4,4);
        A_minvo_pos <<  -0.43020000000000,    0.45680000000000,   -0.02700000000000,    0.00040000000000,
                    0.83490000000000,   -0.45680000000000,   -0.79210000000000,    0.49960000000000,
                    -0.83490000000000,   -0.45680000000000,    0.79210000000000,    0.49960000000000,
                    0.43020000000000,    0.45680000000000,    0.02700000000000,    0.00040000000000;

        A_minvo_vel = Eigen::MatrixXd(3,3);
        A_minvo_vel <<  0.3750,   -0.4330,  0.1250,
                        -0.7500,   0,       0.7500,
                        0.3750,    0.4330,  0.1250;
        
        // roots of lambda polynomial
        roots_lambda = {
            Eigen::Vector2d(0.0309000000000, 1.000000000000),
            Eigen::Vector2d(-1.000000000000, 0.7735000000000),
            Eigen::Vector2d(-0.7735000000000, 1.000000000000),
            Eigen::Vector2d(-1.000000000000, -0.0309000000000)
        };

    }

    Eigen::MatrixXd get_pos_bernstein_on_interval(const std::vector<double>& int2){

        std::vector<double> int1 = {0,1};
        
        // Initialize GiNaC symbols
        symbol t("t");

        // Define transformation from interval 1 to interval 2
        ex u = (int1[1]-int1[0])/(int2[1]-int2[0])*(t-int2[0])+int1[0];

        // Define Bernstein basis polynomials
        ex B1 = -(u - 1) * (u - 1) * (u - 1);
        ex B2 = 3 * u * (u - 1) * (u - 1);
        ex B3 = -3 * u * u * (u - 1);
        ex B4 = u * u * u;

        ex B1_poly = B1.expand();
        ex B2_poly = B2.expand();
        ex B3_poly = B3.expand();
        ex B4_poly = B4.expand();


        std::vector<ex> B_coeffs = {B1_poly, B2_poly, B3_poly, B4_poly};
        Eigen::MatrixXd A_conv(4,4);

        for(int i = 0; i < B_coeffs.size(); ++i){
            ex B = B_coeffs[i];
            for (int deg = 3; deg >= 0; --deg)
                A_conv(i,3-deg) = ex_to<numeric>(B.coeff(t,deg)).to_double();
        }

        return A_conv;
    }

    Eigen::MatrixXd get_vel_bernstein_on_interval(const std::vector<double>& int2){

        std::vector<double> int1 = {0,1};
        
        // Initialize GiNaC symbols
        symbol t("t");

        // Define transformation from interval 1 to interval 2
        ex u = (int1[1]-int1[0])/(int2[1]-int2[0])*(t-int2[0])+int1[0];

        // Define Bernstein basis polynomials
        ex B1 = (u-1)*(u-1);
        ex B2 = -2*u*(u-1);
        ex B3 = u*u;

        ex B1_poly = B1.expand();
        ex B2_poly = B2.expand();
        ex B3_poly = B3.expand();


        std::vector<ex> B_coeffs = {B1_poly, B2_poly, B3_poly};
        Eigen::MatrixXd A_conv(3,3);

        for(int i = 0; i < B_coeffs.size(); ++i){
            ex B = B_coeffs[i];
            for (int deg = 2; deg >= 0; --deg)
                A_conv(i,2-deg) = ex_to<numeric>(B.coeff(t,deg)).to_double();
        }

        return A_conv;
    }

    Eigen::MatrixXd get_pos_minvo_on_interval(const std::vector<double>& int2){
        std::vector<double> int1 = {-1,1};

        symbol t("t");
        ex u = (int1[1]-int1[0])/(int2[1]-int2[0])*(t-int2[0])+int1[0];

        std::vector<ex> T = {
            u*u*u,
            u*u,
            u,
            1
        };

        Eigen::MatrixXd A_conv(4,4);
        std::vector<ex> polynomials;
        for (int row = 0; row < A_minvo_pos.rows(); ++row){
            polynomials.push_back(0);
            for(int i = 0; i < T.size(); ++i)
                polynomials[row] += A_minvo_pos(row,i)*T[i];

            for(int deg = 3; deg >= 0; --deg)
                A_conv(row,3-deg) = ex_to<numeric>(polynomials[row].expand().coeff(t,deg)).to_double();
        }

        return A_conv;
    }


    Eigen::MatrixXd get_vel_minvo_on_interval(const std::vector<double>& int2){
        std::vector<double> int1 = {-1,1};

        symbol t("t");
        ex u = (int1[1]-int1[0])/(int2[1]-int2[0])*(t-int2[0])+int1[0];

        std::vector<ex> T = {
            u*u,
            u,
            1
        };


        Eigen::MatrixXd A_conv(3,3);
        std::vector<ex> polynomials;
        for (int row = 0; row < A_minvo_vel.rows(); ++row){
            polynomials.push_back(0);
            for(int i = 0; i < T.size(); ++i)
                polynomials[row] += A_minvo_vel(row,i)*T[i];

            for(int deg = 2; deg >= 0; --deg)
                A_conv(row,2-deg) = ex_to<numeric>(polynomials[row].expand().coeff(t,deg)).to_double();
        }

        return A_conv;
    }


protected:
    Eigen::MatrixXd A_minvo_pos, A_minvo_vel;
    std::vector<Eigen::Vector2d> roots_lambda;
};


class SolverGurobi
{
public:
  SolverGurobi();

  // void setQ(double q);
  void setN(int N);
  void setOcc(const Eigen::Vector3d& point);
  void setX0(state& data);
  // void set_u0(double u0[]);
  void setXf(state& data);
  void resetX();
  void setBounds(double max_values[3]);
  bool genNewTraj();
  bool callOptimizer();
  double getDTInitial();

  void setDC(double dc);
  void setPolytopes(const std::vector<Eigen::MatrixX4d>& polytopes);
  void setPolytopesConstraints();
  void findDT(double factor);
  void fillX();
  void setObjective();
  void setOcclusionConstraint();
  void setConstraintsXf();
  void setConstraintsX0();
  void setUseMinvo(bool use_minvo);
  void setOcc(const Eigen::MatrixXd& point);
  void setDynamicConstraints();
  void setForceFinalConstraint(bool forceFinalConstraint);
  void setMaxSolverTime(double max_solver_time);

  // For the jackal
  void setWMax(double w_max);
  bool isWmaxSatisfied();

  void setMaxConstraints();
  void createVars();
  void setThreads(int threads);
  void setVerbose(int verbose);

  void StopExecution();
  void ResetToNormalState();


  void setDistanceConstraints();

  void setMode(int mode);
  void setFactorInitialAndFinalAndIncrement(double factor_initial, double factor_final, double factor_increment);

  GRBLinExpr getPos(int t, double tau, int ii);
  GRBLinExpr getVel(int t, double tau, int ii);
  GRBLinExpr getAccel(int t, double tau, int ii);
  GRBLinExpr getJerk(int t, double tau, int ii);

  GRBLinExpr getA(int t, int ii);
  GRBLinExpr getB(int t, int ii);
  GRBLinExpr getC(int t, int ii);
  GRBLinExpr getD(int t, int ii);

  // Getters of the Normalized coefficients
  GRBLinExpr getAn(int t, int ii);
  GRBLinExpr getBn(int t, int ii);
  GRBLinExpr getCn(int t, int ii);
  GRBLinExpr getDn(int t, int ii);

  std::vector<GRBLinExpr> getCP0(int t);
  std::vector<GRBLinExpr> getCP1(int t);
  std::vector<GRBLinExpr> getCP2(int t);
  std::vector<GRBLinExpr> getCP3(int t);

  GRBModel getModel() const;

  std::vector<state> X_temp_;
  double dt_;  // time step found by the solver
  int trials_ = 0;
  int temporal_ = 0;
  double runtime_s_ = 0;
  double factor_that_worked_ = 0;
  double max_solver_time = 0;
  int N_ = 10;
  mycallback cb_;

protected:
  double cost_;

  double xf_[3 * 3];
  double x0_[3 * 3];
  double v_max_;
  double a_max_;
  double j_max_;
  double DC;
  // double q_;  // weight to the 2nd term in the cost function
  double** x_;
  double** u_;

  int N_of_polytopes_ = 3;

  BasisConverter basis_converter_;

  Eigen::MatrixXd A_minvo_, A_bezier_;

  GRBEnv* env = new GRBEnv();
  GRBModel m = GRBModel(*env);

  std::vector<GRBConstr> at_least_1_pol_cons;  // Constraints at least in one polytope
  std::vector<GRBGenConstr> polytopes_cons;    // Used for the whole trajectory
  std::vector<GRBConstr> polytope_cons;        // Used for the rescue path
  std::vector<GRBConstr> dyn_cons;
  std::vector<GRBConstr> init_cons;
  std::vector<GRBConstr> final_cons;

  std::vector<GRBQConstr> occ_cons;

  std::vector<GRBQConstr> distances_cons;

  std::vector<std::vector<GRBVar>> b;  // binary variables
  std::vector<std::vector<GRBVar>> x;
  std::vector<std::vector<GRBVar>> u;
  
  Eigen::MatrixXd occ_point;

  std::vector<double> dist_near_obs_;
  std::vector<Eigen::MatrixX4d> polytopes_;

  std::ofstream times_log;

  int mode_;
  bool forceFinalConstraint_ = true;
  bool is_occ_ = false;
  bool use_minvo_ = false;
  double factor_initial_ = 2;
  double factor_final_ = 2;
  double factor_increment_ = 2;

  int total_not_solved = 0;
  double w_max_ = 1;
};

#endif
