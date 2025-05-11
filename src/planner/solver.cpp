/* ----------------------------------------------------------------------------
 * Copyright 2020, Jesus Tordesillas Torres, Aerospace Controls Laboratory
 * Massachusetts Institute of Technology
 * All Rights Reserved
 * Authors: Jesus Tordesillas, et al.
 * See LICENSE file for the license information
 * -------------------------------------------------------------------------- */

#include <faster/solver.hpp>
#include <faster/solver_utils.hpp>
#include <chrono>
#include <unistd.h>
#include <ros/package.h>



void SolverGurobi::StopExecution()
{
  cb_.should_terminate_ = true;
  std::cout << "Activated flag to stop execution" << std::endl;
}

void SolverGurobi::ResetToNormalState()
{
  cb_.should_terminate_ = false;
}

SolverGurobi::SolverGurobi()
{
  std::cout << "In the Gurobi Constructor\n";

  v_max_ = 5;
  a_max_ = 3;
  j_max_ = 5;
  max_solver_time = .2;
  // N_ = 10;  // Segments: 0,1,...,N_-1

  m.set(GRB_StringAttr_ModelName, "planning");
  m.setCallback(&cb_);  // The callback will be called periodically along the optimization

}

void SolverGurobi::setMaxSolverTime(double max_solver_time)
{
  this->max_solver_time = max_solver_time;
}

void SolverGurobi::setN(int N)
{
  N_ = N;
}

void SolverGurobi::setMode(int mode)
{
  mode_ = mode;
}

void SolverGurobi::createVars()
{
  std::vector<std::string> coeff = { "ax", "ay", "az", "bx", "by", "bz", "cx", "cy", "cz", "dx", "dy", "dz" };

  // Variables: Coefficients of the polynomials
  for (int t = 0; t < N_; t++)
  {
    std::vector<GRBVar> row_t;
    for (int i = 0; i < 12; i++)
    {
      row_t.push_back(m.addVar(-GRB_INFINITY, GRB_INFINITY, 0, GRB_CONTINUOUS, coeff[i] + std::to_string(t)));
    }
    x.push_back(row_t);
  }
}

void SolverGurobi::setObjective()  // I need to set it every time, because the objective depends on the xFinal
{
  GRBQuadExpr control_cost = 0;
  
  double gamma = 1.4;
  double factor = 1.0;
  double radius_2 = 1.5*1.5;

  for (int t = 0; t < N_; t++)
  {
    std::vector<GRBLinExpr> ut = { getJerk(t, 0, 0), getJerk(t, 0, 1), getJerk(t, 0, 2) };

    control_cost = control_cost + GetNorm2(ut);

    if (is_occ_){
      
      Eigen::Vector3d p = occ_point.row(0);

      std::vector<GRBLinExpr> cp0 = getCP0(t);  // Control Point 0
      std::vector<GRBLinExpr> cp1 = getCP1(t);  // Control Point 1
      std::vector<GRBLinExpr> cp2 = getCP2(t);  // Control Point 2
      std::vector<GRBLinExpr> cp3 = getCP3(t);  // Control Point 3


      std::vector<GRBLinExpr> cp0_r = {cp0[0]-p[0], cp0[1]-p[1], cp0[2]-p[2]};
      std::vector<GRBLinExpr> cp1_r = {cp1[0]-p[0], cp1[1]-p[1], cp1[2]-p[2]};
      std::vector<GRBLinExpr> cp2_r = {cp2[0]-p[0], cp2[1]-p[1], cp2[2]-p[2]};
      std::vector<GRBLinExpr> cp3_r = {cp3[0]-p[0], cp3[1]-p[1], cp3[2]-p[2]};

      GRBQuadExpr cp0_r_norm = radius_2 - GetNorm2(cp0_r);
      GRBQuadExpr cp1_r_norm = radius_2 - GetNorm2(cp1_r);
      GRBQuadExpr cp2_r_norm = radius_2 - GetNorm2(cp2_r);
      GRBQuadExpr cp3_r_norm = radius_2 - GetNorm2(cp3_r);
      
      // control_cost = control_cost + cp0_r_norm*cp0_r_norm
      //                             + cp1_r_norm*cp1_r_norm;
      //                             + cp2_r_norm*cp2_r_norm;
      //                             + cp3_r_norm*cp3_r_norm;
      
      // control_cost = control_cost + cp0_r_norm
      //                             + cp1_r_norm
      //                             + cp2_r_norm
      //                             + cp3_r_norm;


      // GRBLinExpr dot = getVel(t, dt_, 0)*(occ_point(0,0)-occ_point(1,0))+
      //                  getVel(t, dt_, 1)*(occ_point(0,1)-occ_point(1,1))+
      //                  getVel(t, dt_, 2)*(occ_point(0,2)-occ_point(1,2));

      // control_cost = control_cost + factor*dot*dot;
      // factor *= gamma;
    }

  }
  // m.setObjective(control_cost + final_state_cost + distance_to_JPS_cost, GRB_MINIMIZE);
  m.setObjective(control_cost, GRB_MINIMIZE);
}

void SolverGurobi::fillX()
{
  double t = 0;
  int interval = 0;
  //#pragma omp parallel for
  //  {

  for (int i = 0; i < X_temp_.size(); i++)
  {
    t = t + DC;
    if (t > dt_ * (interval + 1))
    {
      interval = std::min(interval + 1, N_ - 1);
    }

    double posx = getPos(interval, t - interval * dt_, 0).getValue();
    double posy = getPos(interval, t - interval * dt_, 1).getValue();
    double posz = getPos(interval, t - interval * dt_, 2).getValue();

    double velx = getVel(interval, t - interval * dt_, 0).getValue();
    double vely = getVel(interval, t - interval * dt_, 1).getValue();
    double velz = getVel(interval, t - interval * dt_, 2).getValue();

    double accelx = getAccel(interval, t - interval * dt_, 0).getValue();
    double accely = getAccel(interval, t - interval * dt_, 1).getValue();
    double accelz = getAccel(interval, t - interval * dt_, 2).getValue();

    double jerkx = getJerk(interval, t - interval * dt_, 0).getValue();
    double jerky = getJerk(interval, t - interval * dt_, 1).getValue();
    double jerkz = getJerk(interval, t - interval * dt_, 2).getValue();

    state state_i;
    state_i.setPos(posx, posy, posz);
    state_i.setVel(velx, vely, velz);
    state_i.setAccel(accelx, accely, accelz);
    state_i.setJerk(jerkx, jerky, jerkz);
    state_i.setT(t);
    X_temp_[i] = (state_i);
  }

  // }  // End pragma parallel

  // Force the final input to be 0 (I'll keep applying this input if when I arrive to the final state I still
  // haven't planned again).
  X_temp_[X_temp_.size() - 1].vel = Eigen::Vector3d::Zero().transpose();
  X_temp_[X_temp_.size() - 1].accel = Eigen::Vector3d::Zero().transpose();
  X_temp_[X_temp_.size() - 1].jerk = Eigen::Vector3d::Zero().transpose();
}

void SolverGurobi::setForceFinalConstraint(bool forceFinalConstraint)
{
  forceFinalConstraint_ = forceFinalConstraint;
}

void SolverGurobi::setPolytopes(const std::vector<Eigen::MatrixX4d>& polytopes)
{
  polytopes_ = polytopes;
  std::cout << polytopes.size() << std::endl;
}

void SolverGurobi::setPolytopesConstraints()
{
  // std::cout << "Setting POLYTOPES=" << polytopes_cons.size() << std::endl;

  // Remove previous polytopes constraints
  for (int i = 0; i < polytopes_cons.size(); i++)
  {
    m.remove(polytopes_cons[i]);
  }
  polytopes_cons.clear();

  // Remove previous polytopes constraints
  for (int i = 0; i < polytope_cons.size(); i++)
  {
    m.remove(polytope_cons[i]);
  }

  polytope_cons.clear();

  // Remove previous at_least_1_pol_cons constraints
  for (int i = 0; i < at_least_1_pol_cons.size(); i++)
  {
    m.remove(at_least_1_pol_cons[i]);
  }

  at_least_1_pol_cons.clear();

  // Remove previous binary variables  (They depend on the number of polytopes--> I can't reuse them)
  for (int i = 0; i < b.size(); i++)
  {
    for (int j = 0; j < b[i].size(); j++)
    {
      m.remove(b[i][j]);
    }
  }
  b.clear();

  if (polytopes_.size() > 0)  // If there are polytope constraints
  {
    // Declare binary variables
    for (int t = 0; t < N_ + 1; t++)
    {
      std::vector<GRBVar> row;
      for (int i = 0; i < polytopes_.size(); i++)  // For all the polytopes
      {
        GRBVar variable =
            m.addVar(-GRB_INFINITY, GRB_INFINITY, 0, GRB_BINARY, "s" + std::to_string(i) + "_" + std::to_string(t));
        row.push_back(variable);
      }
      b.push_back(row);
    }

    // std::cout << "NUMBER OF POLYTOPES=" << polytopes_.size() << std::endl;
    // std::cout << "NUMBER OF FACES of the first polytope=" << polytopes_[0].A().rows() << std::endl;

    Eigen::MatrixXd minvo_inv;
    std::vector<std::vector<double>> minvo_inv_std;
    std::vector<std::vector<double>> bezier_std;

    if (use_minvo_){
      A_minvo_ = basis_converter_.get_pos_bernstein_on_interval({0, dt_});
      A_bezier_ = basis_converter_.get_pos_minvo_on_interval({0, dt_});

      minvo_inv = A_minvo_.inverse();
      minvo_inv_std = eigenMatrix2std(minvo_inv);
      bezier_std = eigenMatrix2std(A_bezier_);
    }

    // Polytope constraints (if binary_varible==1 --> In that polytope) and at_least_1_pol_cons (at least one polytope)
    // constraints
    for (int t = 0; t < N_; t++)  // Start in t=1 (because t=0 is already fixed with the initial condition)
    {
      // std::cout << "*********************t= " << t << std::endl;

      GRBLinExpr sum = 0;
      for (int col = 0; col < b[0].size(); col++)
      {
        sum = sum + b[t][col];
      }
      at_least_1_pol_cons.push_back(m.addConstr(sum == 1, "At_least_1_pol_t_" + std::to_string(t)));  // at least in
                                                                                                      // one polytope

      std::vector<GRBLinExpr> cp0 = getCP0(t);  // Control Point 0
      std::vector<GRBLinExpr> cp1 = getCP1(t);  // Control Point 1
      std::vector<GRBLinExpr> cp2 = getCP2(t);  // Control Point 2
      std::vector<GRBLinExpr> cp3 = getCP3(t);  // Control Point 3


      if (use_minvo_){
        std::vector<std::vector<GRBLinExpr>> cps;
        for (int i = 0; i < cp0.size(); i++)
        {
          cps.push_back({
            cp0[i],
            cp1[i],
            cp2[i],
            cp3[i],
          });
        }

        std::vector<std::vector<GRBLinExpr>> cps_minvo = Multiply2Matrices(Multiply2Matrices(cps, bezier_std), minvo_inv_std);
        cp0 = {cps_minvo[0][0], cps_minvo[1][0], cps_minvo[2][0]};
        cp1 = {cps_minvo[0][1], cps_minvo[1][1], cps_minvo[2][1]};
        cp2 = {cps_minvo[0][2], cps_minvo[1][2], cps_minvo[2][2]};
        cp3 = {cps_minvo[0][3], cps_minvo[1][3], cps_minvo[2][3]};
      }

      for (int n_poly = 0; n_poly < polytopes_.size(); n_poly++)  // Loop over the number of polytopes
      {
        // Constraint A1x<=b1
        int cols = polytopes_[n_poly].cols();
        Eigen::MatrixXd A1 = polytopes_[n_poly].leftCols(cols-1);
        Eigen::VectorXd bb = -polytopes_[n_poly].rightCols(1);

        std::vector<std::vector<double>> A1std = eigenMatrix2std(A1);

        // std::cout << "Before multip for n_poly=" << n_poly << std::endl;
        std::vector<GRBLinExpr> Acp0 = MatrixMultiply(A1std, cp0);  // A times control point 0
        std::vector<GRBLinExpr> Acp1 = MatrixMultiply(A1std, cp1);  // A times control point 1
        std::vector<GRBLinExpr> Acp2 = MatrixMultiply(A1std, cp2);  // A times control point 2
        std::vector<GRBLinExpr> Acp3 = MatrixMultiply(A1std, cp3);  // A times control point 3

        for (int i = 0; i < bb.rows(); i++)
        {
          std::string name0 =
              "Poly" + std::to_string(n_poly) + "_face" + std::to_string(i) + "_t" + std::to_string(t) + "_cp0";
          std::string name1 =
              "Poly" + std::to_string(n_poly) + "_face" + std::to_string(i) + "_t" + std::to_string(t) + "_cp1";
          std::string name2 =
              "Poly" + std::to_string(n_poly) + "_face" + std::to_string(i) + "_t" + std::to_string(t) + "_cp2";
          std::string name3 =
              "Poly" + std::to_string(n_poly) + "_face" + std::to_string(i) + "_t" + std::to_string(t) + "_cp3";
          // std::cout << "Plane=" << i << "out of" << bb.rows() - 1 << std::endl;
          // If b[t,0]==1, all the control points are in that polytope

          polytopes_cons.push_back(m.addGenConstrIndicator(b[t][n_poly], 1, Acp0[i], GRB_LESS_EQUAL, bb[i], name0));
          polytopes_cons.push_back(m.addGenConstrIndicator(b[t][n_poly], 1, Acp1[i], GRB_LESS_EQUAL, bb[i], name1));
          polytopes_cons.push_back(m.addGenConstrIndicator(b[t][n_poly], 1, Acp2[i], GRB_LESS_EQUAL, bb[i], name2));
          polytopes_cons.push_back(m.addGenConstrIndicator(b[t][n_poly], 1, Acp3[i], GRB_LESS_EQUAL, bb[i], name3));
        }
      }
    }
  }
}

void SolverGurobi::setDC(double dc)
{
  DC = dc;
}

void SolverGurobi::setX0(state& data)
{
  Eigen::Vector3d pos = data.pos;
  Eigen::Vector3d vel = data.vel;
  Eigen::Vector3d accel = data.accel;

  x0_[0] = data.pos.x();
  x0_[1] = data.pos.y();
  x0_[2] = data.pos.z();
  x0_[3] = data.vel.x();
  x0_[4] = data.vel.y();
  x0_[5] = data.vel.z();
  x0_[6] = data.accel.x();
  x0_[7] = data.accel.y();
  x0_[8] = data.accel.z();
}

void SolverGurobi::setXf(state& data)
{
  Eigen::Vector3d pos = data.pos;
  Eigen::Vector3d vel = data.vel;
  Eigen::Vector3d accel = data.accel;

  xf_[0] = data.pos.x();
  xf_[1] = data.pos.y();
  xf_[2] = data.pos.z();
  xf_[3] = data.vel.x();
  xf_[4] = data.vel.y();
  xf_[5] = data.vel.z();
  xf_[6] = data.accel.x();
  xf_[7] = data.accel.y();
  xf_[8] = data.accel.z();
}

void SolverGurobi::setConstraintsXf()
{
  // Remove previous final constraints
  for (int i = 0; i < final_cons.size(); i++)
  {
    m.remove(final_cons[i]);
  }

  final_cons.clear();

  // Constraint xT==x_final
  for (int i = 0; i < 3; i++)
  {
    if (forceFinalConstraint_ == true)
    {
      // std::cout << "*********FORCING FINAL CONSTRAINT******" << std::endl;
      // std::cout << xf_[i] << std::endl;
      final_cons.push_back(m.addConstr(getPos(N_ - 1, dt_, i) - xf_[i] == 0,
                                       "FinalPosAxis_" + std::to_string(i)));  // Final position
    }
    final_cons.push_back(m.addConstr(getVel(N_ - 1, dt_, i) - xf_[i + 3] == 0,
                                     "FinalVelAxis_" + std::to_string(i)));  // Final velocity
    final_cons.push_back(m.addConstr(getAccel(N_ - 1, dt_, i) - xf_[i + 6] == 0,
                                     "FinalAccel_" + std::to_string(i)));  // Final acceleration
  }
}

void SolverGurobi::setUseMinvo(bool use_minvo)
{
  use_minvo_ = use_minvo;
}

void SolverGurobi::setConstraintsX0()
{
  // Remove previous initial constraints
  for (int i = 0; i < init_cons.size(); i++)
  {
    m.remove(init_cons[i]);
  }

  init_cons.clear();
  // Constraint x0==x_initial
  for (int i = 0; i < 3; i++)
  {
    init_cons.push_back(m.addConstr(getPos(0, 0, i) == x0_[i],
                                    "InitialPosAxis_" + std::to_string(i)));  // Initial position
                                                                              // std::cout << "Velocity" << std::endl;
    init_cons.push_back(m.addConstr(getVel(0, 0, i) == x0_[i + 3],
                                    "InitialVelAxis_" + std::to_string(i)));  // Initial velocity
    // std::cout << "Accel" << std::endl;
    init_cons.push_back(m.addConstr(getAccel(0, 0, i) == x0_[i + 6],
                                    "InitialAccelAxis_" + std::to_string(i)));  // Initial acceleration}
  }
}

void SolverGurobi::resetX()
{
  int size = (int)(N_)*dt_ / DC;
  size = (size < 2) ? 2 : size;  // force size to be at least 2
  std::vector<state> tmp(size);
  X_temp_ = tmp;
}

void SolverGurobi::setMaxConstraints()
{

  Eigen::MatrixXd A_minvo_vel_ = basis_converter_.get_vel_bernstein_on_interval({0, dt_});
  Eigen::MatrixXd A_bezier_vel_ = basis_converter_.get_vel_minvo_on_interval({0, dt_});

  Eigen::MatrixXd minvo_vel_inv = A_minvo_vel_.inverse();
  std::vector<std::vector<double>> minvo_vel_inv_std = eigenMatrix2std(minvo_vel_inv);
  std::vector<std::vector<double>> bezier_vel_std = eigenMatrix2std(A_bezier_vel_);

  // Constraint v<=vmax, a<=amax, u<=umax
  for (int t = 0; t < N_; t++)
  {
    // get control points for segment
    std::vector<GRBLinExpr> cp0 = getCP0(t);
    std::vector<GRBLinExpr> cp1 = getCP1(t);
    std::vector<GRBLinExpr> cp2 = getCP2(t);
    std::vector<GRBLinExpr> cp3 = getCP3(t);

    // get control points for velocity curve of segment
    std::vector<std::vector<GRBLinExpr>> cvs;
    for(int i = 0; i < 3; i++){
      cvs.push_back({
        3*(cp1[i] - cp0[i])/dt_,
        3*(cp2[i] - cp1[i])/dt_,
        3*(cp3[i] - cp2[i])/dt_
      });
    }
    
    std::vector<std::vector<GRBLinExpr>> cvs_minvo = Multiply2Matrices(Multiply2Matrices(cvs, bezier_vel_std), minvo_vel_inv_std);
    // std::vector<GRBLinExpr> cv0_minvo = {cps_minvo[0][0], cps_minvo[1][0], cps_minvo[2][0]};
    // std::vector<GRBLinExpr> cv1_minvo = {cps_minvo[0][1], cps_minvo[1][1], cps_minvo[2][1]};
    // std::vector<GRBLinExpr> cv2_minvo = {cps_minvo[0][2], cps_minvo[1][2], cps_minvo[2][2]};

    // add max vel constraints on minvo points
    // for(int cp = 0; cp < 3; ++cp){
    //   for(int axis = 0; axis < 3; ++axis){
    //     m.addConstr(cvs_minvo[axis][cp] <= v_max_, "MaxVel_t" + std::to_string(t) + "_axis_" + std::to_string(axis) + "_cp_" + std::to_string(cp));
    //     m.addConstr(cvs_minvo[axis][cp] >= -v_max_, "MinVel_t" + std::to_string(t) + "_axis_" + std::to_string(axis) + "_cp_" + std::to_string(cp));
    //   }
    // }

    for (int i = 0; i < 3; i++)
    {
      m.addConstr(getVel(t, 0, i) <= v_max_, "MaxVel_t" + std::to_string(t) + "_axis_" + std::to_string(i));
      m.addConstr(getVel(t, 0, i) >= -v_max_, "MinVel_t" + std::to_string(t) + "_axis_" + std::to_string(i));

      m.addConstr(getAccel(t, 0, i) <= a_max_, "MaxAccel_t" + std::to_string(t) + "_axis_" + std::to_string(i));
      m.addConstr(getAccel(t, 0, i) >= -a_max_, "MinAccel_t" + std::to_string(t) + "_axis_" + std::to_string(i));

      m.addConstr(getJerk(t, 0, i) <= j_max_, "MaxJerk_t" + std::to_string(t) + "_axis_" + std::to_string(i));
      m.addConstr(getJerk(t, 0, i) >= -j_max_, "MinJerk_t" + std::to_string(t) + "_axis_" + std::to_string(i));
    }
  }
}

void SolverGurobi::setBounds(double max_values[3])
{
  v_max_ = max_values[0];
  a_max_ = max_values[1];
  j_max_ = max_values[2];

  // setMaxConstraints();
}

void SolverGurobi::setFactorInitialAndFinalAndIncrement(double factor_initial, double factor_final,
                                                        double factor_increment)
{
  factor_initial_ = factor_initial;
  factor_final_ = factor_final;
  factor_increment_ = factor_increment;
}

void SolverGurobi::setOcclusionConstraint(){
  
  // Remove previous occlusion constraints
  for(int i = 0; i < occ_cons.size(); i++){
    m.remove(occ_cons[i]);
  }
  occ_cons.clear();

  if (!is_occ_)
    return;

  Eigen::Vector3d p = occ_point.row(0);

  for(int t = 0; t < N_; t++){
    std::vector<GRBLinExpr> cp0 = getCP0(t);  // Control Point 0
    std::vector<GRBLinExpr> cp1 = getCP1(t);  // Control Poi_params = params;
    std::vector<GRBLinExpr> cp2 = getCP2(t);  // Control Point 2
    std::vector<GRBLinExpr> cp3 = getCP3(t);  // Control Point 3

    std::string name0 = "Occ_t"+std::to_string(t)+"_cp0";
    std::string name1 = "Occ_t"+std::to_string(t)+"_cp1";
    std::string name2 = "Occ_t"+std::to_string(t)+"_cp2";
    std::string name3 = "Occ_t"+std::to_string(t)+"_cp3";

    double r = 1.5;
    std::vector<GRBLinExpr> cp0_r = {cp0[0]-p[0], cp0[1]-p[1], cp0[2]-p[2]};
    occ_cons.push_back(m.addQConstr(GetNorm2(cp0_r) >= r*r,name0));

    std::vector<GRBLinExpr> cp1_r = {cp1[0]-p[0], cp1[1]-p[1], cp1[2]-p[2]};
    occ_cons.push_back(m.addQConstr(GetNorm2(cp1_r) >= r*r,name1));

    std::vector<GRBLinExpr> cp2_r = {cp2[0]-p[0], cp2[1]-p[1], cp2[2]-p[2]};
    occ_cons.push_back(m.addQConstr(GetNorm2(cp2_r) >= r*r,name2));

    std::vector<GRBLinExpr> cp3_r = {cp3[0]-p[0], cp3[1]-p[1], cp3[2]-p[2]};
    occ_cons.push_back(m.addQConstr(GetNorm2(cp3_r) >= r*r,name3));
  }

}

bool SolverGurobi::genNewTraj()
{
  bool solved = false;

  trials_ = 0;

  /*  std::cout << "A is\n";
    std::cout << polytopes_[0].A() << std::endl;
    std::cout << "B es esto:\n";
    std::cout << polytopes_[0].b().transpose() << std::endl;
    std::cout << "Number of rows= " << bb.rows() << std::endl;*/

  if (factor_initial_ < 1)
  {
    std::cout << "factor_initial_ is less than one, it doesn't make sense" << std::endl;
  }

  runtime_s_ = 0;
  int count = 0;
  for (double i = factor_initial_; i <= factor_final_ && solved == false; i = i + factor_increment_)
  {
    if (runtime_s_ > max_solver_time)
    {
      std::cout << red <<  "SolverGurobi::genNewTraj() - Solver took too long, returning false" << std::endl;
      return false;
    }

    // setup callback to terminate optimization if it takes too long
    // the entire process should be less than max_solver_time
    cb_.set_timeout(max_solver_time-runtime_s_);
    m.setCallback(&cb_);
    // std::cout << "timeout time is " << max_solver_time-runtime_s_ << std::endl;

    trials_ = trials_ + 1;
    findDT(i);
    // generate MINVO and Bezier conversion matrices
    // std::cout << "Going to try with dt_= " << dt_ << ", should_terminate_=" << cb_.should_terminate_ << std::endl;
    setMaxConstraints();
    setPolytopesConstraints();
    // setOcclusionConstraint();
    setConstraintsX0();
    setConstraintsXf();
    setDynamicConstraints();
    setObjective();
    resetX();

    // std::cout << "solving now!" << std::endl;
    solved = callOptimizer();
    // std::cout << "finished solving" << std::endl;
    /*    if (solved == true)
        {
          solved = isWmaxSatisfied();
        }*/
    if (solved == true)  // solved and Wmax is satisfied
    {
      factor_that_worked_ = i;
      // std::cout << "Factor= " << i << "(dt_= " << dt_ << ")---> worked" << std::endl;
    }
    else
    {
      // std::cout << "Factor= " << i << "(dt_= " << dt_ << ")---> didn't worked" << std::endl;
    }

  }

  return solved;
}

void SolverGurobi::setThreads(int threads)
{
  m.set("Threads", std::to_string(threads));
}

void SolverGurobi::setVerbose(int verbose)
{
  m.set("OutputFlag", std::to_string(verbose));  // 1 if you want verbose, 0 if not
}

void SolverGurobi::setWMax(double w_max)
{
  w_max_ = w_max;
}

void SolverGurobi::findDT(double factor)
{
  dt_ = factor * std::max(getDTInitial(), 2 * DC);
}

void SolverGurobi::setDynamicConstraints()
{
  // Remove the previous dynamic constraints
  for (int i = 0; i < dyn_cons.size(); i++)
  {
    m.remove(dyn_cons[i]);
  }
  dyn_cons.clear();

  // Dynamic Constraints
  for (int t = 0; t < N_ - 1; t++)  // From 0....N_-2
  {
    for (int i = 0; i < 3; i++)
    {
      dyn_cons.push_back(m.addConstr(getPos(t, dt_, i) == getPos(t + 1, 0, i),
                                     "ContPos_t" + std::to_string(t) + "_axis" + std::to_string(i)));  // Continuity in
                                                                                                       // position
      dyn_cons.push_back(m.addConstr(getVel(t, dt_, i) == getVel(t + 1, 0, i),
                                     "ContVel_t" + std::to_string(t) + "_axis" + std::to_string(i)));  // Continuity in
                                                                                                       // velocity
      dyn_cons.push_back(
          m.addConstr(getAccel(t, dt_, i) == getAccel(t + 1, 0, i),
                      "ContAccel_t" + std::to_string(t) + "_axis" + std::to_string(i)));  // Continuity in acceleration
    }
  }
}

void SolverGurobi::setOcc(const Eigen::MatrixXd& point){
    occ_point = point;
    is_occ_ = true;
}

// For the Jackal:
bool SolverGurobi::isWmaxSatisfied()
{
  for (int n = 0; n < N_; n++)
  {
    double xd = getVel(n, 0, 0).getValue();
    double yd = getVel(n, 0, 1).getValue();
    double xd2 = getAccel(n, 0, 0).getValue();
    double yd2 = getAccel(n, 0, 1).getValue();

    double numerator = xd * yd2 - yd * xd2;
    double denominator = xd * xd + yd * yd;
    double w_desired = (denominator > 0.001) ? fabs(numerator / denominator) : 0.5 * w_max_;

    if (w_desired > w_max_)
    {
      std::cout << "w_desired > than w_max: " << w_desired << " > " << w_max_ << "  , solving again" << std::endl;
      return false;
    }
  }
  return true;
}

bool SolverGurobi::callOptimizer()
{
  // int threads = m.get(GRB_IntParam_Threads);

  bool solved = true;

  // Select these parameteres with the tuning Tool of Gurobi
  // m.set("NumericFocus", "3");
  // m.set("Presolve", "0");

  m.update();
  temporal_ = temporal_ + 1;
  // printf("Writing into model.lp number=%d\n", temporal_);
  // m.write(ros::package::getPath("faster") + "/models/model2_" + std::to_string(temporal_) + ".lp");

  // Solve model and capture solution information
  auto start = std::chrono::steady_clock::now();
  try{
    m.optimize();
  } catch(GRBException e){
    // std::cout << e.getMessage() << std::endl;
  }
  auto end = std::chrono::steady_clock::now();
  auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() / 1000.0;
  // std::cout << "*************************Finished Optimization: " << elapsed << " s" << std::endl;
  // std::cout << "*************************Gurobi RUNTIME: " << m.get(GRB_DoubleAttr_Runtime) << " s" << std::endl;

  runtime_s_ = runtime_s_ + m.get(GRB_DoubleAttr_Runtime);

  /*  times_log.open("/home/jtorde/Desktop/ws/src/acl-planning/faster/models/times_log.txt", std::ios_base::app);
    times_log << elapsed << "\n";
    times_log.close();*/

  // printf("Going to check status");
  int optimstatus = m.get(GRB_IntAttr_Status);
  if (optimstatus == GRB_OPTIMAL)
  {
    // m.write(ros::package::getPath("faster") + "/models/model_wt" + std::to_string(temporal_) + ".lp");

    /*    if (polytopes_cons.size() > 0)  // Print the binary matrix only if I've included the polytope constraints
        {
          std::cout << "Solution: Binary Matrix:" << std::endl;

          for (int poly = 0; poly < b[0].size(); poly++)
          {
            for (int t = 0; t < N_; t++)
            {
              std::cout << b[t][poly].get(GRB_DoubleAttr_X) << " ";
            }
            std::cout << std::endl;
          }
        }
        std::cout << "Solution: Positions:" << std::endl;
        for (int t = 0; t < N_; t++)
        {
          std::cout << getPos(t, 0, 0) << "   ";
          std::cout << getPos(t, 0, 1) << "   ";
          std::cout << getPos(t, 0, 2) << std::endl;
        }

        std::cout << getPos(N_ - 1, dt_, 0) << "   ";
        std::cout << getPos(N_ - 1, dt_, 1) << "   ";
        std::cout << getPos(N_ - 1, dt_, 2) << std::endl;

        std::cout << "Solution: Coefficients d:" << std::endl;
        for (int t = 0; t < N_; t++)
        {
          std::cout << x[t][9].get(GRB_DoubleAttr_X) << "   ";
          std::cout << x[t][10].get(GRB_DoubleAttr_X) << "   ";
          std::cout << x[t][11].get(GRB_DoubleAttr_X) << std::endl;
        }*/
  }

  else
  {
    // total_not_solved = total_not_solved + 1;
    // std::cout << "TOTAL NOT SOLVED" << total_not_solved << std::endl;
    // No solution

    // m.write("/home/nick/Desktop/" + std::to_string(temporal_) +
    //        "dt=" + std::to_string(dt_) + ".lp");
    solved = false;
    if (optimstatus == GRB_INF_OR_UNBD)
    {
      // printf("GUROBI SOLUTION: Unbounded or Infeasible. Maybe too small dt?\n");

      /*      m.computeIIS();  // Compute the Irreducible Inconsistent Subsystem and write it on a file
            m.write("/home/jtorde/Desktop/ws/src/acl-planning/cvx/models/model_rp" + std::to_string(temporal_) +
                    "dt=" + std::to_string(dt_) + ".ilp");*/
    }

    if (optimstatus == GRB_NUMERIC)
    {
      // printf("GUROBI Status: Numerical issues\n");
      // printf("Model may be infeasible or unbounded\n");  // Taken from the Gurobi documentation
    }

    if (optimstatus == GRB_INTERRUPTED)
    {
      // printf("GUROBI Status: Interrumped by the user\n");
    }
  }
  return solved;

  // printf("Optimstatus= %d\n", optimstatus);
  // std::cout << "*************************Finished Optimization" << std::endl;

  /*  std::cout << "\nOBJECTIVE: " << m.get(GRB_DoubleAttr_ObjVal) << std::endl;


*/
}

double SolverGurobi::getDTInitial()
{
  double dt_initial = 0;
  float t_vx = 0;
  float t_vy = 0;
  float t_vz = 0;
  float t_ax = 0;
  float t_ay = 0;
  float t_az = 0;
  float t_jx = 0;
  float t_jy = 0;
  float t_jz = 0;

  t_vx = fabs(xf_[0] - x0_[0]) / v_max_;
  t_vy = fabs(xf_[1] - x0_[1]) / v_max_;
  t_vz = fabs(xf_[2] - x0_[2]) / v_max_;

  /*  printf("times vel: t_ax, t_ay, t_az:\n");
    std::cout << t_vx << "  " << t_vy << "  " << t_vz << std::endl;*/

  float jerkx = copysign(1, xf_[0] - x0_[0]) * j_max_;
  float jerky = copysign(1, xf_[1] - x0_[1]) * j_max_;
  float jerkz = copysign(1, xf_[2] - x0_[2]) * j_max_;
  float a0x = x0_[6];
  float a0y = x0_[7];
  float a0z = x0_[8];
  float v0x = x0_[3];
  float v0y = x0_[4];
  float v0z = x0_[5];

  // Solve For JERK
  // polynomial ax3+bx2+cx+d=0 --> coeff=[d c b a]
  Eigen::Vector4d coeff_jx(x0_[0] - xf_[0], v0x, a0x / 2.0, jerkx / 6.0);
  Eigen::Vector4d coeff_jy(x0_[1] - xf_[1], v0y, a0y / 2.0, jerky / 6.0);
  Eigen::Vector4d coeff_jz(x0_[2] - xf_[2], v0z, a0z / 2.0, jerkz / 6.0);

  /*  std::cout << "Coefficients for jerk" << std::endl;
    std::cout << "Coeffx=" << coeff_jx.transpose() << std::endl;
    std::cout << "Coeffy=" << coeff_jy.transpose() << std::endl;
    std::cout << "Coeffz=" << coeff_jz.transpose() << std::endl;*/

  Eigen::PolynomialSolver<double, Eigen::Dynamic> psolve_jx(coeff_jx);
  Eigen::PolynomialSolver<double, Eigen::Dynamic> psolve_jy(coeff_jy);
  Eigen::PolynomialSolver<double, Eigen::Dynamic> psolve_jz(coeff_jz);

  std::vector<double> realRoots_jx;
  std::vector<double> realRoots_jy;
  std::vector<double> realRoots_jz;
  psolve_jx.realRoots(realRoots_jx);
  psolve_jy.realRoots(realRoots_jy);
  psolve_jz.realRoots(realRoots_jz);

  t_jx = MinPositiveElement(realRoots_jx);
  t_jy = MinPositiveElement(realRoots_jy);
  t_jz = MinPositiveElement(realRoots_jz);

  /*  printf("times jerk: t_jx, t_jy, t_jz:\n");
    std::cout << t_jx << "  " << t_jy << "  " << t_jz << std::endl;*/

  float accelx = copysign(1, xf_[0] - x0_[0]) * a_max_;
  float accely = copysign(1, xf_[1] - x0_[1]) * a_max_;
  float accelz = copysign(1, xf_[2] - x0_[2]) * a_max_;

  // Solve For ACCELERATION
  // polynomial ax2+bx+c=0 --> coeff=[c b a]
  Eigen::Vector3d coeff_ax(x0_[0] - xf_[0], v0x, 0.5 * accelx);
  Eigen::Vector3d coeff_ay(x0_[1] - xf_[1], v0y, 0.5 * accely);
  Eigen::Vector3d coeff_az(x0_[2] - xf_[2], v0z, 0.5 * accelz);

  /*  std::cout << "Coefficients for accel" << std::endl;
    std::cout << "coeff_ax=" << coeff_ax.transpose() << std::endl;
    std::cout << "coeff_ay=" << coeff_ay.transpose() << std::endl;
    std::cout << "coeff_az=" << coeff_az.transpose() << std::endl;*/

  Eigen::PolynomialSolver<double, Eigen::Dynamic> psolve_ax(coeff_ax);
  Eigen::PolynomialSolver<double, Eigen::Dynamic> psolve_ay(coeff_ay);
  Eigen::PolynomialSolver<double, Eigen::Dynamic> psolve_az(coeff_az);

  std::vector<double> realRoots_ax;
  std::vector<double> realRoots_ay;
  std::vector<double> realRoots_az;
  psolve_ax.realRoots(realRoots_ax);
  psolve_ay.realRoots(realRoots_ay);
  psolve_az.realRoots(realRoots_az);

  t_ax = MinPositiveElement(realRoots_ax);
  t_ay = MinPositiveElement(realRoots_ay);
  t_az = MinPositiveElement(realRoots_az);

  /*  printf("times accel: t_ax, t_ay, t_az:\n");
    std::cout << t_ax << "  " << t_ay << "  " << t_az << std::endl;
  */
  dt_initial = std::max({ t_vx, t_vy, t_vz, t_ax, t_ay, t_az, t_jx, t_jy, t_jz }) / N_;
  if (dt_initial > 10000)  // happens when there is no solution to the previous eq.
  {
    printf("there is not a solution to the previous equations\n");
    dt_initial = 0;
  }
  // printf("returning dt_initial=%f\n", dt_initial);
  return dt_initial;
}

GRBLinExpr SolverGurobi::getPos(int t, double tau, int ii)
{
  GRBLinExpr pos = x[t][0 + ii] * tau * tau * tau + x[t][3 + ii] * tau * tau + x[t][6 + ii] * tau + x[t][9 + ii];
  // std::cout << "x tiene size" << x.size() << std::endl;
  // std::cout << "getPos devuelve=" << pos << std::endl;
  return pos;
}

GRBLinExpr SolverGurobi::getVel(int t, double tau, int ii)
{  // t is the segment, tau is the time inside a specific segment (\in[0,dt], i is the axis)

  GRBLinExpr vel = 3 * x[t][0 + ii] * tau * tau + 2 * x[t][3 + ii] * tau + x[t][6 + ii];
  return vel;
}

GRBLinExpr SolverGurobi::getAccel(int t, double tau, int ii)
{  // t is the segment, tau is the time inside a specific segment(\in[0, dt], i is the axis)

  GRBLinExpr accel = 6 * x[t][0 + ii] * tau + 2 * x[t][3 + ii];
  return accel;
}

GRBLinExpr SolverGurobi::getJerk(int t, double tau, int ii)
{  // t is the segment, tau is the time inside a specific segment (\in[0,dt], i is the axis)

  GRBLinExpr jerk = 6 * x[t][0 + ii];  // Note that here tau doesn't appear (makes sense)
  return jerk;
}

// Coefficient getters: At^3 + Bt^2 + Ct + D  , t \in [0, dt_]
GRBLinExpr SolverGurobi::getA(int t, int ii)  // interval, axis
{
  return x[t][0 + ii];
}

GRBLinExpr SolverGurobi::getB(int t, int ii)  // interval, axis
{
  return x[t][3 + ii];
}

GRBLinExpr SolverGurobi::getC(int t, int ii)  // interval, axis
{
  return x[t][6 + ii];
}

GRBLinExpr SolverGurobi::getD(int t, int ii)  // interval, axis
{
  return x[t][9 + ii];
}

// Coefficients Normalized: At^3 + Bt^2 + Ct + D  , t \in [0, 1]
GRBLinExpr SolverGurobi::getAn(int t, int ii)  // interval, axis
{
  return x[t][0 + ii] * dt_ * dt_ * dt_;
}

GRBLinExpr SolverGurobi::getBn(int t, int ii)  // interval, axis
{
  return x[t][3 + ii] * dt_ * dt_;
}

GRBLinExpr SolverGurobi::getCn(int t, int ii)  // interval, axis
{
  return x[t][6 + ii] * dt_;
}

GRBLinExpr SolverGurobi::getDn(int t, int ii)  // interval, axis
{
  return x[t][9 + ii];
}

// Control Points (of the splines) getters
std::vector<GRBLinExpr> SolverGurobi::getCP0(int t)  // Control Point 0 of interval t
{                                                    // Control Point 0 is initial position
                                                     // std::cout << "Getting CP0" << std::endl;
  std::vector<GRBLinExpr> cp = { getPos(t, 0, 0), getPos(t, 0, 1), getPos(t, 0, 2) };
  return cp;
}

std::vector<GRBLinExpr> SolverGurobi::getCP1(int t)  // Control Point 1 of interval t
{
  GRBLinExpr cpx = (getCn(t, 0) + 3 * getDn(t, 0)) / 3;
  GRBLinExpr cpy = (getCn(t, 1) + 3 * getDn(t, 1)) / 3;
  GRBLinExpr cpz = (getCn(t, 2) + 3 * getDn(t, 2)) / 3;
  std::vector<GRBLinExpr> cp = { cpx, cpy, cpz };
  return cp;
}

std::vector<GRBLinExpr> SolverGurobi::getCP2(int t)  // Control Point 2 of interval t
{
  GRBLinExpr cpx = (getBn(t, 0) + 2 * getCn(t, 0) + 3 * getDn(t, 0)) / 3;
  GRBLinExpr cpy = (getBn(t, 1) + 2 * getCn(t, 1) + 3 * getDn(t, 1)) / 3;
  GRBLinExpr cpz = (getBn(t, 2) + 2 * getCn(t, 2) + 3 * getDn(t, 2)) / 3;
  std::vector<GRBLinExpr> cp = { cpx, cpy, cpz };
  return cp;
}

std::vector<GRBLinExpr> SolverGurobi::getCP3(int t)  // Control Point 3 of interval t
{                                                    // Control Point 3 is end position
  std::vector<GRBLinExpr> cp = { getPos(t, dt_, 0), getPos(t, dt_, 1), getPos(t, dt_, 2) };
  return cp;
}

GRBModel SolverGurobi::getModel() const
{
  return m;
}