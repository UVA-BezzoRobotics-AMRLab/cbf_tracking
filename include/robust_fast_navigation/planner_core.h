#pragma once

#include <faster/solver.hpp>

#include <costmap_2d/cost_values.h>
#include <costmap_2d/costmap_2d.h>

#include <robust_fast_navigation/corridor.h>
#include <robust_fast_navigation/map_util.h>
#include <robust_fast_navigation/spline.h>

struct planner_params
{
  double V_MAX;
  double A_MAX;
  double J_MAX;
  double DT_FACTOR_INIT;
  double DT_FACTOR_FINAL;
  double DT_FACTOR_INCREMENT;
  double SOLVER_TRAJ_DT;
  double W_MAX;
  bool FORCE_FINAL_CONSTRAINT;
  int N_SEGMENTS;
  int N_THREADS;
  bool VERBOSE;
  bool USE_MINVO;
  bool PLAN_IN_FREE;
  bool SIMPLIFY_JPS;
  double MAX_SOLVE_TIME;
};
typedef struct planner_params planner_params_t;

class SplineWrapper
{
public:
  tk::spline spline; // Expose the tk::spline
};
typedef SplineWrapper spline_t;

class Planner
{
public:
  Planner ();
  ~Planner ();

  void set_params (const planner_params_t &params);

  // setters

  void set_start (const Eigen::MatrixXd &start);
  void set_goal (const Eigen::MatrixXd &goal);
  void set_costmap (const map_util::occupancy_grid_t &map);

  // #ifdef FOUND_PYBIND11
  // void set_start(const Eigen::MatrixXd& start);
  // void set_goal(const Eigen::MatrixXd& goal);
  // void set_costmap(const std::shared_ptr<costmap_2d::Costmap2D>& map);
  // #endif

  // getters
  std::vector<Eigen::Vector3d> get_cps ();
  std::vector<state> get_trajectory ();
  std::vector<state> get_arclen_traj ();

  // std::vector<spline_t> get_tube();

  // Eigen::Matrix3Xd get_corridor_boundary();

  bool plan (double horizon, std::vector<Eigen::Vector2d> &jpsPath,
             std::vector<Eigen::MatrixX4d> &hPolys);

  std::vector<Eigen::Vector2d>
  getJPSInFree (const std::vector<Eigen::Vector2d> &path);
  bool JPSIntersectObs (const std::vector<Eigen::Vector2d> &path);

  const SolverGurobi &get_solver ();

private:
  Eigen::MatrixXd _old_goal;
  Eigen::MatrixXd _start;
  Eigen::MatrixXd _goal;
  Eigen::Matrix3Xd _corridor_boundary;

  // Polygon_2 _cgal_border;

  bool _is_occ;
  bool _is_map_set;
  bool _is_goal_set;
  bool _simplify_jps;
  bool _plan_in_free;
  bool _is_start_set;

  SolverGurobi _solver;

  planner_params_t _params;

  map_util::occupancy_grid_t _map;

  double binary_search (int segment, double dl, double start, double end,
                        double tolerance);
  double compute_arclen (int segment, double t0, double tf);
  bool reparam_traj (std::vector<double> &ss, std::vector<double> &xs,
                     std::vector<double> &ys);
};
