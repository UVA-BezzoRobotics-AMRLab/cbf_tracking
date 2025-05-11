#include <robust_fast_navigation/JPS.h>
#include <robust_fast_navigation/planner_core.h>

Planner::Planner ()
{
  _is_map_set = false;
  _is_goal_set = false;
  _simplify_jps = false;
  _is_start_set = false;
  _plan_in_free = false;
}

Planner::~Planner () {}

void
Planner::set_params (const planner_params_t &params)
{
  _params = params;

  _plan_in_free = params.PLAN_IN_FREE;
  _simplify_jps = params.SIMPLIFY_JPS;

  // solver params
  double w_max = params.W_MAX;
  double v_max = params.V_MAX;
  double a_max = params.A_MAX;
  double j_max = params.J_MAX;

  double limits[3] = { v_max, a_max, j_max };
  double factor_init = params.DT_FACTOR_INIT;
  double factor_final = params.DT_FACTOR_FINAL;
  double factor_increment = params.DT_FACTOR_INCREMENT;

  std::cout << "Loaded params: " << std::endl;
  std::cout << "PLAN_IN_FREE: " << _plan_in_free << std::endl;
  std::cout << "SIMPLIFY_JPS: " << _simplify_jps << std::endl;
  std::cout << "W_MAX: " << w_max << std::endl;
  std::cout << "V_MAX: " << v_max << std::endl;
  std::cout << "A_MAX: " << a_max << std::endl;
  std::cout << "J_MAX: " << j_max << std::endl;
  std::cout << "DT_FACTOR_INIT: " << factor_init << std::endl;
  std::cout << "DT_FACTOR_FINAL: " << factor_final << std::endl;
  std::cout << "DT_FACTOR_INCREMENT: " << factor_increment << std::endl;
  std::cout << "MAX_SOLVE_TIME: " << params.MAX_SOLVE_TIME << std::endl;

  _solver.setN (params.N_SEGMENTS);
  _solver.createVars ();
  _solver.setDC (params.SOLVER_TRAJ_DT);
  _solver.setBounds (limits);
  _solver.setWMax (params.W_MAX);
  _solver.setForceFinalConstraint (params.FORCE_FINAL_CONSTRAINT);
  _solver.setFactorInitialAndFinalAndIncrement (factor_init, factor_final,
                                                factor_increment);
  _solver.setThreads (params.N_THREADS);
  _solver.setVerbose (params.VERBOSE);
  _solver.setUseMinvo (params.USE_MINVO);
  _solver.setMaxSolverTime (params.MAX_SOLVE_TIME);
}

void
Planner::set_start (const Eigen::MatrixXd &start)
{
  _start = start;
  _is_start_set = true;
}

void
Planner::set_goal (const Eigen::MatrixXd &goal)
{
  _goal = goal;
  _is_goal_set = true;
}

void
Planner::set_costmap (const map_util::occupancy_grid_t &map)
{
  _map = map;
  _is_map_set = true;
}

const SolverGurobi &
Planner::get_solver ()
{
  return _solver;
}

bool
Planner::plan (double horizon, std::vector<Eigen::Vector2d> &jpsPath,
               std::vector<Eigen::MatrixX4d> &hPolys)
{
  if (!_is_start_set || !_is_goal_set || !_is_map_set)
    {
      std::cout << termcolor::red << "Planner: missing start, goal or costmap"
                << termcolor::reset << std::endl;
      return false;
    }

  /*************************************
  ************ PERFORM  JPS ************
  **************************************/

  jps::JPSPlan jps;
  unsigned int sX, sY, eX, eY;

  // potentially change this to odom instead of start
  std::vector<unsigned int> start_cells
      = _map.world_to_map (_start (0, 0), _start (1, 0));
  sX = start_cells[0];
  sY = start_cells[1];
  // std::cout << "startind is " << _map.cells_to_index(sX, sY) << std::endl;
  // std::cout << "sX = " << sX << std::endl;
  // std::cout << "sY = " << sY << std::endl;

  std::vector<unsigned int> goal_cells
      = _map.world_to_map (_goal (0), _goal (1));
  eX = goal_cells[0];
  eY = goal_cells[1];

  // std::cout << "eX = " << eX << std::endl;
  // std::cout << "eY = " << eY << std::endl;

  jps.set_start (sX, sY);
  jps.set_destination (eX, eY);
  jps.set_occ_value (costmap_2d::INSCRIBED_INFLATED_OBSTACLE);

  jps.set_map (_map.get_data (), _map.width, _map.height, _map.origin_x,
               _map.origin_y, _map.resolution);

  jps.JPS ();

  std::vector<Eigen::Vector2d> oldJps = jpsPath;

  jpsPath = jps.getPath (_simplify_jps);

  if (jpsPath.size () < 2)
    {
      std::cout << termcolor::red << "Planner: JPS failed" << termcolor::reset
                << std::endl;
      return false;
    }

  // check if old jps path has value
  if (oldJps.size () > 0 && !JPSIntersectObs (oldJps))
    {

      // if new path angle is much different from old path with nearly same
      // length, use old path
      Eigen::Vector2d oldDir = oldJps[oldJps.size () - 1] - oldJps[0];
      Eigen::Vector2d newDir = jpsPath[jpsPath.size () - 1] - jpsPath[0];

      oldDir = oldDir / oldDir.norm ();
      newDir = newDir / newDir.norm ();

      double angle = acos (oldDir.dot (newDir)) * 180 / M_PI;
      std::cout << termcolor::on_magenta
                << "Old jps given, and doesn't intersect obs" << std::endl;
      std::cout << "angle between old and new path is " << angle
                << termcolor::reset << std::endl;

      if (angle > 30)
        {
          // if new path is longer or barely shorter than old path, use old
          // path
          double oldLen = 0;
          for (int i = 0; i < oldJps.size () - 1; ++i)
            {
              oldLen += (oldJps[i + 1] - oldJps[i]).norm ();
            }

          double newLen = 0;
          for (int i = 0; i < jpsPath.size () - 1; ++i)
            {
              newLen += (jpsPath[i + 1] - jpsPath[i]).norm ();
            }

          if (newLen > .98 * oldLen)
            {
              jpsPath = oldJps;
            }
          std::cout << termcolor::on_magenta << "new len is " << newLen
                    << "\nand old len is " << oldLen << termcolor::reset
                    << std::endl;
        }
    }

  /*************************************
  ************* REFINE JPS *************
  **************************************/

  std::vector<Eigen::Vector2d> newJPSPath;
  bool enforce_final_pos = false;

  if (_plan_in_free)
    {
      jpsPath = getJPSInFree (jpsPath);
    }
  if (jps.truncateJPS (jpsPath, newJPSPath, horizon))
    {

      jpsPath = newJPSPath;

      // enforce_final_pos = (_goal.col(0).head(2) - jpsPath.back()).norm() >
      // 1e-1;

      _goal << Eigen::Vector3d (jpsPath.back ()[0], jpsPath.back ()[1], 0),
          Eigen::Vector3d::Zero (), Eigen::Vector3d::Zero (),
          Eigen::Vector3d::Zero ();
    }

  if (jpsPath.size () == 0)
    {
      std::cout << termcolor::red << "Planner: JPS modifications failed"
                << termcolor::reset << std::endl;
      return false;
    }

  /*************************************
  ********* GENERATE POLYTOPES *********
  **************************************/

  hPolys.clear ();
  if (!corridor::createCorridorJPS (jpsPath, _map, hPolys, _start, _goal))
    {
      std::cout << termcolor::red << "Planner: Corridor creation failed"
                << termcolor::reset << std::endl;
      return false;
    }

  bool is_in_corridor = false;

  // if adjacent polytopes don't overlap, don't plan
  for (int p = 0; p < hPolys.size () - 1; p++)
    {
      if (!geo_utils::overlap (hPolys[p], hPolys[p + 1]))
        {
          // ROS_ERROR("CORRIDOR IS NOT FULLY CONNECTED");
          return false;
        }

      if (!is_in_corridor)
        is_in_corridor = corridor::isInPoly (
            hPolys[p], Eigen::Vector2d (_start (0, 0), _start (1, 0)));
    }

  // if(corridor::union_corridor(hPolys, _cgal_border, _corridor_boundary))
  //     std::cout << "we did it!!!" << std::endl;

  /*************************************
  ******** GENERATE  TRAJECTORY ********
  **************************************/

  state initialState;
  state finalState;

  initialState.setPos (_start (0, 0), _start (1, 0), _start (2, 0));
  initialState.setVel (_start (0, 1), _start (1, 1), _start (2, 1));
  initialState.setAccel (_start (0, 2), _start (1, 2), _start (2, 2));
  initialState.setJerk (_start (0, 3), _start (1, 3), _start (2, 3));

  finalState.setPos (_goal.col (0));
  finalState.setVel (_goal.col (1));
  finalState.setAccel (_goal.col (2));
  finalState.setJerk (_goal.col (3));

  // ROS_INFO("setting up");
  _solver.setX0 (initialState);
  _solver.setXf (finalState);
  // _solver.setForceFinalConstraint(enforce_final_pos);
  _solver.setPolytopes (hPolys);

  // time trajectory generation
  ros::Time start_solve = ros::Time::now ();
  if (!_solver.genNewTraj ())
    {
      std::cout << termcolor::red << "Planner: Generating trajectory failed"
                << termcolor::reset << std::endl;
      return false;
    }
  else
    {
      std::cout << termcolor::green << "Planner: Solver found trajectory"
                << termcolor::reset << std::endl;
    }

  _solver.fillX ();

  // ensure trajectory does not overlap lethal obstacles
  for (int i = 0; i < _solver.X_temp_.size (); i++)
    {
      Eigen::Vector2d pos = _solver.X_temp_[i].pos.head (2);
      if (_map.is_occupied (pos[0], pos[1]))
        {
          std::cout << termcolor::red
                    << "Planner: Trajectory overlaps obstacle"
                    << termcolor::reset << std::endl;
          return false;
        }
    }

  _is_start_set = false;
  _is_goal_set = false;
  _is_map_set = false;

  _old_goal = _goal;

  return true;
}

std::vector<state>
Planner::get_trajectory ()
{
  return _solver.X_temp_;
}

std::vector<state>
Planner::get_arclen_traj ()
{
  std::vector<double> ss;
  std::vector<double> xs;
  std::vector<double> ys;

  bool status = reparam_traj (ss, xs, ys);

  if (!status)
    return {};

  std::vector<state> ret;
  ret.resize (ss.size ());

  for (int i = 0; i < ret.size (); ++i)
    {
      state x;
      x.pos (0) = xs[i];
      x.pos (1) = ys[i];
      x.t = ss[i];

      ret[i] = x;
    }

  return ret;
}

std::vector<Eigen::Vector3d>
Planner::get_cps ()
{
  std::vector<Eigen::Vector3d> ret;
  for (int interval = 0; interval < _solver.N_; interval++)
    {
      std::vector<GRBLinExpr> cp0 = _solver.getCP0 (interval);
      std::vector<GRBLinExpr> cp1 = _solver.getCP1 (interval);
      std::vector<GRBLinExpr> cp2 = _solver.getCP2 (interval);
      std::vector<GRBLinExpr> cp3 = _solver.getCP3 (interval);

      Eigen::Vector3d cp0_vec (cp0[0].getValue (), cp0[1].getValue (),
                               cp0[2].getValue ());
      Eigen::Vector3d cp1_vec (cp1[0].getValue (), cp1[1].getValue (),
                               cp1[2].getValue ());
      Eigen::Vector3d cp2_vec (cp2[0].getValue (), cp2[1].getValue (),
                               cp2[2].getValue ());
      Eigen::Vector3d cp3_vec (cp3[0].getValue (), cp3[1].getValue (),
                               cp3[2].getValue ());

      ret.push_back (cp0_vec);
      ret.push_back (cp1_vec);
      ret.push_back (cp2_vec);
      ret.push_back (cp3_vec);
    }

  return ret;
}

bool
Planner::JPSIntersectObs (const std::vector<Eigen::Vector2d> &path)
{
  if (!_is_map_set)
    {
      std::cout << termcolor::yellow << "[Planner] costmap not yet set"
                << termcolor::reset << std::endl;
      return false;
    }

  for (int i = 0; i < path.size () - 1; i++)
    {
      double x, y;
      unsigned int sx, sy, ex, ey;

      std::vector<unsigned int> start_cells
          = _map.world_to_map (path[i][0], path[i][1]);
      sx = start_cells[0];
      sy = start_cells[1];

      std::vector<unsigned int> end_cells
          = _map.world_to_map (path[i + 1][0], path[i + 1][1]);
      ex = end_cells[0];
      ey = end_cells[1];

      _map.raycast (sx, sy, ex, ey, x, y,
                    { costmap_2d::LETHAL_OBSTACLE,
                      costmap_2d::INSCRIBED_INFLATED_OBSTACLE });

      // if x,y does not reach the end of the ray, we hit an unknown cell
      Eigen::Vector2d p (x, y);

      if ((path[i + 1] - p).norm () > _map.resolution)
        {
          std::vector<unsigned int> p_cells = _map.world_to_map (p[0], p[1]);
          return true;
        }
    }

  return false;
}

std::vector<Eigen::Vector2d>
Planner::getJPSInFree (const std::vector<Eigen::Vector2d> &path)
{

  if (!_is_map_set)
    {
      std::cout << termcolor::yellow << "[Planner] costmap not yet set"
                << termcolor::reset << std::endl;
      return path;
    }

  std::vector<Eigen::Vector2d> ret;

  if (path.size () < 2)
    {
      std::cout << termcolor::yellow << "[Planner] JPS only contains one point"
                << termcolor::reset << std::endl;
      return path;
    }

  // walk along jps and find first unknown cell

  double x, y;
  unsigned int sx, sy, ex, ey;

  ret.push_back (path[0]);

  // raycast along JPS until we hit an unknown cell
  for (int i = 0; i < path.size () - 1; i++)
    {

      std::vector<unsigned int> start_cells
          = _map.world_to_map (path[i][0], path[i][1]);
      sx = start_cells[0];
      sy = start_cells[1];

      std::vector<unsigned int> end_cells
          = _map.world_to_map (path[i + 1][0], path[i + 1][1]);
      ex = end_cells[0];
      ey = end_cells[1];

      _map.raycast (sx, sy, ex, ey, x, y, { costmap_2d::NO_INFORMATION });

      // if x,y does not reach the end of the ray, we hit an unknown cell
      Eigen::Vector2d p (x, y);

      if ((path[i + 1] - p).norm () > 1e-3)
        {
          ret.push_back (p);
          break;
        }

      ret.push_back (path[i + 1]);
    }

  return ret;
}

bool
Planner::reparam_traj (std::vector<double> &ss, std::vector<double> &xs,
                       std::vector<double> &ys)
{

  if (_solver.X_temp_.size () == 0)
    return false;

  // get arc length for each segment
  int N = _solver.N_;
  double dt = _solver.dt_;

  double cumsum[N];
  cumsum[0] = compute_arclen (0, 0, dt);

  for (int i = 1; i < N; ++i)
    {
      cumsum[i] = cumsum[i - 1] + compute_arclen (i, 0, dt);
    }

  double total_length = cumsum[N - 1];

  double M = 20;
  double ds = total_length / M;

  ss.resize (M + 1);
  xs.resize (M + 1);
  ys.resize (M + 1);

  for (int i = 0; i <= M; ++i)
    {
      double s = i * ds;

      int min_idx = -1;
      for (int j = 0; j < N; ++j)
        {
          if (cumsum[j] - s >= -1e-3)
            {
              min_idx = j;
              break;
            }
        }

      // realistically this shouldn't happen with the 1e-3 tolerance but just
      // in case...
      if (min_idx == -1)
        {
          std::cerr << "reparameterization of trajectory has failed"
                    << std::endl;
          return false;
        }

      double l_before = 0;
      if (min_idx > 0)
        {
          l_before = cumsum[min_idx - 1];
        }

      double ti = binary_search (min_idx, s - l_before, 0, dt, 1e-3);

      ss[i] = s;
      try
        {
          xs[i] = _solver.getPos (min_idx, ti, 0).getValue ();
          ys[i] = _solver.getPos (min_idx, ti, 1).getValue ();
        }
      catch (const GRBException &e)
        {
          std::cerr << "in reparam traj " << e.getMessage () << '\n';
          std::cerr << "segment " << min_idx << std::endl;
          exit (1);
        }
    }

  return true;
}

double
Planner::binary_search (int segment, double dl, double start, double end,
                        double tolerance)
{
  double t_left = start;
  double t_right = end;

  double prev_s = 0;
  double s = -1000;

  while (fabs (prev_s - s) > tolerance)
    {
      prev_s = s;

      double t_mid = (t_left + t_right) / 2;
      s = compute_arclen (segment, start, t_mid);

      if (s < dl)
        t_left = t_mid;
      else
        t_right = t_mid;
    }

  return (t_left + t_right) / 2;
}

double
Planner::compute_arclen (int segment, double t0, double tf)
{

  double s = 0.0;
  double dt = (tf - t0) / 10.;
  for (double t = t0; t <= tf; t += dt)
    {
      double dx, dy;
      try
        {
          dx = _solver.getVel (segment, t, 0).getValue ();
          dy = _solver.getVel (segment, t, 1).getValue ();
        }
      catch (const GRBException &e)
        {
          std::cerr << "in compute_arclen" << e.getMessage () << '\n';
          std::cerr << "segment " << segment << std::endl;
          exit (1);
        }

      s += std::sqrt (dx * dx + dy * dy) * dt;
    }
  return s;
}
