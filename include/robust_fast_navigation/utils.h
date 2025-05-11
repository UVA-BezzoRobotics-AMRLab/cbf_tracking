#pragma once

#include <Eigen/Eigen>
#include <iostream>
#include <string_view>
#include <tuple>
#include <vector>

#include <costmap_2d/costmap_2d_ros.h>
#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/OccupancyGrid.h>
#include <std_msgs/ColorRGBA.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <visualization_msgs/Marker.h>

#include <gcopter/geo_utils.hpp>

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>

#include <faster/faster_types.hpp>

#include <robust_fast_navigation/JPS.h>
#include <robust_fast_navigation/tinycolormap.hpp>

/**********************************************************************
    This function takes in a polygon and shifts all hyperplanes outward
    along the normals by a distance d.

    Inputs:
      - poly: polygon to expand
      - d: distance to move all hyperplanes

    Returns:
      - Expanded polygon
***********************************************************************/
inline Eigen::MatrixX4d
expandPoly (const Eigen::MatrixX4d &poly, const double &d)
{
  Eigen::MatrixX4d ret = poly;

  for (int i = 0; i < ret.rows (); i++)
    {
      Eigen::Vector3f n (ret.row (i)[0], ret.row (i)[1], ret.row (i)[2]);
      Eigen::Vector3f en = n / n.norm ();

      Eigen::Vector3f p;
      if (n[0] < 1e-6 && n[1] < 1e-6 && n[2] < 1e-6)
        continue;

      if (n[0] >= 1e-6)
        p = Eigen::Vector3f (-ret.row (i)[3] / n[0], 0, 0);
      else if (n[1] >= 1e-6)
        p = Eigen::Vector3f (0, -ret.row (i)[3] / n[1], 0);
      else if (n[2] >= 1e-6)
        p = Eigen::Vector3f (0, 0, -ret.row (i)[3] / n[2]);

      ret.row (i)[3] = -n.dot (p) - d * n.dot (en);
    }

  return ret;
}

/**********************************************************************
    This function returns the distance of a point to a polygon

    Inputs:
      - poly: polygon under consideration
      - p: Point to test

    Returns:
      - distance of point to polygon

    TODO: Maybe add feature to make distance negative if point is
    inside polygon?
***********************************************************************/
inline double
dist2Poly (const Eigen::Vector2d &point, const Eigen::MatrixX4d &poly)
{

  double dist = 100000.;

  for (int i = 0; i < poly.rows (); i++)
    {
      Eigen::Vector4d plane = poly.row (i);
      double d = fabs (plane[0] * point[0] + plane[1] * point[1] + plane[3]);
      double n = sqrt (plane[0] * plane[0] + plane[1] * plane[1]);
      double plane_dist = d / n;

      if (plane_dist < dist)
        dist = plane_dist;
    }

  return dist;
}

/**********************************************************************
    This function determines if a given trajectory is fully contained
    within a given corridor. The function also passes in a threshold
    variable, where the trajectory will still be considered "inside"
    the corridor if no point is too far outside.

    Inputs:
      - traj: trajectory under consideration
      - hpolys: polygonal corridor used to test against trajectory
      - thresh: allowed tolerance for trajectory to be outisde corridor

    Returns:
      - boolean whether trajectory is in or out of corridor.
***********************************************************************/
inline bool
isTrajOutsidePolys (const trajectory_msgs::JointTrajectory &traj,
                    const std::vector<Eigen::MatrixX4d> &hpolys, double thresh)
{

  int last_poly_idx = 0;

  for (const trajectory_msgs::JointTrajectoryPoint &pt : traj.points)
    {

      Eigen::Vector2d traj_p (pt.positions[0], pt.positions[1]);
      bool in_corridor = false;
      double max_dist = -1.;

      for (int p = last_poly_idx; p < hpolys.size (); p++)
        {
          // bool in_corridor = isInPoly(hpolys[p], traj_p);
          bool in_corridor = true;
          Eigen::Vector4d pR4 (traj_p[0], traj_p[1], 0, 1);
          Eigen::VectorXd res = hpolys[p] * pR4;

          for (int i = 0; i < res.rows (); ++i)
            {
              if (res (i) > 0)
                {
                  in_corridor = false;
                  break;
                }
            }

          if (in_corridor)
            {
              last_poly_idx = p;
              break;
            }

          max_dist = std::max (max_dist, dist2Poly (traj_p, hpolys[p]));
        }

      if (max_dist > thresh)
        return true;
    }

  return false;
}

/**********************************************************************
    This function determines if a given trajectory overlaps the lethal
    obstacles in a costmap.

    Inputs:
      - traj: trajectory under consideration
      - map: costmap used for testing

    Returns:
      - boolean whether trajectory overlaps obstacles or not.
***********************************************************************/
inline bool
isTrajOverlappingObs (const trajectory_msgs::JointTrajectory &traj,
                      const costmap_2d::Costmap2D &map)
{

  for (const trajectory_msgs::JointTrajectoryPoint &pt : traj.points)
    {

      unsigned int mx, my;
      if (map.worldToMap (pt.positions[0], pt.positions[1], mx, my))
        {

          if (map.getCost (mx, my)
              == costmap_2d::LETHAL_OBSTACLE) // || costmap_2d::)
            return true;
        }
    }

  return false;
}

inline bool
testMap (unsigned char val, unsigned char test_val)
{

  unsigned char true_test_val = test_val;

  if (test_val == costmap_2d::FREE_SPACE)
    return val >= costmap_2d::INSCRIBED_INFLATED_OBSTACLE;

  return val != test_val;
}

// following bresenham / raycast method from
// https://docs.ros.org/en/api/costmap_2d/html/costmap__2d_8h_source.html
inline void
bresenham (const costmap_2d::Costmap2D &_map, unsigned int abs_da,
           unsigned int abs_db, int error_b, int offset_a, int offset_b,
           unsigned int offset, unsigned int max_range, unsigned int &term,
           unsigned int test_val = costmap_2d::FREE_SPACE)
{

  unsigned int end = std::min (max_range, abs_da);
  unsigned int mx, my;
  for (unsigned int i = 0; i < end; ++i)
    {
      offset += offset_a;
      error_b += abs_db;

      _map.indexToCells (offset, mx, my);
      if (testMap (_map.getCost (mx, my), test_val))
        {
          break;
        }

      if ((unsigned int)error_b >= abs_da)
        {
          offset += offset_b;
          error_b -= abs_da;
        }

      _map.indexToCells (offset, mx, my);
      if (testMap (_map.getCost (mx, my), test_val))
        {
          break;
        }
    }

  term = offset;
}

inline void
raycast (const costmap_2d::Costmap2D &_map, unsigned int sx, unsigned int sy,
         unsigned int ex, unsigned int ey, double &x, double &y,
         unsigned int test_val = costmap_2d::FREE_SPACE,
         unsigned int max_range = 1e6)
{

  unsigned int size_x = _map.getSizeInCellsX ();

  int dx = ex - sx;
  int dy = ey - sy;

  unsigned int abs_dx = abs (dx);
  unsigned int abs_dy = abs (dy);

  int offset_dx = dx > 0 ? 1 : -1;
  int offset_dy = (dy > 0 ? 1 : -1) * size_x;

  unsigned int offset = sy * size_x + sx;

  double dist = hypot (dx, dy);
  double scale = (dist == 0.0) ? 1.0 : std::min (1.0, max_range / dist);

  unsigned int term;
  if (abs_dx >= abs_dy)
    {
      int error_y = abs_dx / 2;
      bresenham (_map, abs_dx, abs_dy, error_y, offset_dx, offset_dy, offset,
                 (unsigned int)(scale * abs_dx), term, test_val);
    }
  else
    {
      int error_x = abs_dy / 2;
      bresenham (_map, abs_dy, abs_dx, error_x, offset_dy, offset_dx, offset,
                 (unsigned int)(scale * abs_dy), term, test_val);
    }

  // convert costmap index to world coordinates
  unsigned int mx, my;
  _map.indexToCells (term, mx, my);
  _map.mapToWorld (mx, my, x, y);
}

// https://stackoverflow.com/questions/1073336/circle-line-segment-collision-detection-algorithm
inline bool
lineSphereIntersection (const Eigen::Vector2d &A, const Eigen::Vector2d &B,
                        const Eigen::Vector2d &center, const double &r,
                        Eigen::Vector2d &p)
{

  Eigen::Vector2d d = B - A;
  Eigen::Vector2d f = A - center;

  double a = d.dot (B);
  double b = 2 * f.dot (d);
  double c = f.dot (f) - r * r;

  double discriminant = b * b - 4 * a * c;
  if (discriminant < 0)
    return false;

  discriminant = sqrt (discriminant);

  double t1 = (-b - discriminant) / (2 * a);
  double t2 = (-b + discriminant) / (2 * a);

  if (t1 >= 0 && t1 <= 1)
    {
      p = A + t1 * d;
      return true;
    }

  if (t2 >= 0 && t2 <= 1)
    {
      p = A + t2 * d;
      return true;
    }

  return false;
}

inline bool
getJPSIntersectionWithSphere (const std::vector<Eigen::Vector2d> &path,
                              std::vector<Eigen::Vector2d> &modified_path,
                              const Eigen::Vector2d &center, const double &r,
                              Eigen::Vector2d &p)
{

  if (path.size () == 0)
    return false;

  modified_path.push_back (path[0]);

  for (int i = 0; i < path.size () - 1; i++)
    {

      if (lineSphereIntersection (path[i], path[i + 1], center, r, p))
        {
          modified_path.push_back (p);
          return true;
        }

      modified_path.push_back (path[i + 1]);
    }

  return false;
}

inline Eigen::MatrixX2d
getVertices (const Eigen::MatrixX4d &poly)
{
  Eigen::MatrixX2d ret (poly.rows (), 2);

  for (int i = 0; i < poly.rows (); i++)
    {
      // if()
    }

  return ret;
}

/**********************************************************************
  Function which converts a Trajectory<D> into a JointTrajectory msg.
  This discretizes the Trajectory<D> and packages it such that the
  optimizes trajectory can be broadcast over ros to an external
  tracking node.

  Inputs:
    - JointTrajectory& msg: the message to be populated
    - traj_dt: the time between each point in the trajectory
    - frame_str: the frame_id of the trajectory msg

***********************************************************************/
inline trajectory_msgs::JointTrajectory
convertTrajToMsg (const std::vector<state> &trajectory, double traj_dt,
                  std::string_view frame_str)
{

  trajectory_msgs::JointTrajectory msg;
  msg.header.stamp = ros::Time::now ();
  msg.header.frame_id = frame_str;

  double next_t = 0.;
  for (int i = 0; i < trajectory.size (); i++)
    {
      state x = trajectory[i];
      if (x.t >= next_t)
        {

          trajectory_msgs::JointTrajectoryPoint p;
          p.positions.push_back (x.pos (0));
          p.positions.push_back (x.pos (1));
          p.positions.push_back (x.pos (2));

          p.velocities.push_back (x.vel (0));
          p.velocities.push_back (x.vel (1));
          p.velocities.push_back (x.vel (2));

          p.accelerations.push_back (x.accel (0));
          p.accelerations.push_back (x.accel (1));
          p.accelerations.push_back (x.accel (2));

          p.effort.push_back (x.jerk (0));
          p.effort.push_back (x.jerk (1));
          p.effort.push_back (x.jerk (2));

          p.time_from_start = ros::Duration (x.t);

          msg.points.push_back (p);

          next_t += traj_dt;
        }
    }

  return msg;
}

inline bool
solver_boilerplate (bool simplify_jps, double max_dist_horizon,
                    const costmap_2d::Costmap2D &_map,
                    const Eigen::VectorXd &_odom, Eigen::MatrixXd &initialPVAJ,
                    Eigen::MatrixXd &finalPVAJ,
                    std::vector<Eigen::Vector2d> &jpsPath)
{
  std::cout << "solver boilerplate" << std::endl;

  jps::JPSPlan jps;
  unsigned int sX, sY, eX, eY;
  _map.worldToMap (initialPVAJ (0, 0), initialPVAJ (1, 0), sX, sY);
  _map.worldToMap (finalPVAJ (0, 0), finalPVAJ (1, 0), eX, eY);

  std::cout << "set map locations" << std::endl;

  jps.set_start (sX, sY);
  jps.set_destination (eX, eY);
  jps.set_occ_value (costmap_2d::INSCRIBED_INFLATED_OBSTACLE);

  jps.set_map (_map.getCharMap (), _map.getSizeInCellsX (),
               _map.getSizeInCellsY (), _map.getOriginX (), _map.getOriginY (),
               _map.getResolution ());

  std::cout << "set jps locations" << std::endl;
  jps.JPS ();
  std::cout << "jps finished" << std::endl;

  jpsPath = jps.getPath (simplify_jps);

  std::cout << "simplifying path" << std::endl;
  if (jpsPath.size () == 0)
    return false;

  // check if JPS path intersects with sphere
  std::vector<Eigen::Vector2d> newJPSPath;
  Eigen::Vector2d goalPoint;

  // ROS_WARN("checking sphere intersection");
  std::cout << "truncating jps" << std::endl;
  if (jps.truncateJPS (jpsPath, newJPSPath, max_dist_horizon))
    {

      jpsPath = newJPSPath;

      goalPoint = jpsPath.back ();
      if (finalPVAJ.cols () == 4)
        {
          finalPVAJ << Eigen::Vector3d (goalPoint[0], goalPoint[1], 0),
              Eigen::Vector3d::Zero (), Eigen::Vector3d::Zero (),
              Eigen::Vector3d::Zero ();
        }
      else
        {
          finalPVAJ << Eigen::Vector3d (goalPoint[0], goalPoint[1], 0),
              Eigen::Vector3d::Zero (), Eigen::Vector3d::Zero ();
        }
    }

  std::cout << "returning from solver boilerplate" << std::endl;

  return true;
}

inline void
visualizeExpectedFailuresAlongTraj (
    double threshold,
    const std::vector<std::tuple<Eigen::Vector3d, double> > &expected_failures,
    const ros::Publisher &pub)
{
  visualization_msgs::Marker marker;
  marker.header.frame_id = "map";
  marker.header.stamp = ros::Time::now ();
  marker.type = visualization_msgs::Marker::SPHERE_LIST;
  marker.action = visualization_msgs::Marker::ADD;
  marker.scale.x = 0.1;
  marker.scale.y = 0.1;
  marker.scale.z = 0.1;

  for (const auto &failure : expected_failures)
    {
      Eigen::Vector3d pos = std::get<0> (failure);
      double t = std::get<1> (failure);

      tinycolormap::Color color = tinycolormap::GetColor (
          t / threshold, tinycolormap::ColormapType::Viridis);
      std_msgs::ColorRGBA color_msg;
      color_msg.r = color.r ();
      color_msg.g = color.g ();
      color_msg.b = color.b ();
      color_msg.a = 1.0;

      geometry_msgs::Point p;
      p.x = pos[0];
      p.y = pos[1];
      p.z = pos[2];

      marker.points.push_back (p);
      marker.colors.push_back (color_msg);
    }

  pub.publish (marker);
}

inline void
visualizePolytope (const std::vector<Eigen::MatrixX4d> &hPolys,
                   const ros::Publisher &meshPub,
                   const ros::Publisher &edgePub, bool highlight = false)
{

  if (hPolys.size () == 0)
    return;

  // Due to the fact that H-representation cannot be directly visualized
  // We first conduct vertex enumeration of them, then apply quickhull
  // to obtain triangle meshs of polyhedra
  Eigen::Matrix3Xd mesh (3, 0), curTris (3, 0), oldTris (3, 0);
  for (size_t id = 0; id < hPolys.size (); id++)
    {
      oldTris = mesh;
      Eigen::Matrix<double, 3, -1, Eigen::ColMajor> vPoly;
      // geo_utils::enumerateVs(expandPoly(hPolys[id],.05), vPoly);
      geo_utils::enumerateVs (hPolys[id], vPoly);

      quickhull::QuickHull<double> tinyQH;
      const auto polyHull
          = tinyQH.getConvexHull (vPoly.data (), vPoly.cols (), false, true);
      const auto &idxBuffer = polyHull.getIndexBuffer ();
      int hNum = idxBuffer.size () / 3;

      curTris.resize (3, hNum * 3);
      for (int i = 0; i < hNum * 3; i++)
        {
          curTris.col (i) = vPoly.col (idxBuffer[i]);
        }
      mesh.resize (3, oldTris.cols () + curTris.cols ());
      mesh.leftCols (oldTris.cols ()) = oldTris;
      mesh.rightCols (curTris.cols ()) = curTris;
    }

  // RVIZ support tris for visualization
  visualization_msgs::Marker meshMarker, edgeMarker;

  meshMarker.id = 0;
  meshMarker.header.stamp = ros::Time::now ();
  meshMarker.header.frame_id = "map";
  meshMarker.pose.orientation.w = 1.00;
  meshMarker.action = visualization_msgs::Marker::ADD;
  meshMarker.type = visualization_msgs::Marker::TRIANGLE_LIST;
  meshMarker.ns = "mesh";

  meshMarker.color.r = 0.675;
  meshMarker.color.g = 0.988;
  meshMarker.color.b = .851;

  meshMarker.color.a = 0.15;
  meshMarker.scale.x = 1.0;
  meshMarker.scale.y = 1.0;
  meshMarker.scale.z = 1.0;

  edgeMarker = meshMarker;
  edgeMarker.type = visualization_msgs::Marker::LINE_LIST;
  edgeMarker.ns = "edge";

  if (highlight)
    {
      edgeMarker.color.r = 1;
      edgeMarker.color.g = 0;
      edgeMarker.color.b = 0;
    }
  else
    {
      edgeMarker.color.r = 0.365;
      edgeMarker.color.g = 0.851;
      edgeMarker.color.b = 0.757;
    }
  edgeMarker.color.a = 1.00;
  edgeMarker.scale.x = 0.02;

  geometry_msgs::Point point;

  int ptnum = mesh.cols ();

  for (int i = 0; i < ptnum; i++)
    {
      point.x = mesh (0, i);
      point.y = mesh (1, i);
      point.z = mesh (2, i);
      meshMarker.points.push_back (point);
    }

  for (int i = 0; i < ptnum / 3; i++)
    {
      for (int j = 0; j < 3; j++)
        {
          point.x = mesh (0, 3 * i + j);
          point.y = mesh (1, 3 * i + j);
          point.z = mesh (2, 3 * i + j);
          edgeMarker.points.push_back (point);
          point.x = mesh (0, 3 * i + (j + 1) % 3);
          point.y = mesh (1, 3 * i + (j + 1) % 3);
          point.z = mesh (2, 3 * i + (j + 1) % 3);
          edgeMarker.points.push_back (point);
        }
    }

  meshPub.publish (meshMarker);
  edgePub.publish (edgeMarker);

  return;
}

inline void
visualizeBoundary (const Eigen::Matrix3Xd &boundary, const ros::Publisher &pub,
                   std::string frame_id)
{
  // Create the Marker message
  visualization_msgs::Marker marker;
  marker.header.frame_id = frame_id; // Set the frame of reference (can change
                                     // based on your setup)
  marker.header.stamp = ros::Time::now ();
  marker.ns = "corridor_boundary";
  marker.id = 9476;
  marker.type
      = visualization_msgs::Marker::LINE_STRIP; // Use LINE_STRIP to connect
                                                // points sequentially
  marker.action = visualization_msgs::Marker::ADD;

  // Set the color of the line (RGBA)
  marker.color.r = 1.0; // Red
  marker.color.g = 0.0; // Green
  marker.color.b = 0.0; // Blue
  marker.color.a = 1.0; // Alpha (1 = fully visible)

  // Set the scale of the line (thickness)
  marker.scale.x = 0.05; // Line width

  // Add the points from the boundary to the marker
  for (int i = 0; i < boundary.cols (); ++i)
    {
      geometry_msgs::Point p;
      p.x = boundary (0, i); // X coordinate
      p.y = boundary (1, i); // Y coordinate
      p.z = boundary (2, i); // Z coordinate (typically 0 for 2D)
      marker.points.push_back (p);
    }

  // Close the loop of the boundary (optional)
  if (boundary.cols () > 1)
    {
      geometry_msgs::Point p;
      p.x = boundary (0, 0); // X coordinate of the first point
      p.y = boundary (1, 0); // Y coordinate of the first point
      p.z = boundary (2, 0); // Z coordinate (typically 0 for 2D)
      marker.points.push_back (p);
    }

  // Publish the marker
  pub.publish (marker);
}

inline bool
reparam_traj (std::vector<double> &ss, std::vector<double> &xs,
              std::vector<double> &ys,
              const trajectory_msgs::JointTrajectory &traj)
{
  if (traj.points.size () < 2)
    return false;

  ss.clear ();
  xs.clear ();
  ys.clear ();

  // reparam
  double cumulative_length = 0.0;
  ss.push_back (cumulative_length);

  for (size_t i = 0; i < traj.points.size (); ++i)
    {
      double x = traj.points[i].positions[0];
      double y = traj.points[i].positions[1];

      xs.push_back (x);
      ys.push_back (y);

      if (i > 0)
        {
          double dx = x - xs[i - 1];
          double dy = y - ys[i - 1];
          double segment_length = std::sqrt (dx * dx + dy * dy);

          cumulative_length += segment_length;
          ss.push_back (cumulative_length);
        }
    }

  return true;
}
