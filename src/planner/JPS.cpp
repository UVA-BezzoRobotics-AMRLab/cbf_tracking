#include <algorithm>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <math.h>
#include <string>

#include <robust_fast_navigation/JPS.h>

namespace jps
{

/**********************************************************************
  Simple constructor which sets occupied_val field to default of 100.
***********************************************************************/
JPSPlan::JPSPlan ()
{
  occupied_val = 100;
  originX = 0;
  originY = 0;
  resolution = 1;
}

/**********************************************************************
  Function to set start of JPS. The (x,y) coordinate should be in grid
  cell coordinates.

  Inputs:
    - x: starting x position
    - y: starting y position
***********************************************************************/
void
JPSPlan::set_start (int x, int y)
{
  startX = x;
  startY = y;
}

/**********************************************************************
  Function to set destination of JPS. The (x,y) coordinate should be in
  grid cell coordinates.

  Inputs:
    - x: destination x position
    - y: destination y position
***********************************************************************/
void
JPSPlan::set_destination (int x, int y)
{
  destX = x;
  destY = y;
}

/**********************************************************************
  Below are a collection of heuristics for the grid search. I've found
  through experimentation that manhattan works best, but due to
  geometric setup of environment, it is not an admisisble heuristic and
  thus doesn't give an optimal solution always.

  Inputs:
    - x: current x position
    - y: current y position

  Returns:
    - Cost to destination coordinate
***********************************************************************/
double
JPSPlan::chebyshev_dist (int sx, int sy, int ex, int ey)
{
  // std::cout << "chebyshev dist from " << sx << ", " << sy << " to " << ex <<
  // ", " << ey << "is " << std::max(abs(ex-sx), abs(ey-sy)) << std::endl;
  // std::cout << abs(ex-sx) << "\t" << abs(ey-sy) << "\t" <<
  // std::max(abs(ex-sx), abs(sy-ey)) << std::endl;
  return std::max (abs (ex - sx), abs (ey - sy));
}

// im going to assume we never go over max_int number of cells in either
// direction...
double
JPSPlan::manhattan_distance (int x, int y)
{
  return abs (destX - x) + abs (destY - y);
}

double
JPSPlan::octile_dist (int x, int y)
{
  int dx = abs (destX - x);
  int dy = abs (destY - y);
  return std::max (dx, dy) * (sqrt (2)) + std::min (dx, dy);
}

double
JPSPlan::euclidean_dist (int x, int y)
{
  return sqrt ((x - destX) * (x - destX) + (y - destY) * (y - destY));
}

/**********************************************************************
  Set the value which the JPS should use as occupied in the grid. For
  traditional maps/costmaps this value is usually 100 or 253/254 (in
  ROS at least).

  Inputs:
    - x: value which indicates occupancy in the grid.
***********************************************************************/
void
JPSPlan::set_occ_value (double x)
{
  occupied_val = x;
}

/**********************************************************************
  This function pushes a node onto the priority queue, where cost is
  based on the cost-to-go heuristic function of the node in question.
  To see the comparator for and instantiation of the priority queue,
  see JPS.h.

  Inputs:
    - x: x grid cell coordinate of the node
    - y: y grid cell coordinate of the node
    - dirx: which direction in x is the node searching in. This
      determines whether the node will use explore_straight or
      explore_diagonal when popped off queue.
    - diry: which direction in y is the node searching in.
    - cost: heuristic cost-to-go associated with the node.
***********************************************************************/
void
JPSPlan::add_to_queue (int x, int y, int dirx, int diry, double cost)
{

  static int count = 0;

  if (closedSet.find (y * sizeX + x) != closedSet.end ())
    return;

  // std::cout << "adding to queue (" << x << ", " << y << ", " << dirx << ", "
  // << diry << ")" << std::endl;

  JPSNode_t node;
  node.x = x;
  node.y = y;
  node.dirx = dirx;
  node.diry = diry;
  node.cost = cost;
  node.manhattan = manhattan_distance (x, y); // octile_dist(x,y);
  q.push (node);
  count += 1;
}

bool
JPSPlan::add_to_parents (const JPSNode_t &node, const JPSNode_t &parent,
                         double cost)
{
  // check if node is not in parents
  if (parents.find (node.y * sizeX + node.x) == parents.end ())
    {
      parents[node.y * sizeX + node.x]
          = std::make_pair (parent.y * sizeX + parent.x, cost);
      return true;
    }

  // node is in parents, but check if cost is lower
  if (cost < parents[node.y * sizeX + node.x].second)
    {
      parents[node.y * sizeX + node.x]
          = std::make_pair (parent.y * sizeX + parent.x, cost);
    }
  else
    return false;

  return true;
}

/**********************************************************************
  This function performs jumps in the grid in one of the 4 cardinal
  directions starting at the given node. When this function is called,
  the node is marked visited. Based on neighbor rules, new nodes will
  be added to the open set (priority queue) as they are discovered.

  Inputs:
    - start: JPSNode_t that was popped off the queue.

  Returns:
    - True if a jump point has been found and added to queue.
    - False otherwise
***********************************************************************/
bool
JPSPlan::explore_straight (const JPSNode_t &start)
{

  int dirx = start.dirx;
  int diry = start.diry;

  // start cost is simply parent cost + # cells between parent and start node
  // # cells between parent and start node can be found with chebyshev distance
  int parentInd = parents[start.y * sizeX + start.x].first;
  int py = parentInd / sizeX;
  int px = parentInd - py * sizeX;

  double cost = parents[start.y * sizeX + start.x].second
                + chebyshev_dist (px, py, start.x, start.y);
  // std::cout << "cost to " << start.x << ", " << start.y << " is " << cost <<
  // std::endl; std::cout << "parent cost is " << parents[start.y*sizeX +
  // start.x].second << std::endl;

  // if (cost > 0)
  //     exit(0);
  JPSNode_t curr = start;

  closedSet[start.y * sizeX + start.x] = true;
  // std::cout << "explore straight (" << start.x << ", " << start.y << ", " <<
  // dirx << ", " << diry << ")" << std::endl;

  while (true)
    {
      JPSNode_t n = curr;
      n.x += dirx;
      n.y += diry;
      cost += 1;
      // if(start.x == 21 && start.y == 26)
      //     //std::cout << "[straight](" << curr.x << ", " << curr.y << ")" <<
      //     std::endl;
      // is node on border?
      if ((n.x > sizeX - 1 && dirx == 1) || (n.y > sizeY - 1 && diry == 1)
          || (n.x < 0 && dirx == -1) || (n.y < 0 && diry == -1))
        {

          return false;
        }

      if (_map[n.y * sizeX + n.x] == occupied_val)
        {
          return false;
        }

      if (n.x == destX && n.y == destY)
        {
          // std::cout << "GOAL FOUND" << std::endl;
          //  parents[n.y*sizeX + n.x] = start.y*sizeX + start.x;
          add_to_parents (n, start, cost);
          add_to_queue (n.x, n.y, n.dirx, n.diry, cost);
          return true;
        }

      // only one of dirx or diry can be non-zero in this function call
      bool added = false;
      if (dirx != 0)
        {
          if (n.y != sizeY - 1 && _map[(n.y + 1) * sizeX + n.x] == occupied_val
              && _map[(n.y + 1) * sizeX + n.x + dirx] != occupied_val)
            {
              // std::cout << "found forced at (" << n.x << "," << n.y << ")\t"
              // << dirx << "\t1" << std::endl;
              add_to_queue (n.x, n.y, dirx, 1, cost);
              added = true;
            }

          if (n.y != 0 && _map[(n.y - 1) * sizeX + n.x] == occupied_val
              && _map[(n.y - 1) * sizeX + n.x + dirx] != occupied_val)
            {
              // std::cout << "found forced at (" << n.x << "," << n.y << ")\t"
              // << dirx << "\t-1" << std::endl;
              add_to_queue (n.x, n.y, dirx, -1, cost);
              added = true;
            }
        }

      else
        {
          if (n.x != sizeX - 1 && _map[n.y * sizeX + n.x + 1] == occupied_val
              && _map[(n.y + diry) * sizeX + n.x + 1] != occupied_val)
            {
              // std::cout << "found forced at (" << n.x << "," << n.y <<
              // ")\t1\t" << diry << std::endl;
              add_to_queue (n.x, n.y, 1, diry, cost);
              added = true;
            }

          if (n.x != 0 && _map[n.y * sizeX + n.x - 1] == occupied_val
              && _map[(n.y + diry) * sizeX + n.x - 1] != occupied_val)
            {
              // std::cout << "found forced at (" << n.x << "," << n.y <<
              // ")\t-1\t" << diry << std::endl;
              add_to_queue (n.x, n.y, -1, diry, cost);
              added = true;
            }
        }

      // if node has any forced neighbords, it is a jump point, so add to queue
      // as well
      if (added)
        {

          // parents[n.y*sizeX + n.x] = start.y*sizeX + start.x;
          add_to_parents (n, start, cost);
          // std::cout << "straight found jump: parent of (" <<n.x<< ","
          // <<n.y<< ") is (" <<start.x<< "," <<start.y<< ")" << std::endl;
          add_to_queue (n.x, n.y, n.dirx, n.diry, cost);
          return true;
        }

      // might need this, not sure at the moment...
      // closedSet[n.y*sizeX + n.x] = true;
      curr.x = n.x;
      curr.y = n.y;
      curr.cost = cost;
    }
}

/**********************************************************************
  This function performs jumps in the grid in the diagonal direction.
  starting at the given node. When this function is called, the node is
  marked visited. Based on neighbor rules, new nodes will be added to
  the open set (priority queue) as they are discovered. Given JPS rules,
  this function will also kick off an explore_straight in the dirx and
  diry directions at each node considered.

  Inputs:
    - start: JPSNode_t that was popped off the queue.

  Returns:
    - True if a jump point has been found and added to queue.
    - False otherwise
***********************************************************************/
bool
JPSPlan::explore_diagonal (const JPSNode_t &start)
{

  int dirx = start.dirx;
  int diry = start.diry;

  // start cost is simply parent cost + # cells between parent and start node
  // # cells between parent and start node can be found with chebyshev distance
  int parentInd = parents[start.y * sizeX + start.x].first;
  int py = parentInd / sizeX;
  int px = parentInd - py * sizeX;

  double cost = parents[start.y * sizeX + start.x].second
                + chebyshev_dist (px, py, start.x, start.y);
  // std::cout << "(diag) cost to " << start.x << ", " << start.y << " is " <<
  // cost << std::endl; std::cout << "(diag) parent cost is " <<
  // parents[start.y*sizeX + start.x].second << std::endl;

  // if (cost > 0)
  //     exit(0);
  JPSNode_t curr = start;
  closedSet[start.y * sizeX + start.x] = true;

  // std::cout << "explore diagonal (" << start.x << ", " << start.y << ", " <<
  // dirx << ", " << diry << ")" << std::endl;

  while (true)
    {
      JPSNode_t n = curr;
      n.x += dirx;
      n.y += diry;
      cost += 1; // sqrt(2);

      // //std::cout << "[diagonal] curr is: " << "(" << curr.x << ", " <<
      // curr.y << ")" << std::endl;

      if ((n.x > sizeX - 1 && dirx == 1) || (n.y > sizeY - 1 && diry == 1)
          || (n.x < 0 && dirx == -1) || (n.y < 0 && diry == -1))
        {
          return false;
        }

      if (_map[n.y * sizeX + n.x] == occupied_val)
        {
          return false;
        }

      if (n.x == destX && n.y == destY)
        {
          // std::cout << "GOAL FOUND" << std::endl;
          add_to_parents (n, start, cost);
          // parents[n.y*sizeX + n.x] = start.y*sizeX + start.x;
          // std::cout << "diag found jump: parent of (" <<n.x<< "," <<n.y<< ")
          // is (" <<start.x<< "," <<start.y<< ")" << std::endl;
          add_to_queue (n.x, n.y, n.dirx, n.diry, cost);
          return true;
        }

      bool added = false;
      if (_map[n.y * sizeX + curr.x] == occupied_val
          && _map[(n.y + diry) * sizeX + curr.x] != occupied_val)
        {
          // std::cout << "JP @ (" << n.x << "," << n.y << ")" << std::endl;
          add_to_parents (n, start, cost);
          // parents[n.y*sizeX + n.x] = start.y*sizeX + start.x;
          // std::cout << "diag found jump: parent of (" <<n.x<< "," <<n.y<< ")
          // is (" <<start.x<< "," <<start.y<< ")" << std::endl;
          add_to_queue (n.x, n.y, -n.dirx, n.diry, cost);
          added = true;
        }

      if (_map[curr.y * sizeX + n.x] == occupied_val
          && _map[curr.y * sizeX + n.x + dirx] != occupied_val)
        {
          // std::cout << "JP @ (" << n.x << "," << n.y << ")" << std::endl;
          add_to_parents (n, start, cost);
          // parents[n.y*sizeX + n.x] = start.y*sizeX + start.x;
          // std::cout << "diag found jump: parent of (" <<n.x<< "," <<n.y<< ")
          // is (" <<start.x<< "," <<start.y<< ")" << std::endl;
          add_to_queue (n.x, n.y, n.dirx, -n.diry, cost);
          added = true;
        }

      add_to_parents (n, start, cost);
      JPSNode_t horN;
      horN.x = n.x;
      horN.y = n.y;
      horN.dirx = dirx;
      horN.diry = 0;
      horN.cost = cost;
      bool found_hor = explore_straight (horN);

      JPSNode_t verN;
      verN.x = n.x;
      verN.y = n.y;
      verN.dirx = 0;
      verN.diry = diry;
      verN.cost = cost;
      bool found_ver = explore_straight (verN);

      // moving diagonally
      curr.x = n.x;
      curr.y = n.y;
      curr.cost = cost;
    }

  // std::cout << "**************************" << std::endl;
}

/**********************************************************************
  This function starts off the actual JPS and terminates when the goal
  has been found or all grid cells have been examined. To start the
  JPS, add the start coordinate to the queue in all possible directions.
***********************************************************************/
void
JPSPlan::JPS ()
{

  closedSet.clear ();
  parents.clear ();

  goalInd = -1;

  if (_map[startY * sizeX + startX] == occupied_val)
    {
      std::cerr << "start is in occupied space" << std::endl;
      return;
    }

  if (_map[destY * sizeX + destX] == occupied_val)
    {
      std::cerr << "destination is in occupied space" << std::endl;
      return;
    }

  // std::cout << "start " << (int) _map[startY*sizeX + startX] << " and
  // destination " << (int)_map[destY*sizeX + destX] << std::endl;

  // cardinal
  add_to_queue (startX, startY, 1, 0, 0);
  add_to_queue (startX, startY, -1, 0, 0);
  add_to_queue (startX, startY, 0, 1, 0);
  add_to_queue (startX, startY, 0, -1, 0);

  // diagonal
  add_to_queue (startX, startY, 1, 1, 0);
  add_to_queue (startX, startY, 1, -1, 0);
  add_to_queue (startX, startY, -1, 1, 0);
  add_to_queue (startX, startY, -1, -1, 0);

  parents[startY * sizeX + startX]
      = std::make_pair (startY * sizeX + startX, 0);

  while (q.size () > 0)
    {
      JPSNode_t node = q.top ();
      q.pop ();

      // std::cout << "LOOKING AT NODE (" << node.x << ", " << node.y <<
      // ", " << node.dirx << ", " << node.diry << ", " <<
      // node.cost+node.manhattan << ")" << std::endl;

      if (node.x == destX && node.y == destY)
        {
          goalInd = destY * sizeX + destX;
          break;
        }

      if (node.dirx == 0 || node.diry == 0)
        {
          explore_straight (node);
        }
      else
        {
          explore_diagonal (node);
        }
    }
}

/**********************************************************************
  This function goes through the parents[] field and finds the path
  from destination to start, then reverses the path and stores the
  jump points in a vector of Eigen::Vector2d. Finally, extraneous jump
  points are removed from the path before returning.

  Inputs:
    - simplify: If true, the path will be simplified

  Returns:
    - vector of Eigen::Vector2d which denote the final JPS path
***********************************************************************/
std::vector<Eigen::Vector2d>
JPSPlan::getPath (bool simplify)
{

  std::vector<Eigen::Vector2d> ret;

  if (goalInd == -1)
    return ret;

  int x, y;
  y = goalInd / sizeX;
  x = goalInd - y * sizeX;
  ret.push_back (Eigen::Vector2d (x, y));

  int i = parents[goalInd].first;
  int j = 0;

  while (i != startY * sizeX + startX && j++ < 1000)
    {
      // std::cout << "(" << i-((int)(i/sizeX))*sizeX << "," << i/sizeX << ")
      // -->";
      y = i / sizeX;
      x = i - y * sizeX;
      ret.push_back (Eigen::Vector2d (x, y));
      i = parents[i].first;
    }

  // std::cout << "(" << i-((int)(i/sizeX))*sizeX << "," << i/sizeX << ")\n";
  unsigned int startInd = startY * sizeX + startX;
  int ax, ay;
  ay = startInd / sizeX;
  ax = startInd - ay * sizeX;
  // std::cout << "parents[startInd] " << ax << ", " << ay << std::endl;

  y = i / sizeX;
  x = i - y * sizeX;
  ret.push_back (Eigen::Vector2d (x, y));
  std::reverse (ret.begin (), ret.end ());

  if (simplify)
    {

      ret = simplifyPath (ret);
      std::reverse (std::begin (ret), std::end (ret));
      // std::cout << "path after first simplify is" << std::endl;
      // for(int i = 0; i < ret.size(); i++)
      // {
      //     std::cout << ret[i][0] << ", " << ret[i][1] << std::endl;
      // }

      ret = simplifyPath (ret);
      std::reverse (std::begin (ret), std::end (ret));

      // std::cout << "path after second simplify is" << std::endl;
      // for(int i = 0; i < ret.size(); i++)
      // {
      //     std::cout << ret[i][0] << ", " << ret[i][1] << std::endl;
      // }

      // Logic from JPS3D
      for (int k = 1; k < ret.size () - 1;)
        {
          Eigen::Vector2d p = (ret[k + 1] - ret[k]) - (ret[k] - ret[k - 1]);
          if (fabs (p[0]) + fabs (p[1]) <= 1e-2)
            ret.erase (ret.begin () + i);
          else
            k++;
        }
    }

  for (int k = 0; k < ret.size (); k++)
    {
      double x, y;
      mapToWorld (ret[k][0], ret[k][1], x, y);
      ret[k] = Eigen::Vector2d (x, y);
    }

  return ret;
}

/**********************************************************************
  This function sets the map that will be used in the grid search.
  Traditionally these maps are stored as unsigned char* but in the case
  that there are negative numbers, change to signed char* or int* array.

  Inputs:
    - map in which grid search will be performed on
    - size of the map in both the x and y directions
***********************************************************************/
void
JPSPlan::set_map (unsigned char *map, int sizeX, int sizeY, double originX,
                  double originY, double resolution)
{
  this->_map = map;
  this->sizeX = sizeX;
  this->sizeY = sizeY;
  this->originX = originX;
  this->originY = originY;
  this->resolution = resolution;
}

/**********************************************************************
  This function removes "corners" that appear in the JPS around the
  borders of obstacles. This function was taken and modified slightly
  from the folks at KumarRobotics: https://github.com/KumarRobotics/jps3d

  Inputs:
    - JPS path to be simplified

  Returns:
    - Simplified JPS path
***********************************************************************/
std::vector<Eigen::Vector2d>
JPSPlan::simplifyPath (const std::vector<Eigen::Vector2d> &path)
{
  if (path.size () < 2)
    return path;

  // cut zigzag segment
  std::vector<Eigen::Vector2d> optimized_path;

  Eigen::Vector2d pose1 = path[0];
  Eigen::Vector2d pose2 = path[1];
  Eigen::Vector2d prev_pose = pose1;

  optimized_path.push_back (pose1);

  double cost1, cost2, cost3;

  if (!isBlocked (pose1, pose2))
    cost1 = (pose1 - pose2).norm ();
  else
    cost1 = std::numeric_limits<double>::infinity ();

  for (unsigned int i = 1; i < path.size () - 1; i++)
    {
      pose1 = path[i];
      pose2 = path[i + 1];
      if (!isBlocked (pose1, pose2))
        cost2 = (pose1 - pose2).norm ();
      else
        cost2 = std::numeric_limits<double>::infinity ();

      if (!isBlocked (prev_pose, pose2))
        cost3 = (prev_pose - pose2).norm ();
      else
        cost3 = std::numeric_limits<double>::infinity ();

      if (cost3 < cost1 + cost2)
        cost1 = cost3;
      else
        {
          optimized_path.push_back (path[i]);
          cost1 = (pose1 - pose2).norm ();
          prev_pose = pose1;
        }
    }

  optimized_path.push_back (path.back ());

  return optimized_path;
}

bool
JPSPlan::truncateJPS (const std::vector<Eigen::Vector2d> &path,
                      std::vector<Eigen::Vector2d> &resultPath, double d)
{
  if (path.size () < 2)
    {
      // Not enough points to create a path
      return false;
    }

  double remainingDistance = d;
  Eigen::Vector2d resultPoint;
  resultPath.clear ();
  resultPath.push_back (path[0]); // Start point of the new path

  for (size_t i = 1; i < path.size (); ++i)
    {
      double segmentLength = (path[i] - path[i - 1]).norm ();

      if (remainingDistance <= 0)
        {
          // Requested distance is longer than the entire path
          return false;
        }

      if (remainingDistance <= segmentLength)
        {
          // The target point lies on this segment
          double t = remainingDistance / segmentLength;
          resultPoint = (1 - t) * path[i - 1] + t * path[i];
          resultPath.push_back (resultPoint);
          return true;
        }

      resultPath.push_back (path[i]); // Add entire segment to the new path
      remainingDistance -= segmentLength;
    }

  return false; // Requested distance is longer than the entire path
}

/**********************************************************************
  This function checks if there is an occupied grid cell between two
  points in the grid. Notice that the input points can be in grid cell
  coordinate space or world frame coordinate space depending on the
  map_coords parameter.

  Inputs:
    - First point of line to be checked
    - Second point of line to be check
    - Maximum range for bresenham to check before terminating
    - Map coords flag indicating if incoming points are in map cell
      coordinates or in world coordinates

  Returns:
    - True if an obstacle blocks the line connecting the two points
    - False if line is obstacle free
***********************************************************************/
bool
JPSPlan::isBlocked (const Eigen::Vector2d &p1, const Eigen::Vector2d &p2,
                    double max_range, bool map_coords)
{

  // raycast and bresenham
  unsigned int size_x = this->sizeX;

  unsigned int sx, sy, ex, ey;
  if (map_coords)
    {
      sx = p1[0];
      sy = p1[1];
      ex = p2[0];
      ey = p2[1];
    }
  else
    {
      if (!worldToMap (p1[0], p1[1], sx, sy))
        {
          std::cout << "point " << sx << ", " << sy << " outside map"
                    << std::endl;
          return false;
        }

      if (!worldToMap (p2[0], p2[1], ex, ey))
        {
          std::cout << "point " << ex << ", " << ey << " outside map"
                    << std::endl;
          return false;
        }
    }

  int dx = ex - sx;
  int dy = ey - sy;

  unsigned int abs_dx = abs (dx);
  unsigned int abs_dy = abs (dy);

  int offset_dx = dx > 0 ? 1 : -1;
  int offset_dy = (dy > 0 ? 1 : -1) * this->sizeX;

  unsigned int offset = sy * size_x + sx;
  double dist = hypot (dx, dy);
  double scale = (dist == 0.0) ? 1.0 : std::min (1.0, max_range / dist);

  unsigned int term;
  if (abs_dx >= abs_dy)
    {
      int error_y = abs_dx / 2;
      return bresenham (abs_dx, abs_dy, error_y, offset_dx, offset_dy, offset,
                        (unsigned int)(scale * abs_dx), term);
    }
  else
    {
      int error_x = abs_dy / 2;
      return bresenham (abs_dy, abs_dx, error_x, offset_dy, offset_dx, offset,
                        (unsigned int)(scale * abs_dy), term);
    }
}

/**********************************************************************
  This function implements the Bresenham algorithm and was taken and
  modified from the folks at Costmap2D.

  Inputs:

  Returns:
    - True if bresenham hits an obstacle along the path between two pts
    - False if path is obstacle free
***********************************************************************/
bool
JPSPlan::bresenham (unsigned int abs_da, unsigned int abs_db, int error_b,
                    int offset_a, int offset_b, unsigned int offset,
                    unsigned int max_range, unsigned int &term)
{

  unsigned int end = std::min (max_range, abs_da);
  unsigned int mx, my;

  bool is_hit = false;
  for (unsigned int i = 0; i < end; ++i)
    {
      offset += offset_a;
      error_b += abs_db;

      if (_map[offset] == occupied_val)
        {
          is_hit = true;
          break;
        }

      if ((unsigned int)error_b >= abs_da)
        {
          offset += offset_b;
          error_b -= abs_da;
        }

      if (_map[offset] == occupied_val)
        {
          is_hit = true;
          break;
        }
    }

  term = offset;
  return is_hit;
}

bool
JPSPlan::worldToMap (double x, double y, unsigned int &mx, unsigned int &my)
{
  mx = (int)((x - originX) / resolution);
  my = (int)((y - originY) / resolution);

  if (mx >= sizeX || my >= sizeY)
    return false;

  return true;
}

void
JPSPlan::mapToWorld (unsigned int mx, unsigned int my, double &x, double &y)
{
  x = originX + (mx + .5) * resolution;
  y = originY + (my + .5) * resolution;
}

} // end namespace jps
