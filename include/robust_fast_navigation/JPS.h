#pragma once

#include <queue>
#include <unordered_map>
#include <utility>
#include <vector>

#include <Eigen/Core>

namespace jps
{

struct JPSNode
{
  bool isValid = true;
  int x, y;

  int dirx, diry;

  double cost, manhattan;
};
typedef JPSNode JPSNode_t;

class Compare
{
public:
  // returns True if first argument is higher priority than n2 (lower cost)
  bool
  operator() (const JPSNode_t &n1, const JPSNode_t &n2)
  {
    return n1.cost + n1.manhattan > n2.cost + n2.manhattan;
  }
};

class JPSPlan
{

public:
  JPSPlan ();

  void set_occ_value (double x);
  void set_start (int x, int y);
  void set_destination (int x, int y);
  void set_map (unsigned char *map, int sizeX, int sizeY, double originX,
                double originY, double resolution);
  void JPS ();

  std::vector<Eigen::Vector2d> getPath (bool simplify = true);
  std::vector<Eigen::Vector2d>
  simplifyPath (const std::vector<Eigen::Vector2d> &path);
  bool truncateJPS (const std::vector<Eigen::Vector2d> &path,
                    std::vector<Eigen::Vector2d> &resultPath, double d);
  bool worldToMap (double x, double y, unsigned int &mx, unsigned int &my);
  void mapToWorld (unsigned int mx, unsigned int my, double &x, double &y);

private:
  double chebyshev_dist (int sx, int sy, int ex, int ey);
  double manhattan_distance (int x, int y);
  double octile_dist (int x, int y);
  double euclidean_dist (int x, int y);

  bool explore_straight (const JPSNode_t &start);
  bool explore_diagonal (const JPSNode_t &start);
  void add_to_queue (int x, int y, int dirx, int diry, double cost);
  bool add_to_parents (const JPSNode_t &node, const JPSNode_t &parent,
                       double cost);

  bool bresenham (unsigned int abs_da, unsigned int abs_db, int error_b,
                  int offset_a, int offset_b, unsigned int offset,
                  unsigned int max_range, unsigned int &term);
  bool isBlocked (const Eigen::Vector2d &p1, const Eigen::Vector2d &p2,
                  double max_range = 1e6, bool map_coords = true);

  std::unordered_map<int, bool> closedSet;
  std::unordered_map<int, std::pair<int, int> > parents;

  unsigned char *_map;

  int sizeX, sizeY, startX, startY, destX, destY, goalInd;
  double occupied_val, originX, originY, resolution;

  std::priority_queue<JPSNode_t, std::deque<JPSNode_t>, Compare> q;
};

} // end namespace jps
