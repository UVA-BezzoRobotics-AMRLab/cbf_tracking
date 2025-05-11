#ifndef JPS3D_H
#define JPS3D_H

#include <queue>
#include <vector>
#include <memory>
#include <utility>
#include <unordered_map>

#include <octomap/OcTree.h>

#include <Eigen/Core>

struct JPS3DNode{
    bool isValid = true;
    int x, y, z;

    int dirx, diry, dirz;

    double cost, heuristic;

}; typedef JPS3DNode JPS3DNode_t;

// taken from https://github.com/KumarRobotics/jps3d/
struct JPS3DNeib{
    
    JPS3DNeib();
    // for each (dx,dy,dz) these contain:
    //    ns: neighbors that are always added
    //    f1: forced neighbors to check
    //    f2: neighbors to add if f1 is forced
    int ns[27][3][26];
    int f1[27][3][12];
    int f2[27][3][12];
    // nsz contains the number of neighbors for the four different types of moves:
    // no move (norm 0):        26 neighbors always added
    //                          0 forced neighbors to check (never happens)
    //                          0 neighbors to add if forced (never happens)
    // straight (norm 1):       1 neighbor always added
    //                          8 forced neighbors to check
    //                          8 neighbors to add if forced
    // diagonal (norm sqrt(2)): 3 neighbors always added
    //                          8 forced neighbors to check
    //                          12 neighbors to add if forced
    // diagonal (norm sqrt(3)): 7 neighbors always added
    //                          6 forced neighbors to check
    //                          12 neighbors to add if forced
    static constexpr int nsz[4][2] = {{26, 0}, {1, 8}, {3, 12}, {7, 12}};
    private:
    void Neib(int dx, int dy, int dz, int norm1, int dev, int& tx, int& ty, int& tz);
    void FNeib( int dx, int dy, int dz, int norm1, int dev,
        int& fx, int& fy, int& fz,
        int& nx, int& ny, int& nz);
};

class Compare{
    public:
    // returns True if first argument is higher priority than n2 (lower cost)
    bool operator()(const JPS3DNode_t& n1, const JPS3DNode_t& n2){
        return n1.cost+n1.heuristic > n2.cost+n2.heuristic;
    }
};

class JPS3DPlan{

public:
    JPS3DPlan();
    
    void set_occ_value(double x);
    void set_start(double x, double y, double z);
    void set_destination(double x, double y, double z);
    void set_map(const std::shared_ptr<octomap::OcTree> &octree);
    void JPS();

    std::vector<Eigen::Vector2d> getPath(bool simplify = true);
    std::vector<Eigen::Vector2d> simplifyPath(const std::vector<Eigen::Vector2d>& path);
    bool worldToMap(double x, double y, unsigned int& mx, unsigned int& my);
    void mapToWorld(unsigned int mx, unsigned int my, double& x, double& y);


private:

    double euclidean_dist(int x, int y, int z);

    bool isOccupied(int x, int y, int z);

    // bool explore_straight(const JPS3DNode_t& start);
    // bool explore_diagonal(const JPS3DNode_t& start);
    bool jump(const JPS3DNode_t& start);
    void add_to_queue(int x, int y, int z, int dirx, int diry, int dirz, double cost);
    bool add_to_parents(const JPS3DNode_t& node, const JPS3DNode_t& parent, double cost);
    bool get_map_key(double x, double y, double z, std::vector<int>& key);

    bool bresenham(unsigned int abs_da, unsigned int abs_db, int error_b, int offset_a,
        int offset_b, unsigned int offset, unsigned int max_range, unsigned int& term);
    bool isBlocked(const Eigen::Vector2d& p1, const Eigen::Vector2d& p2, 
                    double max_range=1e6, bool map_coords=true);

    std::unordered_map<int, bool> closedSet;
    std::unordered_map<int, std::pair<int, int>> parents;

    unsigned char* _map;

    int sizeX, sizeY, sizeZ, startX, startY, startZ, destX, destY, destZ, goalInd;
    double occupied_val, originX, originY, originZ, resolution,
        minx, miny, minz, maxx, maxy, maxz;

    std::priority_queue<JPS3DNode_t, std::deque<JPS3DNode_t>, Compare> q;

    JPS3DNeib jps3d_neib;

    std::shared_ptr<octomap::OcTree> octree_;
};

#endif
