#pragma once
#include <vector>
#include <cppad/ipopt/solve.hpp>

class SafetyFilterUnicycle
{
public:
    SafetyFilterUnicycle();

    std::vector<double> filter_x;
    std::vector<double> filter_y;

    std::vector<double> filter(const std::vector<double> &x,
                               const std::vector<double> &v,
                               const std::vector<double> &u,
                               const std::vector<double> &obstacle);

protected:
    int _w_idx;
};
