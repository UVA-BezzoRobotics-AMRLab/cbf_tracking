#ifndef SOLVER_UTILS_H
#define SOLVER_UTILS_H

#include <Eigen/Dense>
#include "gurobi_c++.h"

std::vector<GRBLinExpr> gurobiMatMul(const Eigen::MatrixXd& A, const std::vector<GRBLinExpr>& x){
    std::vector<GRBLinExpr> res;

    for(int i = 0; i < A.rows(); i++){
        
        GRBLinExpr exp = 0;
        for(int j = 0; j < x.size(); j++)
            exp += A(i,j)*x[j];

        res.push_back(exp);
    }
    return res;
}

std::vector<std::vector<GRBLinExpr> > gurobiMatMul(const std::vector<std::vector<GRBLinExpr> >& A, const Eigen::MatrixXd& B){
    std::vector<std::vector<GRBLinExpr> > res(A.size());

    for(int i = 0; i < A.size(); i++){
        res[i] = std::vector<GRBLinExpr>(B.cols(), 0.0);
        for(int j = 0; j < B.cols(); j++){
            GRBLinExpr entry = 0;
            for(int k = 0; k < B.rows(); k++)
                entry += A[i][k]*B(k,j);

            res[i][j] = entry;
        }
    }

    return res;
}

GRBQuadExpr gurobiNorm(const std::vector<GRBLinExpr>& vec){
    GRBQuadExpr res = 0;

    for(int i = 0; i < vec.size(); i++)
        res += vec[i]*vec[i];

    return res;
}

#endif
