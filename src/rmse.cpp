//
// Created by Oleg Leyzerov on 30/03/2017.
//

#include "rmse.h"
#include <vector>
#include <iostream>
#include "Eigen/Dense"

using Eigen::VectorXd;
using std::vector;

VectorXd CalculateRMSE(const vector<VectorXd> &estimations,
                       const vector<VectorXd> &ground_truth) {
    size_t N = estimations.size();
    VectorXd RMSE(4);
    RMSE << 0,0,0,0;

    if (estimations.size() != 0 && estimations.size() == ground_truth.size()) {
        for (size_t i = 0; i < N; i++) {
            VectorXd residual;
            residual = estimations[i] - ground_truth[i];
            residual = residual.array() * residual.array();
            RMSE += residual;
        }
        RMSE = (RMSE/N).array().sqrt();
        return RMSE;
    } else {
        std::cout << "Error: the size of input arrays is out of range" << std::endl;
    }

}

