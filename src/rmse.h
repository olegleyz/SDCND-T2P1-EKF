//
// Created by Oleg Leyzerov on 30/03/2017.
//

#ifndef KALMAN2D_RMSE_H
#define KALMAN2D_RMSE_H

#include <vector>
#include "Eigen/Dense"

using Eigen::VectorXd;
using std::vector;

VectorXd CalculateRMSE(const vector<VectorXd> &estimations,
                       const vector<VectorXd> &ground_truth);

#endif //KALMAN2D_RMSE_H
