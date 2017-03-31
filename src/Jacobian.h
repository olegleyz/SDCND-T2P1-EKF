//
// Created by Oleg Leyzerov on 30/03/2017.
//

#ifndef KALMAN2D_JACOBIAN_H
#define KALMAN2D_JACOBIAN_H

#include "Eigen/Dense"

using Eigen::MatrixXd;
using Eigen::VectorXd;

MatrixXd Jacobian(const VectorXd &z);

#endif //KALMAN2D_JACOBIAN_H
