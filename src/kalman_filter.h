//
// Created by Oleg Leyzerov on 29/03/2017.
//

#ifndef KALMAN2D_KALMAN_FILTER_H
#define KALMAN2D_KALMAN_FILTER_H

#include "Eigen/Dense"

using Eigen::MatrixXd;
using Eigen::VectorXd;

class KalmanFilter {
public:
    VectorXd x_; // state vector
    MatrixXd P_; // state covariance matrix
    MatrixXd F_; // state transition matrix
    MatrixXd Q_; // process covariance matrix
    MatrixXd H_; // measurement matrix
    MatrixXd R_; // measurement (LIDAR) covariance matrix
    MatrixXd Rr_; // measurement (RADAR) covariance matrix

    KalmanFilter(); // constructor
    virtual ~KalmanFilter(); // destructor

    void Predict(); // predicts state and state covariance using the process model
    void Update(const VectorXd &z); // updates the state and param z at k+1
    void UpdateRadar(const VectorXd &z); // measurement update for Radar measurements
};

#endif //KALMAN2D_KALMAN_FILTER_H
