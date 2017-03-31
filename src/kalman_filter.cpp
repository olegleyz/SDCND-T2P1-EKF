//
// Created by Oleg Leyzerov on 29/03/2017.
//
#include <iostream>
#include "kalman_filter.h"
#include "jacobian.h"
#define PI 3.14159265

KalmanFilter::KalmanFilter() {
}
KalmanFilter::~KalmanFilter() {
}

void KalmanFilter::Predict() {
    x_ = F_ * x_;
    P_ = F_ * P_ * F_.transpose() + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
    VectorXd z_pred = H_ * x_;
    VectorXd y = z - z_pred;
    MatrixXd Ht = H_.transpose();
    MatrixXd S = H_ * P_ * Ht + R_;
    MatrixXd K = P_ * Ht * S.inverse();
    x_ = x_ + K * y;
    long x_size = x_.size();
    MatrixXd I_ = MatrixXd::Identity(x_size, x_size);
    P_ = (I_ - K * H_) * P_;
}

void KalmanFilter::UpdateRadar(const VectorXd &z) {
    if (x_[0] == 0 && x_[1] == 0) {
        x_[0] = 0.001;
        x_[1] = 0.001;
    }
    VectorXd h_(3);
    float px, px2, py, py2, sps, vx, vy;
    px = x_[0];
    px2 = px * px;
    py = x_[1];
    py2 = py * py;
    sps = pow((px2 + py2), 0.5);
    vx = x_[2];
    vy = x_[3];
    h_ << sps, atan(py / px), (px * vx + py * vy) / sps;
    VectorXd y = z - h_;

    //y[1] += 2*PI; // normalizing PHI

    MatrixXd Hj = Jacobian(x_);
    MatrixXd Ht = Hj.transpose();
    MatrixXd S = Hj * P_ * Ht + Rr_;
    MatrixXd K = P_ * Ht * S.inverse();
    x_ = x_ + K * y;
    long x_size = x_.size();
    MatrixXd I_ = MatrixXd::Identity(x_size, x_size);
    P_ = (I_ - K * Hj) * P_;

}