//
// Created by Oleg Leyzerov on 29/03/2017.
//

#include "tracking.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::VectorXd;
using Eigen::MatrixXd;

Tracking::Tracking(){
    is_initialized_ = false;
    previous_timestamp_ = 0;
    kf_.epsilon = 0.001;
    kf_.x_ = VectorXd(4); // 4D state vector
    kf_.P_ = MatrixXd(4, 4); // state covariance matrix
    kf_.P_ << 1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1000, 0,
            0, 0, 0, 1000;

    kf_.F_ = MatrixXd(4, 4);
    kf_.F_ << 1, 0, 1, 0,
            0, 1, 0, 1,
            0, 0, 1, 0,
            0, 0, 0, 1;

    kf_.H_ = MatrixXd(2, 4);
    kf_.H_ << 1, 0, 0, 0,
            0, 1, 0, 0;

    kf_.R_ = MatrixXd(2, 2);
    kf_.R_ << 0.0225, 0,
            0, 0.0225;

    kf_.Rr_ = MatrixXd(3,3);
    kf_.Rr_ << 0.9, 0, 0,
            0, 0.0009, 0,
            0, 0, 0.09;

    noise_ax = 6; // acceleration x noise component
    noise_ay = 6; // acceleration y noise component

    RMSE = VectorXd(4); // root mean squared error
    RMSE << 0, 0, 0, 0;

}
Tracking::~Tracking(){
}
void Tracking::ProcessMeasurement(const MeasurementPackage &measurement_pack, const VectorXd &ground_truth) {
    if (!is_initialized_) {
        if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
            kf_.x_ << measurement_pack.raw_measurements_[0], measurement_pack.raw_measurements_[1], 0, 0;
        } else if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
            // converting rho, phi, rho dot into position and velocity
            float px = measurement_pack.raw_measurements_[0] * cos(measurement_pack.raw_measurements_[1]);
            float py = measurement_pack.raw_measurements_[0] * sin(measurement_pack.raw_measurements_[1]);
            float vx = measurement_pack.raw_measurements_[2] * cos(measurement_pack.raw_measurements_[1]);
            float vy = measurement_pack.raw_measurements_[2] * sin(measurement_pack.raw_measurements_[1]);
            if (px == 0 && py == 0) {
                px = kf_.epsilon;
                py = kf_.epsilon;
            }
            kf_.x_ << px, py, vx, vy;
        }
        previous_timestamp_ = measurement_pack.timestamp_;
        is_initialized_ = true;
        return;
    }
    float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
    float dt_2 = pow(dt, 2);
    float dt_3 = dt_2 * dt;
    float dt_4 = dt_3 * dt;

    previous_timestamp_ = measurement_pack.timestamp_;
    kf_.F_(0, 2) = dt;
    kf_.F_(1, 3) = dt;

    kf_.Q_ = MatrixXd(4, 4);
    kf_.Q_ << dt_4 / 4.0 * noise_ax, 0, dt_3/2.0*noise_ax, 0,
            0, dt_4/4.0*noise_ay, 0, dt_3/2.0*noise_ay,
            dt_3/2.0*noise_ax, 0 , dt_2*noise_ax, 0,
            0, dt_3/2.0*noise_ay, 0, dt_2*noise_ay;
    if (dt>kf_.epsilon) {
        kf_.Predict();
    }
    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
        kf_.UpdateRadar(measurement_pack.raw_measurements_);
    } else {
        kf_.Update(measurement_pack.raw_measurements_);
    }
    VectorXd residual;
    residual = kf_.x_ - ground_truth;
    residual = residual.array() * residual.array();
    RMSE += residual;

    //std::cout << "x_= " << kf_.x_ << std::endl;
    //std::cout << "P_= " << kf_.P_ << std::endl;

}
VectorXd Tracking::getRMSE() {
    return RMSE;
}


