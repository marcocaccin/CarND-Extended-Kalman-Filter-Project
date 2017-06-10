#include "kalman_filter.h"
#include "tools.h"
#include <iostream>


using Eigen::MatrixXd;
using Eigen::VectorXd;


KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init() {
  F_ = MatrixXd::Identity(4, 4); // Transition matrix
  H_laser_ = MatrixXd(2, 4); // Measurement matrix - laser
  R_laser_ = MatrixXd(2, 2); // Measurement covariance matrix - laser
  R_radar_ = MatrixXd(3, 3); // Measurement covariance matrix - radar
  P_ = MatrixXd(4, 4); // State covariance matrix
  Q_ = MatrixXd(4, 4); // Process covariance matrix
  x_ = VectorXd(4); // State
  noise_ax = 9; // Acceleration noise x
  noise_ay = 9; // Acceleration noise y

  F_ << 1, 0, 1, 0,
        0, 1, 0, 1,
        0, 0, 1, 0,
        0, 0, 0, 1;

  H_laser_ << 1, 0, 0, 0,
              0, 1, 0, 0;

  R_laser_ << 0.0225, 0,
              0, 0.0225;

  R_radar_ << 0.09,      0,    0,
                 0, 0.0009,    0,
                 0,      0, 0.09;
  /*
  Why are the other variables not filled up:
  * F_ is updated at each timestep with the correct dt
  * P_ is initialized differently depending on whether the first measurement 
    is laser (position only) or radar (position + velocity)
  * Q_ is calculated at each timestep
  */
}

void KalmanFilter::Predict() {
  // Update state with state transition matrix F computed for a given dt
  x_ = F_ * x_;
  // Update state covariance matrix
  P_ = F_ * P_ * F_.transpose() + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  // Measurement residual y
  VectorXd y = z - H_laser_ * x_;

  UpdateCommon(y, H_laser_, R_laser_);
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  // Measurement residual y
  VectorXd y = z - Cartesian2Polar(x_);

  // Normalise angle
  while (y(1) >   M_PI) y(1) -= 2. * M_PI;
  while (y(1) <= -M_PI) y(1) += 2. * M_PI;

  // Measurement matrix
  MatrixXd Hj = CalculateJacobian(x_);

  UpdateCommon(y, Hj, R_radar_);
}


void KalmanFilter::UpdateCommon(const VectorXd &y, 
                                const MatrixXd &H, 
                                const MatrixXd &R) {
  // Get to the Kalman matrix
  MatrixXd Ht = H.transpose();
  MatrixXd PHt = P_ * Ht;
  MatrixXd S = H * PHt + R;
  MatrixXd Si = S.inverse();
  MatrixXd K = PHt * Si;
  // New estimate
  x_ = x_ + (K * y);
  int x_size = x_.size();
  P_ = (MatrixXd::Identity(x_size, x_size) - K * H) * P_;
}
