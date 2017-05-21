#include "FusionEKF.h"
#include "Eigen/Dense"
#include <iostream>
#include "tools.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

/*
 * Constructor.
 */
FusionEKF::FusionEKF() {
  is_initialized_ = false;
  // initialize EKF object and arrays
  std::cout << "EKF: initializing ..." << std::endl;
  ekf_.Init();
}

/**
* Destructor.
*/
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {
  /*****************************************************************************
   *  Initialization
   ****************************************************************************/
  if (!is_initialized_) {

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      // Fill in both position and velocity
      ekf_.x_ << Polar2Cartesian(measurement_pack.raw_measurements_);
      // I am confident in both position and velocity
      ekf_.P_ << 1, 0, 0, 0,
                 0, 1, 0, 0,
                 0, 0, 1, 0,
                 0, 0, 0, 1;
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      // Fill in only position
      ekf_.x_(0) = measurement_pack.raw_measurements_(0); 
      ekf_.x_(1) = measurement_pack.raw_measurements_(1);
      // I have no idea about velocity
      ekf_.P_ << 1, 0,    0,    0,
                 0, 1,    0,    0,
                 0, 0, 1000,    0,
                 0, 0, 0   , 1000;
    }

    // done initializing, no need to predict or update
    is_initialized_ = true;
    previous_timestamp_ = measurement_pack.timestamp_;
    return;
  }

  /*****************************************************************************
  *  Prediction
  ****************************************************************************/

  //compute the time elapsed between the current and previous measurements
  float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
  previous_timestamp_ = measurement_pack.timestamp_;

  float dt_2 = dt * dt;
  float dt_3 = dt_2 * dt / 2;
  float dt_4 = dt_2 * dt_2 / 4;

  // Update the state transition matrix F according to the new elapsed time.
  ekf_.F_(0, 2) = dt;
  ekf_.F_(1, 3) = dt;

  float s_x = ekf_.noise_ax;
  float s_y = ekf_.noise_ay;
  // Update the process noise covariance matrix.
  ekf_.Q_ << dt_4 * s_x,          0, dt_3 * s_x,          0,
                      0, dt_4 * s_y,          0, dt_3 * s_y,
             dt_3 * s_x,          0, dt_2 * s_x,          0,
                      0, dt_3 * s_y,          0, dt_2 * s_y;


  ekf_.Predict();

  /*****************************************************************************
   *  Update
   ****************************************************************************/

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    ekf_.UpdateEKF(measurement_pack.raw_measurements_);
  } else {
    ekf_.Update(measurement_pack.raw_measurements_);
  }
}
