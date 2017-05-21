#ifndef KALMAN_FILTER_H_
#define KALMAN_FILTER_H_
#include "Eigen/Dense"
#include "tools.h"

class KalmanFilter {
public:

  // state vector
  Eigen::VectorXd x_;
  // state covariance matrix
  Eigen::MatrixXd P_;
  // state transistion matrix
  Eigen::MatrixXd F_;
  // measurement matrix: laser
  Eigen::MatrixXd H_laser_;
  // measurement covariance matrix: radar
  Eigen::MatrixXd R_radar_;
  // measurement covariance matrix: laser
  Eigen::MatrixXd R_laser_;
  // process covariance matrix
  Eigen::MatrixXd Q_;
  // Acceleration noise componentss
  float noise_ax;
  float noise_ay;


  /**
   * Constructor
   */
  KalmanFilter();

  /**
   * Destructor
   */
  virtual ~KalmanFilter();

  /**
   * Init Initializes Kalman filter arrays
   */
  void Init();

  /**
   * Prediction Predicts the state and the state covariance
   * using the process model
   * @param delta_T Time between k and k+1 in s
   */
  void Predict();

  /**
   * Updates the state by using standard Kalman Filter equations
   * @param z The measurement at k+1
   */
  void Update(const Eigen::VectorXd &z);

  /**
   * Updates the state by using Extended Kalman Filter equations
   * @param z The measurement at k+1
   */
  void UpdateEKF(const Eigen::VectorXd &z);

private:

  /**
   * Updates the state by using Extended Kalman Filter equations.
   * This bit is the common part between KF and EKF.
   * @param y = z - H * x measurement residual
   * @param H measurement matrix
   * @param R measurement covariance matrix
   */
  void UpdateCommon(const Eigen::VectorXd &y, 
                    const Eigen::MatrixXd &H, 
                    const Eigen::MatrixXd &R);
};

#endif /* KALMAN_FILTER_H_ */
