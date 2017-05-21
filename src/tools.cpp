#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;
// using std::cout;
// using std::endl;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  VectorXd rmse(4);
	rmse << 0, 0, 0, 0;

	// check the validity of the following inputs:
	//  * the estimation vector size should not be zero
	//  * the estimation vector size should equal ground truth vector size
	if(estimations.size() != ground_truth.size()
			|| estimations.size() == 0){
		std::cout << "Invalid estimation or ground_truth data" << std::endl;
		return rmse;
	}

	//accumulate squared residuals
	for(unsigned int i=0; i < estimations.size(); ++i){

		VectorXd residual = estimations[i] - ground_truth[i];

		//coefficient-wise multiplication
		residual = residual.array()*residual.array();
		rmse += residual;
	}
	//calculate the mean
	rmse = rmse / estimations.size();
	//calculate the squared root
	rmse = rmse.array().sqrt();
	//return the result
	return rmse;
}

MatrixXd CalculateJacobian(const VectorXd& x_state) {
  MatrixXd Hj(3, 4);
  //recover state parameters
  const float px = x_state(0);
	const float py = x_state(1);
	const float vx = x_state(2);
	const float vy = x_state(3);

  // Precompute repeated entries in Jacobian
  float c1 = px*px + py*py;
  //check division by zero
  if (c1 < 0.0001) {
    std::cout << "CalculateJacobian () - Warning - Division by Zero" << std::endl;
    c1 = 0.0001;
  }
  float c2 = sqrt(c1);
  float c3 = c1*c2;
  float c4 = (vx*py - vy*px) / c3;

	//compute the Jacobian matrix
	Hj << px/c2, py/c2,     0,     0,
       -py/c1, px/c1,     0,     0,
        py*c4, px*c4, px/c2, py/c2;

	return Hj;
}

VectorXd Polar2Cartesian(const VectorXd& x_polar) {
 // Initialize state in Cartesian coordinates, 
 VectorXd x_cartesian(4);
 // Recover state parameters
 float rho = x_polar(0);
 float phi = x_polar(1);
 float rho_dot = x_polar(2);
 // Get Cartesian position
 float px = rho * cos(phi);
 float py = rho * sin(phi);
 // Get Cartesian speed
 float vx = rho_dot * cos(phi);
 float vy = rho_dot * sin(phi);
 // fill it up and return it
 x_cartesian << px, py, vx, vy;
 return x_cartesian;
}


VectorXd Cartesian2Polar(const VectorXd& x_cartesian) {
 // Initialize state in polar coordinates
 VectorXd x_polar(3);
 // Get components of state
 float px = x_cartesian(0);
 float py = x_cartesian(1);
 float vx = x_cartesian(2);
 float vy = x_cartesian(3);
 // Compute radial distance
 float rho = sqrt(px*px + py*py);
 // Compute bearing angle in radians in range[-pi, pi]
 float phi = atan2(py, px);
 // Compute radial speed. Catch division by zero!
 float rho_dot;
 if (rho < 0.0001){
    std::cout << "Cartesian2Polar () - Warning - Division by Zero" << std::endl;
   rho_dot = 0;
 } else {
   rho_dot = (px*vx + py*vy) / rho;
 }
 // Assign component values to out array and return
 x_polar << rho, phi, rho_dot;
 return x_polar;
}
