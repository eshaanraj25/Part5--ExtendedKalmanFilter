#include "kalman_filter.h"
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;

/* 
 * Please note that the Eigen library does not initialize 
 *   VectorXd or MatrixXd objects with zeros upon creation.
 */

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}

void KalmanFilter::Predict() {
  x_ = F_ * x_;
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  // Calculations
  VectorXd y = z - H_ * x_;
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd K = P_ * Ht * Si;
  MatrixXd I = MatrixXd::Identity(4, 4);

  // Update state
  x_ = x_ + (K * y);
  P_ = (I - K * H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  // Auxillary variables
  double px = x_(0), py = x_(1);
  double vx = x_(2), vy = x_(3);

  // Calculate h(x)
  double rho = sqrt(px * px + py * py);
  double phi = atan2(py, px);
  double rho_dot = (px * vx + py * vy) / rho;
  
  VectorXd h(3);
  h << rho, phi, rho_dot;

  // Calculations
  VectorXd y = z - h;
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd K = P_ * Ht * Si;
  MatrixXd I = MatrixXd::Identity(4, 4);
  
  // Retain values of phi between -pi and pi
  while (fabs(y(1)) > M_PI){
     if (y(1) > M_PI) {
        y(1) = y(1) - M_PI;
     } else {
        y(1) = y(1) + M_PI; 
     }
  }

  // Update state
  x_ = x_ + (K * y);
  P_ = (I - K * H_) * P_;
}
