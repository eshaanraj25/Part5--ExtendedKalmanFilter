#include "tools.h"
#include <iostream>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  // Define the RMSE vector
  VectorXd rmse(4);
  rmse << 0, 0, 0, 0;

  // The estimation vector size should not be zero
  // The estimation vector size should equal ground truth size
  if (estimations.size() == 0 || estimations.size() != ground_truth.size()){
     std::cout << "CalculateRMSE() - Error - Invalid estimation vector" << std::endl;
     return rmse;
  }

  // Generate the squared error
   for (int unsigned i = 0; i < estimations.size(); i += 1){
      VectorXd squared_error = estimations[i] - ground_truth[i];
      squared_error = squared_error.array() * squared_error.array();
      rmse = rmse + squared_error;
   }

   // Calculate the mean
   rmse = rmse / estimations.size();

   // Calculate the squared root
   rmse = rmse.array().sqrt();

   // Output RMSE
   return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
   // Define the Jacobian Matrix
   MatrixXd Hj(3, 4);
   double px = x_state(0);
   double py = x_state(1);
   double vx = x_state(2);
   double vy = x_state(3);

   // Define auxillary variables
   double c1 = px * px + py * py;
   double c2 = sqrt(c1);
   double c3 = c1 * c2;

   // Check division by zero
   if (fabs(c1) < 0.0001) {
      return Hj;
   }

   // Calculate the matrix
   Hj << (px / c2), (py / c2), 0, 0,
         -(py / c1), (px / c1), 0, 0,
         py * (vx * py - vy * px) / c3, px * (px * vy - py * vx) / c3, px / c2, py / c2;

   // Output Jacobian
   return Hj;
}
