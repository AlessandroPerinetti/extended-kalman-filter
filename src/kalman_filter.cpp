#include "kalman_filter.h"

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
   /**
   * TODO: predict the state
   */
  x_ = F_ * x_ ;
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
   * TODO: update the state by using Kalman Filter equations in case of laser measurements
   */
  VectorXd z_pred = H_ * x_;
  VectorXd y = z - z_pred;
  UpdateInCommon(y);										// Continue the rest of the Update phase with a common function for both 
  															// laser and radar measurements since it is the same process
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
   * TODO: update the state by using Extended Kalman Filter equations in case of radar measurements
   */
  
  // Define the non-linear relation between the radar measurement and the state variables (x_(0) = px, x_(1) = py, x_(2) = vx, x_(3) = vy)
  double rho     = sqrt(x_(0)*x_(0) + x_(1)*x_(1));
  double theta   = atan2(x_(1), x_(0));  
  double rho_dot;

  
  //The rho_dot formula contains the variable rho at the denominator. For this reason, before evaluating rho_dot, we have to check that rho is non zero
  if (fabs(rho) < 0.0001) {
    rho_dot = 0;
  } else {
    rho_dot = (x_(0)*x_(2) + x_(1)*x_(3)) / rho;
  }
  
  //Build the non linear measurement function h(x')
  VectorXd h = VectorXd(3);
  h << rho, theta, rho_dot;
  
  VectorXd y = z - h;

  // Normalize y(1) in the range [-pi, pi]
  while (y(1) > M_PI)  y(1) -= 2.0 * M_PI;
  while (y(1) < -M_PI) y(1) += 2.0 * M_PI;
  
  UpdateInCommon(y);
}

void KalmanFilter::UpdateInCommon(const VectorXd &y){
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd K =  P_ * Ht * Si;
  // New state estimate
  x_ = x_ + (K * y);
  int x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
}
