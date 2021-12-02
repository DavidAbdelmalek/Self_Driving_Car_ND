#include "kalman_filter.h"
#include "tools.h"
#include "cmath"
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
  x_ = F_ * x_;
  P_ = F_ * P_ * F_.transpose() + Q_ ;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
   * TODO: update the state by using Kalman Filter equations
   */
  MatrixXd H;

  H << 1,0,0,0,
      0,1,0,0;

  VectorXd y = z - H*x_;
   
  MatrixXd S= H_ * P_ * H_.transpose() * R_;
  MatrixXd KG = P_ * H_.transpose() * S.inverse();

  x_ = x_ + KG *y ;
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - KG * H_) * P_;

}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
   * TODO: update the state by using Extended Kalman Filter equations
   */
  float px = x_[0];
  float py = x_[1];
  float vx = x_[2];
  float vy = x_[3];

  float c1 = sqrt(pow(px,2)+pow(py,2));
  VectorXd hx = VectorXd(3);
  if (px > 0){ 
  hx << c1,
        atan(py/px),
        ((px*vx) + (py+vy))/c1;
  }

  // Calculate Jacobian.
  Tools t = Tools();
  MatrixXd HJ= t.CalculateJacobian(x_);

  VectorXd y = z - hx;
   
  MatrixXd S= HJ * P_ * HJ.transpose() * R_;
  MatrixXd KG = P_ * HJ.transpose() * S.inverse();

  x_ = x_ + KG *y ;
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - KG * HJ) * P_;

}
