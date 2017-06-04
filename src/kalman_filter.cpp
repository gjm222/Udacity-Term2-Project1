#include "kalman_filter.h"
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;

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
  TODO:
    * predict the state
  */
  x_ = F_ * x_;
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */
  VectorXd z_pred = H_ * x_;
  VectorXd y = z - z_pred;
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;

  //new estimate
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */

  //H_ should have already been set to the Jacobian before this point

  //Helpers 
  float Px = x_[0];
  float Py = x_[1];
  float Vx = x_[2];
  float Vy = x_[3];
 
  float Px2Py2sqrt = sqrt(pow(Px,2) + pow(Py,2));
  
  VectorXd z_pred(3); 

  //Directly convert from cartesian to polar...

  //rho
  z_pred[0] = Px2Py2sqrt;

  //phi
  //Check for divide by zero
  if (fabs(Px) < 0.001) {
    Px =0.001;	
  }
  z_pred[1] = atan2(Py, Px);
   
  //Phi dot
  float pxvxpyvy = Px * Vx + Py * Vy;
  if( Px2Py2sqrt < 0.001 && fabs(pxvxpyvy) < 0.001 )
    z_pred[2] = 1;
  else if ( Px2Py2sqrt < 0.001 )
    z_pred[2] = prev_velocity;
  else {
	z_pred[2] = pxvxpyvy / Px2Py2sqrt;
	prev_velocity = z_pred[2];
  }

  //std::cout << " " << z_pred[1] << std::endl;


  VectorXd y = z - z_pred;
  //Normalize
  if( y[1] > M_PI )
	  y[1]-= 2*M_PI;

  if( y[1] < -M_PI )
	  y[1]+= 2*M_PI;	  
  

  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;

  //new estimate
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
}
