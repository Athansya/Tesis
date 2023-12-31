#include <iostream>
#include <stdexcept>

#include "kalman.hpp"

KalmanFilter::KalmanFilter(
    double dt,
    const Eigen::MatrixXd& A,
    const Eigen::MatrixXd& C,
    const Eigen::MatrixXd& Q,
    const Eigen::MatrixXd& R,
    const Eigen::MatrixXd& P)
  : A(A), C(C), Q(Q), R(R), P0(P),
    m(C.rows()), n(A.rows()), dt(dt), initialized(false),
    I(n, n), x_hat(n), x_hat_new(n)
{
  I.setIdentity();
}

KalmanFilter::KalmanFilter() {}

void KalmanFilter::init(double t0, const Eigen::VectorXd& x0) {
  x_hat = x0;
  P = P0;
  this->t0 = t0;
  t = t0;
  initialized = true;
}

void KalmanFilter::init() {
  x_hat.setZero();
  P = P0;
  t0 = 0;
  t = t0;
  initialized = true;
}

void KalmanFilter::update(const Eigen::VectorXd& y) {

  if(!initialized)
    throw std::runtime_error("Filter is not initialized!");

  /**
  * Extrapolation Equations
  */ 
  // Predicted state
  x_hat_new = A * x_hat;
  // Predicted error covariance
  P = A*P*A.transpose() + Q;
  // std::cout << "1st P: \n" << P << std::endl;
  // Kalman Gain
  K = P*C.transpose()*(C*P*C.transpose() + R).inverse();
  /**
  * Update equations
  */
  // Estimated state
  x_hat_new += K * (y - C*x_hat_new); 
  // Estimated error covariance
  P = (I - K*C)*P*(I - K*C).transpose() + K*R*K.transpose();
  // std::cout << "2nd P: \n" << P << std::endl;
  
  x_hat = x_hat_new;

  t += dt;
}

void KalmanFilter::update(const Eigen::VectorXd& y, double dt, const Eigen::MatrixXd A) {

  this->A = A;
  this->dt = dt;
  update(y);
}
