/*
 * File: KalmanFilterAbstract.hpp
 * Project: IMU_Kalman_filter
 * File Created: Saturday, 4th November 2023 7:09:45 pm
 * Author: ATV
 * -----
 * License: MIT License
 * -----
 * Description: Simple Kalman Filter abstract class. Inspired by
 * https://github.com/hmartiro/kalman-cpp.
 */

#pragma once // Prevent multiple inclusions

#include <Eigen/Dense>

using namespace Eigen;

class KalmanFilterAbstract
{
protected:
  // Config
  double t0 = 0.0; // Initial time
  double t = 0.0;  // Current time
  int m, n;        // Number of states and measuremens -> System dimensions
  double dt;       // Time step

  // Matrices
  MatrixXd F;            // State Transition
  MatrixXd B; // Control Input Transtion (optional)
  MatrixXd H;            // Observation
  MatrixXd P_prior;      // Prior Error Covariance
  MatrixXd P_post;       // Estimation Error Covariance
  MatrixXd Q;            // Process Noise Covariance
  MatrixXd R;            // Measurement Noise Covariance

  MatrixXd I; // n-size identity

  // Vectors
  VectorXd x_prior; // Prior state
  VectorXd x_post;  // Estimated state
  VectorXd u;       // Control input

public:
  // Contructors
  KalmanFilterAbstract(){}; // Basic constructor
  // With control input matrix B
  KalmanFilterAbstract(double dt,
                       const MatrixXd& F,
                       const MatrixXd& H,
                       const MatrixXd& Q,
                       const MatrixXd& R,
                       const MatrixXd& P,
                       const MatrixXd& B)
    : dt(dt)
    , F(F)
    , H(H)
    , Q(Q)
    , R(R)
    , P_prior(P)
    , B(B)
    , m(H.rows())
    , n(F.rows())
    , I(n, n)
    , x_prior(n)
    , x_post(n)
    , u(n)
  {
    I.setIdentity(n, n);
  }
  
  // Without control input matrix B
  KalmanFilterAbstract(double dt,
                       const MatrixXd& F,
                       const MatrixXd& H,
                       const MatrixXd& Q,
                       const MatrixXd& R,
                       const MatrixXd& P)
    : dt(dt)
    , F(F)
    , H(H)
    , Q(Q)
    , R(R)
    , P_prior(P)
    , m(H.rows())
    , n(F.rows())
    , I(n, n)
    , x_prior(n)
    , x_post(n)
    , u(n)
    , B(n, n)
  {
    I.setIdentity(n, n);
  }

  // Destructor
  virtual ~KalmanFilterAbstract() {}

  virtual void init() = 0;    // Initialize filter
  virtual void predict() = 0; // Predict filter
  virtual void update() = 0;  // Update filter
  VectorXd get_state() { return x_prior; };  // Get current state
};
