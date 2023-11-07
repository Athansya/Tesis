/*
 * File: LinearKalmanFilter.cpp
 * Project: IMU_Kalman_filter
 * File Created: Monday, 6th November 2023 11:10:20 am
 * Author: Alfonso Toriz Vazquez (atoriz98@comunidad.unam.mx)
 * -----
 * Last Modified: Monday, 6th November 2023 11:10:23 am
 * Modified By: Alfonso Toriz Vazquez (atoriz98@comunidad.unam.mx>)
 * -----
 * License: MIT License
 * -----
 * Description:
 */

#include "LinearKalmanFilter.hpp"

LinearKalmanFilter::LinearKalmanFilter(double dt,
                                       const MatrixXd& F,
                                       const MatrixXd& H,
                                       const MatrixXd& Q,
                                       const MatrixXd& R,
                                       const MatrixXd& P,
                                       const MatrixXd& B)
  : KalmanFilterAbstract(dt, F, H, Q, R, P, B)
{
  init();
}

LinearKalmanFilter::LinearKalmanFilter(double dt,
                                       const MatrixXd& F,
                                       const MatrixXd& H,
                                       const MatrixXd& Q,
                                       const MatrixXd& R,
                                       const MatrixXd& P)
  : KalmanFilterAbstract(dt, F, H, Q, R, P)
{
  init();
}
void
LinearKalmanFilter::init()
{
  x_prior.setZero();
  P_post = P_prior;
}
// Override predict method
void
LinearKalmanFilter::predict()
{
  // Implementation of predict method for LinearKalmanFilter
}

// Override update method
void
LinearKalmanFilter::update() // Vector z, Vector u)
{
  // Implementation of update method for LinearKalmanFilter
  // Checar vector u != nullptr{}
}