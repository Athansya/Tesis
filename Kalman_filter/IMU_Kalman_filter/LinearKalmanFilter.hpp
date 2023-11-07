/*
 * File: LinearKalmanFilter.hpp
 * Project: IMU_Kalman_filter
 * File Created: Monday, 6th November 2023 10:51:34 am
 * Author: Alfonso Toriz Vazquez (atoriz98@comunidad.unam.mx)
 * -----
 * Last Modified: Monday, 6th November 2023 10:51:36 am
 * Modified By: Alfonso Toriz Vazquez (atoriz98@comunidad.unam.mx>)
 * -----
 * License: MIT License
 * -----
 * Description:
 */

#ifndef LINEAR_KALMAN_FILTER_HPP
#define LINEAR_KALMAN_FILTER_HPP

#include "KalmanFilterAbstract.hpp"

class LinearKalmanFilter : public KalmanFilterAbstract
{
public:
  // Constructor with B and u
  LinearKalmanFilter(double dt,
                     const MatrixXd& F,
                     const MatrixXd& H,
                     const MatrixXd& Q,
                     const MatrixXd& R,
                     const MatrixXd& P,
                     const MatrixXd& B);

  // Constructor without B and u
  LinearKalmanFilter(double dt,
                     const MatrixXd& F,
                     const MatrixXd& H,
                     const MatrixXd& Q,
                     const MatrixXd& R,
                     const MatrixXd& P);

  // Override init method
  void init() override;

  // Override predict method
  void predict() override;

  // Override update method
  void update() override;
};

#endif // LINEAR_KALMAN_FILTER_HPP