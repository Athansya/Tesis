/*
 * File: main.cpp
 * Project: vehicle_location_example
 * File Created: Thursday, 12th October 2023 5:33:06 pm
 * Author: Alfonso Toriz Vazquez (atoriz98@comunidad.unam.mx)
 * -----
 * Last Modified: Thursday, 12th October 2023 5:33:08 pm
 * Modified By: Alfonso Toriz Vazquez (atoriz98@comunidad.unam.mx>)
 * -----
 * License: MIT License
 * -----
 * Description: Example 9 - vehicle location estimation from
 * Alex Becker's book, Kalman Filter from the ground up.
 */

#include <librealsense2/rs.hpp>
#include <iostream>
#include <Eigen/Dense>  // You'll need to install the Eigen library

#include "kalman.hpp"

const int NUMBER_OF_MEASUREMENTS = 1000;
// TODO CORROBORAR CON LOS EJEMPLOS DEL LIBRO

int main(int argc, char* argv[])
{
    // Kalman filter
    // Number of states: position, velocity and acceleration (x,y)
    int n = 6; 
    // Measurements of position in the 2D plane.
    int m = 2; 

    // Random variance in acceleration \sigma^2_a
    double std_a = 0.2;  // m/s^2
    double var_a = std_a * std_a;

    // Measurement error std for position (x,y)
    double std_pos = 3.0;  // meters
    double var_pos = std_pos * std_pos;

    double dt = 1.0; // seconds
    double dt2 = dt * dt;
    double dt3 = dt2 * dt;
    double dt4 = dt3 * dt;

    Eigen::MatrixXd A(n, n); // Transition matrix
    Eigen::MatrixXd C(m, n); // Output/Observation matrix
    Eigen::MatrixXd Q(n, n); // Process noise covariance
    Eigen::MatrixXd R(m, m); // Measurement noise covariance
    Eigen::MatrixXd P(n, n); // Estimate error covariance

    // Define dynamics for each axis
    A << 1, dt, 0.5*dt2, 0,  0,       0, // x_pos_hat
         0,  1,      dt, 0,  0,       0, // x_vel_hat
         0,  0,       1, 0,  0,       0, // x_acc_dot
         0,  0,       0, 1, dt, 0.5*dt2, // y_pos_hat
         0,  0,       0, 0,  1,      dt, // y_vel_hat
         0,  0,       0, 0,  0,       1; // y_acc_dot

    // Measuring position (x,y)
    C << 1, 0, 0, 0, 0, 0,
         0, 0, 0, 1, 0, 0; 

    // Covariance matrix Q obtained from the transition matrix
    Q << dt4/4, dt3/3, dt2/2,     0,     0,     0,
         dt3/2,   dt2,    dt,     0,     0,     0,
         dt2/2,    dt,     1,     0,     0,     0,
             0,     0,     0, dt4/4, dt3/2, dt2/2,
             0,     0,     0, dt3/2,   dt2,    dt,
             0,     0,     0, dt2/2,    dt,     1;

    Q = Q * var_a;

    // Measurement noise based on accuracy/precision of sensor.
    R << var_pos,       0,
               0, var_pos; 
            
    // Covariance matrix P 
    P << 500,   0,   0,   0,   0,   0,
           0, 500,   0,   0,   0,   0,
           0,   0, 500,   0,   0,   0,
           0,   0,   0, 500,   0,   0, 
           0,   0,   0,   0, 500,   0,
           0,   0,   0,   0,   0, 500;
         

    std::cout << "A: \n" << A << std::endl;
    std::cout << "C: \n" << C << std::endl;
    std::cout << "Q: \n" << Q << std::endl;
    std::cout << "R: \n" << R << std::endl;
    std::cout << "P: \n" << P << std::endl;
    
    // Construct the filter
    KalmanFilter kf(dt,A, C, Q, R, P);
   
    // Position measurements (x,y)
    Eigen::MatrixXd measurements(35, 2);
    measurements << 301.50, -401.46,
                    298.23, -375.44,
                    297.83, -346.15,
                    300.42, -320.20,
                    301.94, -300.08,
                    299.60, -274.12,
                    305.98, -253.45,
                    301.25, -226.40,
                    299.73, -200.65,
                    299.20, -171.62,
                    298.62, -152.11,
                    301.84, -125.19,
                    299.60,  -93.40,
                    295.30,  -74.79,
                    299.30,  -49.12,
                    301.95,  -28.73,
                    296.30,    2.99,
                    295.11,   25.65,
                    295.12,   49.86,
                    289.90,   72.87,
                    283.51,   96.34,
                    276.42,  120.40,
                    264.22,  144.69,
                    250.25,  168.06,
                    236.66,  184.99,
                    217.47,  205.11,
                    199.75,  221.82,
                    179.70,  238.30,
                    160.00,  253.02,
                    140.92,  267.19,
                    113.53,  270.71,
                     93.68,  285.86,
                     69.71,  288.48,
                     45.93,  292.90,
                     20.87,  298.77;

    // Best guess of initial states
    Eigen::VectorXd x0(n);
    double t = 0;
    x0 << 0, 0, 0, 0, 0, 0;
    kf.init(t, x0);

    // Feed measurements into filter, output estimated states

    for(int i = 0; i < measurements.rows(); ++i) // - 1 because of leftover value
    {
         t += dt;
         Eigen::VectorXd y = measurements.row(i);
         kf.update(y);
         std::cout << "t = " << t << ", " << "y[" << i << "] = " << y.transpose() << ", x_hat[" << i << "] = " << kf.state().transpose() << std::endl;
    }
    // Results match with the book!

    return 0;
}