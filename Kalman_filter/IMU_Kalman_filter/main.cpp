#include <librealsense2/rs.hpp>
#include <iostream>
#include <Eigen/Dense>  // You'll need to install the Eigen library

#include "kalman.hpp"

const int NUMBER_OF_MEASUREMENTS = 10000;
const float BAND_STOP_MEASUREMENT_FILTER = 0.5;

int main(int argc, char* argv[])
{
    // Kalman filter
    // Number of states: distance, velocity and acceleration
//     int n = 3; 
    int n = 3; 
    // Number of measurements, just acceleration. The others are hidden.
    int m = 1; 

    // Random variance in acceleration \sigma^2_a
    double std_a = 5;
    double var_a = std_a * std_a;

    // Time step based on D435i Realsense Datasheet
    double dt = 1.0 / 250.0; 
    double dt2 = dt * dt;
    double dt3 = dt2 * dt;
    double dt4 = dt3 * dt;

    Eigen::MatrixXd A(n, n); // Transition matrix
    Eigen::MatrixXd C(m, n); // Output/Observation matrix
    Eigen::MatrixXd Q(n, n); // Process noise covariance
    Eigen::MatrixXd R(m, m); // Measurement noise covariance
    Eigen::MatrixXd P(n, n); // Estimate error covariance

    // Define dynamics (x, dot_x, dot_dot_x) for each axis
    A << 1, dt, dt2/2, // X-axis
         0, 1, dt,
         0, 0, 1;

    C << 0, 0, 1; // Just taking the acceleration. Observation matrix.

    // Covariance matrix Q obtained from the transition matrix
    Q << dt4/4, dt3/3, dt2/2,
         dt3/2, dt2, dt,
         dt2/2, dt, 1;
    // Q << 0, 0, 0,
        //  0, 0, 0,
        //  0, 0, 0;

    Q = Q * var_a;  // Multiplied by random variance in acceleration

    R << 0.001; // Measurement noise based on accuracy/precision of sensor.

    // Covariance matrix P obtained from correlations between position, velocity and acceleration 
    // P << 0.3, 0.5, 0.2,
        //  0.5, 0.3, 0.2,
        //  0.2, 0.2, 0.1;
    // P << 10000, 10000, 1,
        //  10000, 10000, 1000,
        //  1, 1000, 1;
    // P << 100, 5, 2,
        //  5, 1000, 10,
        //  2, 10, 20;
    P << 100, 0, 0,
         0, 100, 0,
         0, 0, 100;

    std::cout << "A: \n" << A << std::endl;
    std::cout << "C: \n" << C << std::endl;
    std::cout << "Q: \n" << Q << std::endl;
    std::cout << "R: \n" << R << std::endl;
    std::cout << "P: \n" << P << std::endl;
    
    // Construct the filter
    KalmanFilter kf(dt,A, C, Q, R, P);
    // Create a RealSense context and configure IMU streaming
    rs2::context ctx;
    rs2::device_list devices = ctx.query_devices();
    if (devices.size() == 0)
    {
        std::cerr << "No RealSense devices found." << std::endl;
        return 1;
    }
    
    rs2::device dev = devices[0];  // Assuming the first device
    rs2::config cfg;
    cfg.enable_stream(RS2_STREAM_ACCEL, RS2_FORMAT_MOTION_XYZ32F, 250);  // Accelerometer
    cfg.enable_stream(RS2_STREAM_GYRO, RS2_FORMAT_MOTION_XYZ32F, 200);  // Gyroscope
    
    // Start the RealSense pipeline
    rs2::pipeline pipe;
    pipe.start(cfg);

    
    // Acceleration measurements (y)
    Eigen::VectorXd measurements;
    measurements.resize(1);
    // Get measurements
    for (int i = 0; i < NUMBER_OF_MEASUREMENTS; i++)
    {
        rs2::frameset frames = pipe.wait_for_frames();
        rs2::motion_frame accel_frame = frames.first_or_default(RS2_STREAM_ACCEL);
     //    rs2::motion_frame gyro_frame = frames.first_or_default(RS2_STREAM_GYRO);

        // if (accel_frame && gyro_frame)
        if (accel_frame)
        {
            rs2_vector accel_data = accel_frame.get_motion_data();
          //   rs2_vector gyro_data = gyro_frame.get_motion_data();
            // std::cout << accel_data.x << std::endl;
            // FILTER
            if (accel_data.x >= -BAND_STOP_MEASUREMENT_FILTER && accel_data.x <= BAND_STOP_MEASUREMENT_FILTER)
            {
                measurements(measurements.size() - 1) = 0;
            }
            else
            {
                measurements(measurements.size() - 1) = accel_data.x;
            }
          //   measurements(measurements.size() - 1) = gyro_data.z;
            measurements.conservativeResize(measurements.size() + 1);
        }
   }
   // Show measurements
   std::cout << measurements.transpose() << std::endl;

   // Best guess of initial states
   Eigen::VectorXd x0(n);
   double t = 0;
   x0 << 0, 0, measurements[0];
   kf.init(t, x0);

   // Feed measurements into filter, output estimated states
   Eigen::VectorXd y(m);
   std::cout << "t = " << t << ", " << "x_hat[0]: " << kf.state().transpose() << std::endl;
   for(int i = 0; i < measurements.size() - 1; i++) // - 1 because of leftover value
   {
        t += dt;
        y << measurements[i];
        kf.update(y);
        std::cout << "t = " << t << ", " << "y[" << i << "] = " << y.transpose() << ", x_hat[" << i << "] = " << kf.state().transpose() << std::endl;
   }

   return 0;
}
