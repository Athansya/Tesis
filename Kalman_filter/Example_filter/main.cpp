#include <iostream>
#include <vector>
#include <cmath>
#include <Eigen/Dense>  // You'll need to install Eigen library

using namespace std;
using namespace Eigen;

void print_vector(vector<double> v);

int main() {
    // Define the state variables: position and velocity
    Vector2d x(0, 0);  // Initial estimate: position = 0, velocity = 0

    // Define the initial state covariance matrix (P)
    Matrix2d P;
    P << 1, 0,
         0, 1;

    // Define the time step between sensor readings
    double dt = 1.0;  // Time step

    // Define the constant velocity (10 m/s)
    double constant_velocity = 10.0;

    // Define the state transition matrix (A) for constant velocity
    Matrix2d A;
    A << 1, dt,
         0, 1;

    // Define the measurement matrix (H) for position and velocity
    Matrix<double, 2, 2> H;
    H << 1, 0,
         0, 1;

    // Adjusted measurement noise covariance matrix (R)
    Matrix2d R;
    R << 0.5, 0,
         0, 0.5;

    // Adjusted process noise covariance matrix (Q) for position and velocity
    Matrix2d Q;
    Q << 0.01, 0.02,
         0.02, 0.1;

    // Simulated sensor readings and true values with a constant velocity of 10 m/s
    vector<double> sensor_readings = {10.1, 20.0, 30.1, 30.8, 50.2, 60.1, 70.2, 70.9, 90.0, 100.1};
    vector<double> true_positions;
    vector<double> true_velocities;

    for (int i = 1; i <= sensor_readings.size(); ++i) {
        true_positions.push_back(constant_velocity * i * dt);
        true_velocities.push_back(constant_velocity);
    }

    // Lists to store estimated and predicted values
    vector<double> estimated_positions;
    vector<double> estimated_velocities;
    vector<double> predicted_positions;
    vector<double> predicted_velocities;

    // Kalman Filter loop
    for (int i = 0; i < sensor_readings.size(); ++i) {
        // Prediction Step
        x = A * x;  // Predicted state estimate
        P = A * P * A.transpose() + Q;  // Predicted error covariance

        // Update Step
        Vector2d y;
        y << sensor_readings[i] - x[0], constant_velocity - x[1];  // Measurement residual for position and velocity
        Matrix2d S = H * P * H.transpose() + R;  // Residual covariance
        Matrix2d K = P * H.transpose() * S.inverse();  // Kalman gain

        x = x + K * y;  // Updated state estimate
        P = P - K * H * P;  // Updated error covariance

        // Store estimated position and velocity
        estimated_positions.push_back(x[0]);
        estimated_velocities.push_back(x[1]);

        // Predict next position and velocity based on constant velocity model
        predicted_positions.push_back(x[0] + constant_velocity * dt);
        predicted_velocities.push_back(x[1]);
    }

    // Print or use estimated_positions and estimated_velocities as needed
   cout << "True positions:" << endl;
   print_vector(true_positions);
   cout << "Sensor readings:" << endl;
   print_vector(sensor_readings);
   cout << "Estimated positions:" << endl;
   print_vector(estimated_positions);
   cout << "Predicted positions:" << endl;
   print_vector(predicted_positions);

   cout << "True velocities:" << endl;
   print_vector(true_velocities);
   cout << "Estimated velocities:" << endl;
   print_vector(estimated_velocities);
   cout << "Predicted velocities:" << endl;
   print_vector(predicted_velocities);

    return 0;
}

void print_vector(vector<double> v)
{
    for (double i: v)
        cout << i << ' ';
    cout << endl;
}
