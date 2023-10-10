import numpy as np
import matplotlib.pyplot as plt

# Define the state variables: position and velocity
x = np.array([0, 0])  # Initial estimate: position = 0, velocity = 0

# Define the initial state covariance matrix (P)
P = np.array([[1, 0],
              [0, 1]])

# Define the time step between sensor readings
dt = 1.0  # Time step

# Define the constant velocity (10 m/s)
constant_velocity = 10.0

# Define the state transition matrix (A) for constant velocity
A = np.array([[1, dt],
              [0, 1]])

# Define the measurement matrix (H) for position and velocity
H = np.array([[1, 0],
              [0, 1]])

# Adjusted measurement noise covariance matrix (R)
R = np.array([[0.5, 0],
              [0, 0.5]])

# Adjusted process noise covariance matrix (Q) for position and velocity
Q = np.array([[0.01, 0.02],
              [0.02, 0.1]])

# Simulated sensor readings and true values with a constant velocity of 10 m/s
true_positions = [constant_velocity * i * dt for i in range(1, 11)]
true_velocities = [constant_velocity] * 10

# Simulate noisy measurements by adding random noise
sensor_readings = [position + np.random.normal(0, 0.5) for position in true_positions]

# Lists to store estimated and predicted values
estimated_positions = []
estimated_velocities = []
predicted_positions = []
predicted_velocities = []

for measurement in sensor_readings:
    # Prediction Step
    x = np.dot(A, x)  # Predicted state estimate
    P = np.dot(np.dot(A, P), A.T) + Q  # Predicted error covariance

    # Update Step
    y = np.array([measurement - x[0], constant_velocity - x[1]])  # Measurement residual for position and velocity
    S = np.dot(np.dot(H, P), H.T) + R  # Residual covariance
    K = np.dot(np.dot(P, H.T), np.linalg.inv(S))  # Kalman gain
    x = x + np.dot(K, y)  # Updated state estimate
    P = P - np.dot(np.dot(K, H), P)  # Updated error covariance

    # Store estimated position and velocity
    estimated_positions.append(x[0])
    estimated_velocities.append(x[1])

    # Predict next position and velocity based on constant velocity model
    predicted_positions.append(x[0] + constant_velocity * dt)
    predicted_velocities.append(x[1])



print(f"True positions: \n{true_positions}")
print(f"Measured positions: \n{sensor_readings}")
print(f"Estimated positions: \n{estimated_positions}")
print(f"Predicted positions: \n{predicted_positions}")

print(f"True velocities: \n{true_velocities}")
print(f"Estimated velocities: \n{estimated_velocities}")
print(f"Predicted velocities: \n{predicted_velocities}")
