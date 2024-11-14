import numpy as np

class EKF:
    def __init__(self, dt, process_noise_cov, measurement_noise_cov, initial_state, initial_covariance):
        self.dt = dt  # Time step

        # State vector [x, y, theta, v]
        self.state = np.array(initial_state)

        # Covariance matrix
        self.P = np.array(initial_covariance)

        # Process noise covariance
        self.Q = np.array(process_noise_cov)

        # Measurement noise covariance
        self.R = np.array(measurement_noise_cov)

        # Identity matrix
        self.I = np.eye(len(self.state))

    def predict(self, acceleration, yaw_rate):
        """Predict step of EKF."""
        x, y, theta, v = self.state

        # State prediction
        x_pred = x + v * np.cos(theta) * self.dt
        y_pred = y + v * np.sin(theta) * self.dt
        theta_pred = theta + yaw_rate * self.dt
        v_pred = v + acceleration * self.dt

        self.state = np.array([x_pred, y_pred, theta_pred, v_pred])

        # Jacobian of the state transition function
        F = np.array([
            [1, 0, -v * np.sin(theta) * self.dt, np.cos(theta) * self.dt],
            [0, 1, v * np.cos(theta) * self.dt, np.sin(theta) * self.dt],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ])

        # Covariance prediction
        self.P = F @ self.P @ F.T + self.Q

    def update(self, measurement):
        """Update step of EKF."""
        # Measurement model
        H = np.array([
            [0, 0, 0, 1],
            [0, 0, 1, 0]
        ])

        # Innovation
        z = np.array(measurement)  # Measurement [a, omega]
        z_pred = H @ self.state  # Predicted measurement
        y = z - z_pred  # Residual

        # Innovation covariance
        S = H @ self.P @ H.T + self.R

        # Kalman gain
        K = self.P @ H.T @ np.linalg.inv(S)

        # State update
        self.state = self.state + K @ y

        # Covariance update
        self.P = (self.I - K @ H) @ self.P

    def get_state(self):
        """Get the current estimated state."""
        return self.state


# Example usage
if __name__ == "__main__":
    # Time step
    dt = 0.1

    # Initial state [x, y, theta, v]
    initial_state = [0, 0, 0, 0]

    # Initial covariance
    initial_covariance = np.eye(4) * 0.1

    # Process noise covariance
    process_noise_cov = np.diag([0.1, 0.1, 0.05, 0.1])

    # Measurement noise covariance
    measurement_noise_cov = np.diag([0.1, 0.05])

    # Create EKF instance
    ekf = EKF(dt, process_noise_cov, measurement_noise_cov, initial_state, initial_covariance)

    # Simulated IMU data (acceleration and yaw rate)
    imu_data = [
        [0.5, 0.1],
        [0.6, 0.1],
        [0.7, 0.2],
        [0.8, 0.2],
        [0.9, 0.3]
    ]

    # Run EKF
    for i, measurement in enumerate(imu_data):
        acceleration, yaw_rate = measurement
        ekf.predict(acceleration, yaw_rate)
        ekf.update(measurement)
        print(f"Step {i+1}, Estimated State: {ekf.get_state()}")
