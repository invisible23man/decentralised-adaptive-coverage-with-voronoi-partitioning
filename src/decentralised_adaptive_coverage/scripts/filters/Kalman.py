import numpy as np

class KalmanFilter:
    def __init__(self, A, B, H, Q, R, initial_state, initial_covariance):
        """
        Initialize the Kalman Filter.

        Parameters:
        A (numpy array): State transition matrix.
        B (numpy array): Control matrix.
        H (numpy array): Observation matrix.
        Q (numpy array): Process noise covariance matrix.
        R (numpy array): Measurement noise covariance matrix.
        initial_state (numpy array): Initial state estimate.
        initial_covariance (numpy array): Initial state covariance matrix.
        """
        self.A = A #Make static , identity, + white noise
        self.B = B
        self.H = H
        self.Q = Q # tuning parameter
        self.R = R # based on real sensor
        self.state = initial_state
        self.covariance = initial_covariance

    def update(self, measurement):
        """
        Update the Kalman Filter with a new measurement.

        Parameters:
        measurement (numpy array): Measurement vector.

        Returns:
        numpy array: Updated state estimate.
        """
        predicted_state = self.A @ self.state
        predicted_covariance = self.A @ self.covariance @ self.A.T + self.Q

        innovation = measurement - self.H @ predicted_state
        innovation_covariance = self.H @ predicted_covariance @ self.H.T + self.R

        kalman_gain = predicted_covariance @ self.H.T @ np.linalg.inv(innovation_covariance)

        self.state = predicted_state + kalman_gain @ innovation
        self.covariance = (np.eye(self.A.shape[0]) - kalman_gain @ self.H) @ predicted_covariance

        return self.state


class ExtendedKalmanFilter(KalmanFilter):
    def __init__(self, f, h, A, B, H, Q, R, initial_state, initial_covariance):
        """
        Initialize the Extended Kalman Filter.

        Parameters:
        f (function): State transition function.
        h (function): Observation function.
        A (function): Jacobian of f with respect to state.
        B (numpy array): Control matrix.
        H (function): Jacobian of h with respect to predicted state.
        Q (numpy array): Process noise covariance matrix.
        R (numpy array): Measurement noise covariance matrix.
        initial_state (numpy array): Initial state estimate.
        initial_covariance (numpy array): Initial state covariance matrix.
        """
        super().__init__(A, B, H, Q, R, initial_state, initial_covariance)
        self.f = f
        self.h = h

    def update(self, measurement):
        """
        Update the Extended Kalman Filter with a new measurement.

        Parameters:
        measurement (numpy array): Measurement vector.

        Returns:
        numpy array: Updated state estimate.
        """
        predicted_state = self.f(self.state)
        predicted_jacobian = self.A(self.state)

        predicted_covariance = predicted_jacobian @ self.covariance @ predicted_jacobian.T + self.Q

        innovation = measurement - self.h(predicted_state)
        innovation_jacobian = self.H(predicted_state)

        innovation_covariance = innovation_jacobian @ predicted_covariance @ innovation_jacobian.T + self.R

        kalman_gain = predicted_covariance @ innovation_jacobian.T @ np.linalg.inv(innovation_covariance)

        self.state = predicted_state + kalman_gain @ innovation
        self.covariance = (np.eye(self.A(self.state).shape[0]) - kalman_gain @ innovation_jacobian) @ predicted_covariance

        return self.state


class UnscentedKalmanFilter(KalmanFilter):
    def __init__(self, f, h, initial_state, initial_covariance, process_noise, measurement_noise):
        """
        Initialize the Unscented Kalman Filter.

        Parameters:
        f (function): State transition function.
        h (function): Observation function.
        initial_state (numpy array): Initial state estimate.
        initial_covariance (numpy array): Initial state covariance matrix.
        process_noise (numpy array): Process noise covariance matrix.
        measurement_noise (numpy array): Measurement noise covariance matrix.
        """
        super().__init__(None, None, None, process_noise, measurement_noise, initial_state, initial_covariance)
        self.f = f
        self.h = h

    def update(self, measurement):
        """
        Update the Unscented Kalman Filter with a new measurement.

        Parameters:
        measurement (numpy array): Measurement vector.

        Returns:
        numpy array: Updated state estimate.
        """
        n = self.state.shape[0]
        k = 2 * n + 1

        alpha = 1e-3
        beta = 2
        kappa = 0

        lambda_ = alpha**2 * (n + kappa) - n

        weights_mean = np.full(k, 1 / (2 * (n + lambda_)))
        weights_mean[0] = lambda_ / (n + lambda_)
        weights_covariance = weights_mean.copy()

        sigma_points = np.zeros((k, n))

        # Generate sigma points
        sigma_points[0] = self.state
        sigma_points[1:n+1] = self.state + np.sqrt(n + lambda_) * np.linalg.cholesky(self.covariance).T
        sigma_points[n+1:] = self.state - np.sqrt(n + lambda_) * np.linalg.cholesky(self.covariance).T

        # Predict sigma points
        predicted_sigma_points = np.array([self.f(sigma_point) for sigma_point in sigma_points])
        # predicted_sigma_points = self.f(sigma_points)

        # Predict mean and covariance
        predicted_state = np.sum(weights_mean[:, None] * predicted_sigma_points, axis=0)
        predicted_covariance = np.sum(
            weights_covariance[:, None, None] * (predicted_sigma_points - predicted_state)[:, :, None] @
            (predicted_sigma_points - predicted_state)[:, None, :],
            axis=0
        ) + self.Q

        # Calculate cross-covariance matrix
        cross_covariance = np.sum(
            weights_covariance[:, None, None] * (sigma_points - self.state)[:, :, None] @
            (predicted_sigma_points - predicted_state)[:, None, :],
            axis=0
        )

        # Calculate Kalman gain
        kalman_gain = cross_covariance @ np.linalg.inv(predicted_covariance + self.R)

        # Update state and covariance
        self.state = predicted_state + kalman_gain @ (measurement - self.h(predicted_sigma_points))
        self.covariance = predicted_covariance - kalman_gain @ predicted_covariance @ kalman_gain.T

        return self.state
