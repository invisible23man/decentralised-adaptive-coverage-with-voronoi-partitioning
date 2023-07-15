import numpy as np
from filters.Kalman import KalmanFilter

class Robot:
    def __init__(self, vor_center, finite_region, finite_vertices):
        """
        Initialize the Robot.

        Parameters:
        vor_center (numpy array): Coordinates of the Voronoi center.
        finite_region (list): List of vertices comprising the finite region.
        finite_vertices (numpy array): Coordinates of the finite vertices.
        """
        self.vor_center = vor_center
        self.finite_region = finite_region
        self.finite_vertices = finite_vertices
        self.sensor_function = None
        self.kalman_filter = None

    def set_sensor_function(self, sensor_function):
        """
        Set the sensor function for the Robot.

        Parameters:
        sensor_function (callable): Sensor function.
        """
        self.sensor_function = sensor_function

    def set_initial_state(self, initial_state, initial_covariance):
        """
        Set the initial state and covariance for the Kalman filter.

        Parameters:
        initial_state (numpy array): Initial state estimate.
        initial_covariance (numpy array): Initial state covariance matrix.
        """
        self.kalman_filter = KalmanFilter(A, B, H, Q, R, initial_state, initial_covariance)

    def move(self, move_vector):
        """
        Move the Robot.

        Parameters:
        move_vector (numpy array): Vector representing the movement.
        """
        self.vor_center += move_vector

    def sense(self):
        """
        Perform sensing and update the estimated sensor function.

        Returns:
        float: Sensor value at the Robot's location.
        """
        sensor_value = self.sensor_function(self.vor_center)
        self.kalman_filter.update(sensor_value)
        return sensor_value

    def calculate_new_voronoi_center(self):
        """
        Calculate the new Voronoi center based on the updated sensor function.

        Returns:
        numpy array: New Voronoi center coordinates.
        """
        # Sample points
        cnt = 10000
        xp = min(self.finite_vertices[:, 0]) + np.random.rand(cnt) * (max(self.finite_vertices[:, 0]) - min(self.finite_vertices[:, 0]))
        yp = min(self.finite_vertices[:, 1]) + np.random.rand(cnt) * (max(self.finite_vertices[:, 1]) - min(self.finite_vertices[:, 1]))
        in_voronoi = inpolygon(xp, yp, self.finite_vertices[:, 0], self.finite_vertices[:, 1])
        xp = xp[in_voronoi]
        yp = yp[in_voronoi]

        # Integrals over Voronoi region
        kq = kappa(xp, yp)
        if self.sensor_info_flag:
            phi_est = kq * self.sensor_function(xp, yp)
        else:
            phi_est = kq * self.kalman_filter.state

        mv = np.sum(phi_est)
        cx = np.sum(xp * phi_est) / mv
        cy = np.sum(yp * phi_est) / mv

        return np.array([cx, cy])
