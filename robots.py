import numpy as np
from filters.Kalman import KalmanFilter
from move import generate_rectangular_spiral_path
from sensor import gaussian_sensor_model
from shapely.geometry import Point, Polygon

class Robot:
    def __init__(self, vor, vor_center, region, vertices, config):
        """
        Initialize the Robot.

        Parameters:
        vor_center (numpy array): Coordinates of the Voronoi center.
        region (list): List of vertices comprising the finite region.
        vertices (numpy array): Coordinates of the finite vertices.
        """
        self.vor = vor
        self.vor_center = vor_center
        self.region = region
        self.vertices = vertices
        self.sensor_function = None
        self.kalman_filter = None
        self.planned_path = None
        self.sampled_sensor_values = None
        self.config = config

        initial_covariance = np.eye(2)
        self.set_sensor_function(gaussian_sensor_model)
        self.set_initial_state(self.vor_center, initial_covariance)

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

        A = np.array([[1, 0], [0, 1]])
        B =  None
        H = np.array([[1, 0], [0, 1]])
        Q = np.array([[0.1, 0], [0, 0.1]])
        R = np.array([[0.1, 0], [0, 0.1]])

        # Define the state transition function for the EKF and UKF
        def f(state: np.ndarray) -> np.ndarray:
            # logging.debug(f"f() input shape: {state.shape}")
            return A @ state

        # Define the observation function for the EKF and UKF
        def h(state: np.ndarray) -> np.ndarray:
            # logging.debug(f"h() input shape: {state.shape}")
            return H @ state

        # Define the Jacobian of f with respect to state
        def A_jacobian(state: np.ndarray) -> np.ndarray:
            # logging.debug(f"A_jacobian() input shape: {state.shape}")
            return A

        # Define the Jacobian of h with respect to predicted state
        def H_jacobian(state: np.ndarray) -> np.ndarray:
            # logging.debug(f"H_jacobian() input shape: {state.shape}")
            return H


        self.kalman_filter = KalmanFilter(A, B, H, Q, R, initial_state, initial_covariance)

    def move_and_sense(self, vor, grid_points, weed_density):
        """
        Move the Robot and Sample a Sensor Value

        Parameters:
        move_vector (numpy array): Vector representing the movement.
        """

        grid_resolution = self.config.getfloat('INITIAL_SETUP', 'grid_resolution')
        sampling_time = self.config.getint('SIMULATION_SETUP', 'sampling_time')
        time_per_step = self.config.getfloat('SIMULATION_SETUP', 'time_per_step')

        self.planned_path, self.sampled_sensor_values \
            = generate_rectangular_spiral_path(self.vor_center, [vor.vertices[i] for i in self.region], 
                grid_resolution, grid_points, weed_density,
                sampling_time, time_per_step, boundary_tolerance=0.02)

    def update(self):
        """
        Update the estimated sensor function.

        Returns:
        none
        """

        for sensor_value in self.sampled_sensor_values:
            self.kalman_filter.update(sensor_value)
    
    def calculate_new_voronoi_center(self):
        """
        Calculate the new Voronoi center based on the updated sensor function.

        Returns:
        numpy array: New Voronoi center coordinates.
        """
        # Sample points
        cnt = 10000
        xp = min(self.vertices[:, 0]) + np.random.rand(cnt) * (max(self.vertices[:, 0]) - min(self.vertices[:, 0]))
        yp = min(self.vertices[:, 1]) + np.random.rand(cnt) * (max(self.vertices[:, 1]) - min(self.vertices[:, 1]))
        voronoi_region = Polygon(self.vertices)
        in_voronoi = [voronoi_region.contains(Point(x, y)) for x, y in zip(xp, yp)]
        xp = xp[in_voronoi]
        yp = yp[in_voronoi]

        # Integrals over Voronoi region
        kq = np.array(self.sensor_function(np.array([x,y]), self.kalman_filter.state, self.kalman_filter.covariance) for x,y in zip(xp,yp))
        phi_est = kq * self.kalman_filter.state

        mv = np.sum(phi_est)
        cx = np.sum(xp * phi_est) / mv
        cy = np.sum(yp * phi_est) / mv

        return np.array([cx, cy])