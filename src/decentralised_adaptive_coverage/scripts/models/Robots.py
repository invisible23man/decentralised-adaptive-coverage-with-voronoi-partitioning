import numpy as np
from filterpy.kalman import UnscentedKalmanFilter, MerweScaledSigmaPoints, KalmanFilter
from filterpy.common import Q_discrete_white_noise

class Robot:
    """
    A base class for a robot that uses a Kalman filter to estimate the state of a sensor function.
    """
    def __init__(self, config):
        """
        Initialize a Robot with a given configuration.

        Parameters:
        config (configparser.ConfigParser): Configuration for the robot.
        """

        self.config = config
        self.sensor_function = None
        self.kalman_filter = None
        self.planned_path = [] 
        self.sampled_sensor_values = []
        self.dt = None

        self.other_states = []
        self.other_covariances = []

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

        dt = self.config.getfloat('FILTER_SETUP','sampling_time')
        self.dt = dt
        A = np.array([[1, dt], [0, 1]])
        B =  None
        H = np.array([[1, 0], [0, 1]])
        Q = np.array([[0.1, 0], [0, 0.1]])
        R = np.array([[0.1, 0], [0, 0.1]])

        # noise = np.random.normal(loc=0, scale=0.1, size=A.shape)  # adjust loc and scale as needed
        # A += noise

        # Define the state transition function for the EKF and UKF
        def f(state: np.ndarray, dt) -> np.ndarray:
            A = np.array([[1, dt], [0, 1]])
            return A @ state

        # Define the observation function for the EKF and UKF
        def h(state: np.ndarray) -> np.ndarray:
            return state

        if self.config.get('FILTER_SETUP','filter_type')=='UKF':
            sigma_points = MerweScaledSigmaPoints(n=A.shape[0], alpha=1e-3, beta=2, kappa=0)  # Sigma Points
            self.kalman_filter = UnscentedKalmanFilter(dim_x=A.shape[0], dim_z=H.shape[0], dt=dt, points=sigma_points, fx=f, hx=h)
        else:
            self.kalman_filter = KalmanFilter(dim_x=A.shape[0], dim_z=H.shape[0])

        self.kalman_filter.x = initial_state
        self.kalman_filter.P = initial_covariance
        self.kalman_filter.Q = Q_discrete_white_noise(dim=A.shape[0], dt=dt, var=0.1)   # Process noise
        self.kalman_filter.R = np.array([[0.1, 0], [0, 0.1]])                           # Measurement noise
        
    def update(self):
        """
        Update the estimated sensor function.

        Returns:
        none
        """

        for sensor_value in self.sampled_sensor_values:
            if self.config.get('FILTER_SETUP','filter_type')=='UKF':
                self.kalman_filter.predict(dt=self.dt)
            
            self.kalman_filter.update(sensor_value)

    def move(self):
        """
        Move the robot. This method should be implemented by subclasses.

        Returns:
        none
        """
        raise NotImplementedError("This method should be implemented by subclasses.")
