import ast
import configparser
import numpy as np
import rospy
from iq_gnc.py_gnc_functions_swarm import gnc_api
from iq_gnc.PrintColours import *
from std_msgs.msg import String
from geometry_msgs.msg import Point as rosMsgPoint
from utils import callbacks, voronoi
from filters.Kalman import KalmanFilter, UnscentedKalmanFilter
from filterpy.kalman import UnscentedKalmanFilter, MerweScaledSigmaPoints
from filterpy.common import Q_discrete_white_noise
from shapely.geometry import Point, Polygon
from move import generate_trajectory
from sensor import gaussian_sensor_model
from gazebo_msgs.msg import ModelStates
import tf2_ros

class Robot:
    def __init__(self, config):
        self.config = config
        self.sensor_function = None
        self.kalman_filter = None
        self.planned_path = [] 
        self.sampled_sensor_values = []
        self.dt = None


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

        def f_pyfilter(state, dt):
            A = np.array([[1, dt], [0, 1]])
            return A @ state

        def h_pyfilter(state):
            return state


        if self.config.get('FILTER_SETUP','filter_type')=='UKF':
            # self.kalman_filter = UnscentedKalmanFilter(f, h, initial_state, initial_covariance, Q, R)
            
            # Initialize the sigma points
            points = MerweScaledSigmaPoints(n=A.shape[0], alpha=1e-3, beta=2, kappa=0)

            # Initialize the UKF
            ukf = UnscentedKalmanFilter(dim_x=A.shape[0], dim_z=H.shape[0], dt=dt, points=points, fx=f_pyfilter, hx=h_pyfilter)
            ukf.x = initial_state
            ukf.P = initial_covariance

            # Process noise
            ukf.Q = Q_discrete_white_noise(dim=A.shape[0], dt=dt, var=0.1)

            # Measurement noise
            ukf.R = np.array([[0.1, 0], [0, 0.1]])
           
            self.kalman_filter = ukf

        else:
            self.kalman_filter = KalmanFilter(A, B, H, Q, R, initial_state, initial_covariance)
        
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
        raise NotImplementedError("This method should be implemented by subclasses.")

class Drone(Robot):
    def __init__(self, config:configparser.ConfigParser()):
        super().__init__(config)
        self.grid_resolution = config.getfloat('INITIAL_SETUP', 'grid_resolution')
        self.sampling_time = config.getint('SIMULATION_SETUP', 'sampling_time')
        self.time_per_step = config.getfloat('SIMULATION_SETUP', 'time_per_step')
        self.boundary_tolerance = 0.02
        self.enable_physics_simulation = self.config.getboolean('GAZEBO_SETUP','enable_physics_simulation')


        self.ns = rospy.get_namespace()
        self.drone_id = int(self.ns.strip('/').replace('drone', ''))-1
        # Set up ROS publishers, subscribers and callbacks
        self.setup_publishers()
        self.callback_handler = callbacks.CallbackHandler(self)
        self.setup_subscribers()
        self.deregister_inital_topics = False

        # Drone Variables for Voronoi Partition
        self.voronoi_center = None
        self.voronoi_region = None
        self.voronoi_center_tracker = []
        self.all_voronoi_center_tracker = []
        # Initialize dictionaries to store parameters of the other drones
        self.other_centers = {}
        self.other_states = {}
        self.other_covariances = {}

        self.all_vertices = String()
        self.voronoi_centers = String()
        self.finite_regions = String()
        self.finite_vertices = String()
        self.boundary_points = String()
        self.xx = String()
        self.yy = String()
        self.grid_points = String()
        self.weed_density = String()

        # Kalman Settings
        self.set_sensor_function(gaussian_sensor_model)
        self.initial_covariance = np.eye(2)
        self.move_and_sense_started = False
        self.apply_consensus = False

        # GNC API
        if self.enable_physics_simulation:
            self.drone = gnc_api()
            self.drone.drone_id = self.drone_id
            # Wait for FCU connection.
            self.drone.wait4connect()
            # Wait for the mode to be switched.
            # drone.wait4start()
            self.drone.set_mode("GUIDED")

            # Subscriber for coordinate transformations
            self.model_states_sub = rospy.Subscriber(
                '/gazebo/model_states', 
                ModelStates, 
                self.callback_handler.mavros_set_home_callback, 
                self.drone_id
            )

            # Wait for the transformation matrix to become available
            while self.drone.transformation_matrix is None:
                rospy.loginfo(f"Waiting for transformation matrix for {self.drone_id}")
                rospy.sleep(0.1)

            # Unsubscribe after receiving the data once
            self.model_states_sub.unregister()
            
            # Create a tf2_ros.TransformBroadcaster
            # self.tf_broadcaster = tf2_ros.TransformBroadcaster()
            rospy.loginfo(f"Transform Broadcasted for {self.drone_id}")


            # Create local reference frame.
            self.drone.initialize_local_frame()
            # Request takeoff with an altitude of 3m.
            self.drone.takeoff(3)

            # Wait for the drone to reach the desired takeoff height.
            while True:
                self.drone_position = self.drone.get_current_location()
                altitude = self.drone_position.z
                if altitude >= 3:
                    break
                rospy.sleep(1)  # Sleep for 1 second before checking the altitude again.

            # Specify control loop rate. We recommend a low frequency to not over load the FCU with messages. Too many messages will cause the drone to be sluggish.
            self.rate = rospy.Rate(3)
            self.rate.sleep()

            self.drone_position = self.drone.get_current_location()
            rospy.loginfo(f"Drone Position G {self.drone_id}:{self.drone.current_pose_g.pose.pose.position}")

            # self.drone.set_destination(3,0,3,0)
            # self.rate.sleep()
            # while not self.drone.check_waypoint_reached():
                # pass    
            # self.drone.land()


    def setup_publishers(self):
        self.center_pub = rospy.Publisher(f'/drone{self.drone_id}/center', rosMsgPoint, queue_size=10)

    def setup_subscribers(self):
        self.all_vertices_sub = rospy.Subscriber(
            "/all_vertices", 
            String, 
            self.callback_handler.callback_all_vertices
        )

        self.voronoi_centers_sub = rospy.Subscriber(
            "/voronoi_centers", 
            String, 
            self.callback_handler.callback_voronoi_centers
        )
        self.boundary_points_sub = rospy.Subscriber(
            "/boundary_points", 
            String, 
            self.callback_handler.callback_boundary_points
        )
        self.finite_regions_sub = rospy.Subscriber(
            "/finite_regions", 
            String, 
            self.callback_handler.callback_finite_regions
        )
        self.xx_sub = rospy.Subscriber(
            "/xx", 
            String, 
            self.callback_handler.callback_xx
        )
        self.yy_sub = rospy.Subscriber(
            "/yy", 
            String, 
            self.callback_handler.callback_yy
        )

        self.finite_vertices_sub = rospy.Subscriber(
            "/finite_vertices", 
            String, 
            self.callback_handler.callback_finite_vertices
        )
        self.grid_points_sub = rospy.Subscriber(
            "/grid_points", 
            String, 
            self.callback_handler.callback_grid_points
        )
        self.weed_density_sub = rospy.Subscriber(
            "/weed_density", 
            String, 
            self.callback_handler.callback_weed_density
        )

        # Initialize subscribers for the other drones
        for i in range(self.config.getint('INITIAL_SETUP','n_drones')):
            if i != self.drone_id:  # Don't subscribe to our own topic
                rospy.Subscriber(
                    f'/drone{i}/center', 
                    rosMsgPoint, 
                    self.callback_handler.center_callback, 
                    callback_args=i)

    def unsubscribe_initial_topics(self):
        self.all_vertices_sub.unregister()
        self.voronoi_centers_sub.unregister()
        self.finite_regions_sub.unregister()
        self.finite_vertices_sub.unregister()


    @property
    def ready(self):
        return all([self.voronoi_centers.data, self.boundary_points.data, self.finite_regions.data, self.xx.data, self.yy.data])

    def move_and_sense(self):

        if not self.move_and_sense_started:
            self.voronoi_center = self.voronoi_centers[self.drone_id]
            self.voronoi_region = self.finite_regions[self.drone_id]
            self.voronoi_vertices = self.finite_vertices
            self.voronoi_vertices = self.voronoi_vertices.replace('array(', '').replace(')', '')
            self.voronoi_vertices = np.array(ast.literal_eval(self.voronoi_vertices)[self.drone_id])

        # self.planned_path, self.sampled_sensor_values = generate_rectangular_spiral_path(
        self.planned_path, self.sampled_sensor_values = generate_trajectory(
            self.voronoi_center, 
            [self.all_vertices[i] for i in self.voronoi_region], 
            self.grid_resolution, 
            self.grid_points, 
            self.weed_density,
            self.sampling_time, 
            self.time_per_step, 
            self.boundary_tolerance,
            self.drone if self.enable_physics_simulation else None)

        self.all_voronoi_center_tracker.append(self.voronoi_centers)

    def initialize_kalman(self):
        if not self.move_and_sense_started:
            rospy.loginfo("Initialized Kalman Filter")
            self.set_initial_state(self.voronoi_center, self.initial_covariance)
            self.move_and_sense_started = True

    def calculate_new_voronoi_center(self):
        """
        Calculate the new Voronoi center based on the updated sensor function.

        Returns:
        numpy array: New Voronoi center coordinates.
        """
        # Sample points
        cnt = 10000
        xp = min(self.voronoi_vertices[:, 0]) + np.random.rand(cnt) * (max(self.voronoi_vertices[:, 0]) - min(self.voronoi_vertices[:, 0]))
        yp = min(self.voronoi_vertices[:, 1]) + np.random.rand(cnt) * (max(self.voronoi_vertices[:, 1]) - min(self.voronoi_vertices[:, 1]))
        voronoi_region = Polygon(self.voronoi_vertices)
        in_voronoi = [voronoi_region.contains(Point(x, y)) for x, y in zip(xp, yp)]
        xp = xp[in_voronoi]
        yp = yp[in_voronoi]

        # Integrals over Voronoi region
        if self.config.get('FILTER_SETUP','filter_type')=='UKF':
            phi_est = np.array([self.sensor_function(np.array([x,y]), self.kalman_filter.x, self.kalman_filter.P) for x,y in zip(xp,yp)])
        else:
            phi_est = np.array([self.sensor_function(np.array([x,y]), self.kalman_filter.state, self.kalman_filter.covariance) for x,y in zip(xp,yp)])
    
        if self.apply_consensus:
            self.fuse_estimates()

        mv = np.sum(phi_est)
        cx = np.sum(xp * phi_est) / mv
        cy = np.sum(yp * phi_est) / mv

        if np.isnan(cx) or np.isnan(cy):
            rospy.loginfo(f"Encountered KF nan. Setting to last visited point {self.planned_path[1]}")
            (cx,cy) = self.planned_path[-1]

        self.voronoi_center_tracker.append(self.voronoi_center)
        self.voronoi_center = np.array([cx, cy])
        rospy.loginfo(f"New Centre:{self.voronoi_center}:drone{self.drone_id}")

        # Publish the new center
        center_msg = rosMsgPoint(self.voronoi_center[0], self.voronoi_center[1],0)
        self.center_pub.publish(center_msg)

        if not self.deregister_inital_topics:
            self.deregister_inital_topics = True
            self.unsubscribe_initial_topics()

        return self.voronoi_center

    def calculate_voronoi_partitions(self):
        other_centers_array = np.array(list(self.other_centers.values()))
        all_centers = np.concatenate([self.voronoi_center[np.newaxis, :], other_centers_array])
        vor, self.finite_vertices, self.finite_regions, self.voronoi_centers, self.all_vertices = \
            voronoi.compute_voronoi_with_boundaries(all_centers, self.boundary_points)
        self.other_centers = {}

    def fuse_estimates(self):
        # Compute Delaunay triangulation and Laplacian matrix
        laplacian = voronoi.compute_graph_laplacian(self.voronoi_centers)

        # Normalize the weights
        weights = laplacian[self.drone_id] / np.sum(laplacian[self.drone_id])

        # Fuse estimates from all drones
        fused_state = weights[self.drone_id] * self.kalman_filter.state
        fused_covariance = weights[self.drone_id] * self.kalman_filter.covariance
        for drone_id, weight in enumerate(weights):
            if drone_id != self.drone_id:
                fused_state += weight * self.other_states[drone_id]
                fused_covariance += weight * self.other_covariances[drone_id]

        # Update the state and covariance
        self.kalman_filter.state = fused_state
        self.kalman_filter.covariance = fused_covariance