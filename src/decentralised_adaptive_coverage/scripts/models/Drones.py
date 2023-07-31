import ast
import configparser
import numpy as np
import rospy
from iq_gnc.py_gnc_functions_swarm import gnc_api
from iq_gnc.PrintColours import *
from std_msgs.msg import String
from geometry_msgs.msg import Point as rosMsgPoint
from utils import callbacks, voronoi, msg_handler
from shapely.geometry import Point, Polygon
from move import generate_trajectory
from sensor import gaussian_sensor_model
from gazebo_msgs.msg import ModelStates
from models.Robots import Robot

class Drone(Robot):
    """
    A subclass of Robot that represents a drone.
    """
    def __init__(self, config:configparser.ConfigParser()):
        """
        Initialize a Drone with a given configuration.

        Parameters:
        config (configparser.ConfigParser): Configuration for the drone.
        """
        super().__init__(config)
        self.grid_resolution = config.getfloat('INITIAL_SETUP', 'grid_resolution')
        self.sampling_time = config.getint('SIMULATION_SETUP', 'sampling_time')
        self.time_per_step = config.getfloat('SIMULATION_SETUP', 'time_per_step')
        self.boundary_tolerance = config.getfloat('VORONOI_SETUP', 'boundary_tolerance')
        self.enable_physics_simulation = self.config.getboolean('GAZEBO_SETUP','enable_physics_simulation')
        self.apply_consensus = self.config.getboolean('FILTER_SETUP','apply_consensus')

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

        self.all_vertices = String()
        self.voronoi_centers = String()
        self.finite_regions = String()
        self.finite_vertices = String()
        self.boundary_points = String()
        self.xx = String()
        self.yy = String()
        self.grid_points = String()
        self.weed_density = String()

        # Initialize dictionaries to store parameters of the other drones
        self.other_centers = {}
        self.other_states = {}
        self.other_covariances = {}

        # Kalman Settings
        self.set_sensor_function(gaussian_sensor_model)
        self.initial_covariance = np.eye(2)
        self.move_and_sense_started = False

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

    def setup_publishers(self):
        """
        Set up ROS publishers for the drone.

        Returns:
        none
        """
        self.center_pub = rospy.Publisher(f'/drone{self.drone_id}/center', rosMsgPoint, queue_size=10)
        self.state_pub = rospy.Publisher(f'/drone{self.drone_id}/state', String, queue_size=10)
        self.covariance_pub = rospy.Publisher(f'/drone{self.drone_id}/covariance', String, queue_size=10)

    def setup_subscribers(self):
        """
        Set up ROS subscribers for the drone.

        Returns:
        none
        """
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

                if self.apply_consensus:        
                    rospy.Subscriber(
                        f'/drone{i}/state', 
                        String, 
                        self.callback_handler.state_callback, 
                        callback_args=i)
                    rospy.Subscriber(
                        f'/drone{i}/covariance', 
                        String, 
                        self.callback_handler.covariance_callback, 
                        callback_args=i)

    def unsubscribe_initial_topics(self):
        """
        Unsubscribe from initial topics.

        Returns:
        none
        """
        self.all_vertices_sub.unregister()
        self.voronoi_centers_sub.unregister()
        self.finite_regions_sub.unregister()
        self.finite_vertices_sub.unregister()

    @property
    def ready(self):
        return all([self.voronoi_centers.data, self.boundary_points.data, self.finite_regions.data, self.xx.data, self.yy.data])

    def move_and_sense(self):
        """
        Move the drone and sense the environment.

        Returns:
        none
        """
        if not self.move_and_sense_started:
            self.voronoi_center = self.voronoi_centers[self.drone_id]
            self.voronoi_region = self.finite_regions[self.drone_id]
            self.voronoi_vertices = self.finite_vertices
            self.voronoi_vertices = self.voronoi_vertices.replace('array(', '').replace(')', '')
            self.voronoi_vertices = np.array(ast.literal_eval(self.voronoi_vertices)[self.drone_id])

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
        """
        Initialize the Kalman filter for the drone.

        Returns:
        none
        """
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

        if self.apply_consensus:
            # Publish the state for current drone
            state_str = msg_handler.encode_data(self.kalman_filter.x.tolist())
            covariance_str = msg_handler.encode_data(self.kalman_filter.P.tolist())

            # Publish the state and covariance
            self.state_pub.publish(state_str)
            self.covariance_pub.publish(covariance_str)

            rospy.loginfo(f"State and Covariance Published Drone:{self.drone_id}")

            self.apply_covariance_intersection()

        # Integrals over Voronoi region
        if self.config.get('FILTER_SETUP','filter_type')=='UKF':
            phi_est = np.array([self.sensor_function(np.array([x,y]), self.kalman_filter.x, self.kalman_filter.P) for x,y in zip(xp,yp)])

        mv = np.sum(phi_est)
        cx = np.sum(xp * phi_est) / mv
        cy = np.sum(yp * phi_est) / mv

        if np.isnan(cx) or np.isnan(cy):
            rospy.loginfo(f"Encountered KF nan for drone{self.drone_id}. Setting to last visited point {self.planned_path[1]}")
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
        """
        Calculate the Voronoi partitions for the drone.

        Returns:
        none
        """
        other_centers_array = np.array(list(self.other_centers.values()))
        all_centers = np.concatenate([self.voronoi_center[np.newaxis, :], other_centers_array])
        vor, self.finite_vertices, self.finite_regions, self.voronoi_centers, self.all_vertices = \
            voronoi.compute_voronoi_with_boundaries(all_centers, self.boundary_points)
        self.other_centers = {}

    def apply_covariance_intersection(self):
        """
        Apply covariance intersection to the drone's estimates.

        Returns:
        none
        """
        
        # Receive the states from other drones
        while len(self.other_states) < self.config.getint('INITIAL_SETUP','n_drones') - 1:
            rospy.sleep(0.1)  # Sleep for a short duration before checking again
            rospy.loginfo(f"Waiting for other drones to send x,P:drone{self.drone_id}:: \
                            x,P:{len(self.other_states)},{len(self.other_covariances)}")

            # Apply Sensor Fusiom
        self.fuse_estimates()

    def fuse_estimates(self):
        """
        Fuse estimates from all drones.

        Returns:
        none
        """
        # Compute Delaunay triangulation and Laplacian matrix
        # laplacian = voronoi.compute_graph_laplacian(self.voronoi_centers)
        laplacian = voronoi.compute_laplacian(self.voronoi_centers)

        # Compute the absolute values of the weights
        weights = np.abs(laplacian[self.drone_id])

        # Normalize the weights
        weights = weights / np.sum(weights)

        # Fuse estimates from all drones
        fused_state = weights[self.drone_id] * self.kalman_filter.x
        fused_covariance = weights[self.drone_id] * self.kalman_filter.P
        for drone_id, weight in enumerate(weights):
            if drone_id != self.drone_id:
                fused_state += weight * np.array(self.other_states[drone_id])
                fused_covariance += weight * np.array(self.other_covariances[drone_id])

        # Update the state and covariance
        self.kalman_filter.x = fused_state
        self.kalman_filter.P = fused_covariance

        rospy.loginfo(f"Sensor Fusion Complete Drone:{self.drone_id}")