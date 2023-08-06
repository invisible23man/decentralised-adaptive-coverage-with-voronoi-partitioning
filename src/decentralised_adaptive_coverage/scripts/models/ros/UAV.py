from tqdm import tqdm
import numpy as np
import rospy

from iq_gnc.py_gnc_functions_swarm import gnc_api
from iq_gnc.PrintColours import *

from models import Sensor, Estimators, Environment
from tools import voronoi, planner
from tools.rostools.callbacks import CallbackHandler
from tools.rostools.actions import Action
from tools.rostools.services import Service
from tools.rostools.publishers import Publisher
from tools.rostools.subscribers import Subscriber
from tools.rostools.navigation import navigate_to_destination

class DroneRosNode:
    def __init__(self, field: Environment.Field):
        self.ns = rospy.get_namespace()
        self.drone_id = int(self.ns.strip('/').replace('drone', ''))-1
        self.drone_count = field.drone_count
        self.drone_positions = field.drone_positions

        # Set up modules
        self.callback_handler = CallbackHandler(self) 
        self.action = Action(self)
        self.service = Service(self)
        self.publisher = Publisher(self)
        self.subscriber = Subscriber(self)

        # Set up services, publishers, subscribers and actions
        self.action.setup_actions()
        self.service.setup_services()
        self.publisher.setup_publishers()
        self.subscriber.setup_subscribers()

        self.deregister_inital_topics = False

        # Initialize dictionaries to store parameters of the other drones
        self.other_centers = {}
        self.other_states = {}
        self.other_covariances = {}

class Drone(DroneRosNode):
    def __init__(self, field: Environment.Field, planner_config, estimator_config):
        super().__init__(field)
        self.altitude = 3

        self.field_size = field.size
        self.grid_resolution = field.grid_resolution
        self.grid_points = field.grid_points
        self.get_grid_coordinates = field.get_grid_coordinates
        self.true_weed_distribution = field.weed_distribution

        self.voronoi_region = None
        self.voronoi_center_tracker = [self.voronoi_center]
        self.lawnmower_path = None
        self.lawnmower_path_tracker = []
        self.planner_config = planner_config

        self.sampling_time = field.sampling_time
        self.true_sensor = Sensor.sense_field

        self.measurements = []
        self.scaling_enabled = False
        self.estimation_enabled = False

        self.initialize_estimator(estimator_config)

    def initialize_estimator(self, estimator_config):
        self.estimator_config = estimator_config
        estimator_name = self.estimator_config.get("name", "None")

        self.estimated_weed_distribution = np.ones_like(
            self.true_weed_distribution) / np.prod(self.true_weed_distribution.shape)
        self.estimate_uncertainty_tracker = [
            np.ones_like(self.estimated_weed_distribution)]

        if estimator_name == "GPR":
            self.estimator_sensor = Estimators.GaussianProcessRegressorEstimator(
                self.estimator_config["kernel"]
            )
        elif estimator_name == "Particle Filter":
            self.estimator_sensor = Estimators.ParticleFilterEstimator(
                self.true_weed_distribution.shape[0],                
                self.estimator_config.get("num_particles", 1000),
                self.estimator_config.get("temperature", 1.0),
                self.estimator_config.get("cooling", 0.99)
            )
        else:
            raise ValueError(
                "Invalid estimator name provided. Available options are 'GPR' and 'Particle Filter'.")

    def initialize_iteration(self, iteration):
        self.update_voronoi_completed = False
        self.current_iteration = iteration

    def compute_voronoi(self, plot=False):
        voronoi_calculator = voronoi.VoronoiCalculator(
            self.drone_positions, 'square', self.field_size)
        self.voronoi_region = voronoi_calculator.compute_voronoi()
        if plot:
            voronoi_calculator.plot_voronoi()

    def plan(self, plot=False):
        pathplanner = planner.Planner(self)
        pathplanner.compute_lawnmower_path()
        if plot:
            pathplanner.plot_lawnmower_path()
        self.lawnmower_path_tracker.append(self.lawnmower_path)

    def sense(self):
        # Cut Short Sampling Path
        if self.sampling_time < len(self.lawnmower_path):
            self.lawnmower_sampling_path = self.lawnmower_path[:self.sampling_time]
            self.remaining_path = self.lawnmower_path[self.lawnmower_sampling_path.shape[0]:]
        else:  # Sample Full, No Estimation
            self.lawnmower_sampling_path = self.lawnmower_path
            self.remaining_path = []

        self.measurements = np.squeeze(
            np.array([self.true_sensor(point[:2], self.grid_points, self.true_weed_distribution)
                for point in self.lawnmower_sampling_path])
            )

        # Estimator Training (State Update)
        for point, measurement in zip(self.lawnmower_sampling_path, self.measurements):
            grid_x, grid_y = self.get_grid_coordinates(point)
            index_1d = grid_x * int(self.field_size /
                                    self.grid_resolution) + grid_y

            if self.estimator_config["name"] == "Particle Filter":
                self.estimated_weed_distribution[index_1d], self.estimator_sensor.particles[index_1d], \
                    self.estimator_sensor.particle_weights[index_1d], self.temperature = \
                        self.estimator_sensor.update(
                            measurement, 
                            index_1d
                        )

            else:
                self.estimated_weed_distribution[index_1d] = measurement

        if self.estimator_config["name"] == "GPR":
            self.estimator_sensor.train(
                X=self.lawnmower_sampling_path, y=self.measurements)

    def estimate(self):
        # Enable Estimation
        if (self.measurements.shape[0] < self.lawnmower_path.shape[0] or self.estimation_enabled) and len(self.remaining_path) > 0:

            self.estimated_measurements, self.estimate_uncertainities = \
                Sensor.estimate_field(
                self,
                self.remaining_path, 
                self.grid_points, 
                self.estimated_weed_distribution,
                self.estimator_sensor, 
                mode=self.estimator_config["name"]
            )

            self.measurements = np.concatenate(
                (self.measurements, self.estimated_measurements), axis=0)
            self.lawnmower_path = np.concatenate(
                (self.lawnmower_sampling_path, self.remaining_path), axis=0)
            print(
                f"Drone {self.drone_id+1} Performing Estimation for {self.remaining_path.shape[0]} waypoints")
        else:
            self.estimated_measurements, self.estimate_uncertainities = np.empty_like(
                self.measurements), np.empty_like(self.measurements)


    def update_voronoi(self):
        if self.scaling_enabled:
            scaling_factor = self.grid_resolution*1000
        else:
            scaling_factor = 1

        if self.estimator_config["weigh_uncertainity"]:
            mv, cx, cy = Estimators.handle_estimate_uncertainities(
                self, scaling_factor)
        else:
            mv = np.sum(self.measurements)*scaling_factor
            cx = np.sum(np.squeeze(
                self.lawnmower_path[:, 0]) * self.measurements) / mv
            cy = np.sum(np.squeeze(
                self.lawnmower_path[:, 1]) * self.measurements) / mv

            self.estimate_uncertainty_tracker.append(
                np.ones_like(self.estimated_weed_distribution))

        new_center = [cx+self.grid_resolution/1000,
                      cy+self.grid_resolution/1000, self.altitude]
        self.voronoi_center_tracker.append(new_center)
        self.voronoi_center = np.array(new_center)

        self.publisher.publish_payload(self.voronoi_center)

class GNCDrone(Drone):
    def __init__(self, field: Environment.Field, planner_config, estimator_config):
        super().__init__(field, planner_config, estimator_config)
        self.gnc_drone = gnc_api(self.drone_id)
        self.initialize_gnc_drone()

    def initialize_gnc_drone(self):
        # Wait for FCU connection.
        self.gnc_drone.wait4connect()
        # Wait for the mode to be switched.
        # drone.wait4start()
        self.gnc_drone.set_mode("GUIDED")

        # Setup Subscribers and Get Transformation Matrix
        self.subscriber.setup_gnc_drone_subscribers()

        # Create local reference frame.
        self.gnc_drone.initialize_local_frame()
        # Request takeoff with an altitude of 3m.
        self.gnc_drone.takeoff(self.altitude)

         # Wait for the drone to reach the desired takeoff height.
        while True:
            if self.gnc_drone.get_current_location().z > self.altitude:
                break
            rospy.sleep(1)  # Sleep for 1 second before checking the altitude again.

    def sense(self):
        # Cut Short Sampling Path
        if self.sampling_time < len(self.lawnmower_path):
            self.lawnmower_sampling_path = self.lawnmower_path[:self.sampling_time]
            self.remaining_path = self.lawnmower_path[self.lawnmower_sampling_path.shape[0]:]
        else:  # Sample Full, No Estimation
            self.lawnmower_sampling_path = self.lawnmower_path
            self.remaining_path = []

        self.measurements = []

        for i, point in tqdm(enumerate(self.lawnmower_sampling_path), 
                             desc=f"Drone {self.drone_id+1} Sampling Progress", 
                             total=len(self.lawnmower_sampling_path)):            # Use the gnc_api to move the drone to the current point
            reached_waypoint = False
            while not reached_waypoint:
                reached_waypoint = navigate_to_destination(x=point[0], y=point[1], z=point[2], gnc_drone=self.gnc_drone)
                rospy.sleep(3)

            # Then sense the environment at the current point
            measurement = self.true_sensor(point[:2], self.grid_points, self.true_weed_distribution)

            # Append the measurement to the measurements list
            self.measurements.append(measurement)

            # Use the measurement to update the estimator
            grid_x, grid_y = self.get_grid_coordinates(point)
            index_1d = grid_x * int(self.field_size / self.grid_resolution) + grid_y

            if self.estimator_config["name"] == "Particle Filter":
                self.estimated_weed_distribution[index_1d], self.estimator_sensor.particles[index_1d], \
                    self.estimator_sensor.particle_weights[index_1d], self.temperature = \
                        self.estimator_sensor.update(measurement, index_1d)
            else:
                self.estimated_weed_distribution[index_1d] = measurement

        # Train the GPR estimator if necessary
        if self.estimator_config["name"] == "GPR":
            self.measurements = np.squeeze(np.array(self.measurements))
            self.estimator_sensor.train(X=self.lawnmower_sampling_path, y=self.measurements)

