import sys
sys.path.append(r'/home/invisible23man/Robotics/Simulations/decentralised-adaptive-coverage-with-voronoi-partitioning/src/decentralised_adaptive_coverage/scripts/')

from models.Environment import Field
from models import Sensor, Estimators
from tools import voronoi, planner as planpath
from tools.rostools import callbacks
from tqdm import tqdm
from scipy.stats import norm
import numpy as np

import rospy
from iq_gnc.py_gnc_functions_swarm import gnc_api
from iq_gnc.PrintColours import *
from std_msgs.msg import String
from std_srvs.srv import SetBool, SetBoolResponse
from geometry_msgs.msg import Point as rosMsgPoint
from gazebo_msgs.msg import ModelStates

class DroneRosNode:
    def __init__(self, field:Field):
        self.ns = rospy.get_namespace()
        self.drone_id = int(self.ns.strip('/').replace('drone', ''))-1
        self.drone_count = field.drone_count

        # Set up ROS publishers, subscribers and callbacks
        self.setup_publishers()
        self.callback_handler = callbacks.CallbackHandler(self)
        self.setup_subscribers()
        self.deregister_inital_topics = False

        # Initialize dictionaries to store parameters of the other drones
        self.other_centers = {}
        self.other_states = {}
        self.other_covariances = {}

    def setup_publishers(self):
        # Setup Publishers: voronoi center, measurements, variances
        self.pub_center = rospy.Publisher(f'/drone{self.drone_id}/center', rosMsgPoint, queue_size=10)

    def setup_subscribers(self):
        # Setup Subscribers: all voronoi centers, measurements, variances

        self.voronoi_centers = String()
        
        # Initialize subscribers for the other drones
        for i in range(self.drone_count):
            if i != self.drone_id:  # Don't subscribe to our own topic
                rospy.Subscriber(
                    f'/drone{i}/center', 
                    rosMsgPoint, 
                    self.callback_handler.center_callback, 
                    callback_args=i)

    def publish_payload(self, voronoi_center):
        # Setup Publishers: voronoi centers, measurements, variances
        center_msg = rosMsgPoint(voronoi_center[0], voronoi_center[1], voronoi_center[2])
        self.pub_center.publish(center_msg)

class Drone(DroneRosNode):
    def __init__(self, position, field:Field, planner_config, estimator_config):
        super().init(field)
        self.position = position
        self.altitude = 3
        self.drone_positions = field.drone_positions

        self.field_size = field.size
        self.grid_resolution = field.grid_resolution
        self.grid_points = field.grid_points
        self.get_grid_coordinates = field.get_grid_coordinates
        self.true_weed_distribution = field.weed_distribution

        self.voronoi_region = None
        self.voronoi_center_tracker = [position]    
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

        self.estimated_weed_distribution = np.ones_like(self.true_weed_distribution) / np.prod(self.true_weed_distribution.shape)
        self.estimate_uncertainty_tracker = [np.ones_like(self.estimated_weed_distribution)]

        if estimator_name == "GPR":
            self.estimator_sensor = Estimators.GaussianProcessRegressorEstimator(self.estimator_config["kernel"])
        elif estimator_name == "PF":
            self.estimator_sensor = Estimators.ParticleFilterEstimator
            self.num_particles = self.estimator_config["num_particles"]
            self.particles = np.ones((self.true_weed_distribution.shape[0], self.num_particles)) / np.prod(self.true_weed_distribution.shape)
            self.particle_weights = np.ones((self.true_weed_distribution.shape[0], self.num_particles)) / self.num_particles
            self.temperature = self.estimator_config["temperature"]
            self.cooling_rate = self.estimator_config["cooling"]
        else:
            raise ValueError("Invalid estimator name provided. Available options are 'GPR' and 'PF'.")

    def compute_voronoi(self, plot=False):
        voronoi_calculator = voronoi.VoronoiCalculator(self.drone_positions, 'square', self.field_size)
        self.voronoi_region = voronoi_calculator.compute_voronoi()
        if plot:
            voronoi_calculator.plot_voronoi()

    def plan(self, plot=False):
        planner = planpath.Planner(self)
        planner.compute_lawnmower_path()
        if plot:
            planner.plot_lawnmower_path()
        self.lawnmower_path_tracker.append(self.lawnmower_path)

    def sense(self):
        if self.sampling_time < len(self.lawnmower_path): # Cut Short Sampling Path
            self.lawnmower_sampling_path = self.lawnmower_path[:self.sampling_time]
            self.remaining_path = self.lawnmower_path[self.lawnmower_sampling_path.shape[0]:]
        else: # Sample Full, No Estimation
            self.lawnmower_sampling_path = self.lawnmower_path
            self.remaining_path = []

        self.measurements = np.squeeze(np.array([self.true_sensor(point, self.grid_points, self.true_weed_distribution) \
                                        for point in self.lawnmower_sampling_path]))

        # Estimator Training (State Update)
        for point, measurement in zip(self.lawnmower_sampling_path, self.measurements):
            grid_x, grid_y = self.get_grid_coordinates(point)
            index_1d = grid_x * int(self.field_size/self.grid_resolution) + grid_y

            if self.estimator_config["name"] == "PF":
                self.estimated_weed_distribution[index_1d], self.particles[index_1d], self.particle_weights[index_1d], self.temperature = \
                    Estimators.ParticleFilterEstimator.update(measurement, self.particles[index_1d], self.particle_weights[index_1d], self.num_particles, self.temperature, self.cooling_rate)
            else:
                self.estimated_weed_distribution[index_1d] = measurement

        if self.estimator_config["name"] == "GPR":
            self.estimator_sensor.train(X= self.lawnmower_sampling_path, y= self.measurements)

    def estimate(self):
        if (self.measurements.shape[0] < self.lawnmower_path.shape[0] or self.estimation_enabled) and len(self.remaining_path)>0: # Enable Estimation

            self.estimated_measurements, self.estimate_uncertainities = \
                    Sensor.estimate_field(self.remaining_path, self.grid_points, self.estimated_weed_distribution, self.estimator_sensor, mode=self.estimator_config["name"])

            self.measurements = np.concatenate((self.measurements, self.estimated_measurements), axis=0)
            self.lawnmower_path = np.concatenate((self.lawnmower_sampling_path, self.remaining_path), axis=0)
            print(f"Drone {self.id+1} Performing Estimation for {self.remaining_path.shape[0]} waypoints")
        else:
            self.estimated_measurements, self.estimate_uncertainities = [],[]

    def update_voronoi(self):
        if self.scaling_enabled:
            scaling_factor = self.grid_resolution*1000
        else:
            scaling_factor = 1

        if self.estimator_config["weigh_uncertainity"]:
            mv, cx, cy = Estimators.handle_estimate_uncertainities(self, scaling_factor)
        else:
            mv = np.sum(self.measurements)*scaling_factor
            cx = np.sum(np.squeeze(self.lawnmower_path[:, 0]) * self.measurements) / mv
            cy = np.sum(np.squeeze(self.lawnmower_path[:, 1]) * self.measurements) / mv
        
            self.estimate_uncertainty_tracker.append(np.ones_like(self.estimated_weed_distribution))

        new_center = [cx+self.grid_resolution/1000, cy+self.grid_resolution/1000, self.altitude]
        self.voronoi_center_tracker.append(new_center)
        self.position = np.array(new_center)

        self.publish_payload(self.position)  

if __name__ == "__main__":

    # Example usage
    size = 50
    grid_resolution = 1 
    drone_count = 16
    weed_centers = [[size/4, -size/4], [size/4, size/4]]
    weed_cov = [[5, 0], [0, 5]]

    field = Field(size, grid_resolution, drone_count, weed_centers, weed_cov)
    field.plot_field()

    drones = [Drone(pos, id, field) for id,pos in enumerate(field.drone_positions)]

    for i, drone in enumerate(drones[:4]):
        drone.compute_voronoi(plot=True)
        drone.plan()
        print(f"Drone {i+1} Path Length: {len(drone.lawnmower_path)}")

        drone.sense()
        print(f"Drone {i+1} Measurements: {drone.measurements[:10]}")

        drone.update_voronoi()
        print(f"Drone {i+1} Centers: {drone.voronoi_center_tracker}")

