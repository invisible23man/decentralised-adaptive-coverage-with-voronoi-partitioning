import sys
sys.path.append(r'/home/invisible23man/Robotics/Simulations/decentralised-adaptive-coverage-with-voronoi-partitioning/src/decentralised_adaptive_coverage/scripts/')

from models.Environment import Field
from models.Sensor import sense_field
from tools import voronoi, planner as planpath
from tqdm import tqdm
import numpy as np


class Drone:
    def __init__(self, position, field:Field, id):
        self.id = id
        self.position = position
        self.altitude = 3
        self.drone_positions = field.drone_positions

        self.field_size = field.size
        self.grid_resolution = field.grid_resolution
        self.grid_points = field.grid_points
        self.true_weed_distribution = field.weed_distribution
        
        self.voronoi_region = None
        self.voronoi_center_tracker = [position]    
        self.lawnmower_path = None
        self.lawnmower_path_tracker = []
        
        self.sampling_time = field.sampling_time
        self.true_sensor = sense_field
        self.measurements = []
        self.scaling_enabled = False
        self.estimation_enabled = False 
        
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
        if self.sampling_time: # Cut Short Sampling Path
            self.lawnmower_sampling_path = self.lawnmower_path[:min(self.sampling_time, len(self.lawnmower_path))]
        else:
            self.lawnmower_sampling_path = self.lawnmower_path
        self.measurements = np.squeeze(np.array([self.true_sensor(point, self.grid_points, self.true_weed_distribution) for point in self.lawnmower_sampling_path]))

    def estimate(self):
        if self.measurements.shape[0] < self.lawnmower_path.shape[0] or self.estimation_enabled: # Enable Estimation
            self.remaining_path = self.lawnmower_path[self.lawnmower_sampling_path.shape[0]:]
            self.estimated_measurements = np.squeeze(np.array([self.true_sensor(point, self.grid_points, self.true_weed_distribution) for point in self.remaining_path]))
                
            self.measurements = np.concatenate((self.measurements, self.estimated_measurements), axis=0)
            self.lawnmower_path = np.concatenate((self.lawnmower_sampling_path, self.remaining_path), axis=0)
            print(f"Drone {self.id+1} Performing Estimation for {self.remaining_path.shape[0]} waypoints")

    def update_voronoi(self):
        if self.scaling_enabled:
            scaling_factor = self.grid_resolution*1000
        else:
            scaling_factor = 1

        mv = np.sum(self.measurements)*scaling_factor
        cx = np.sum(np.squeeze(self.lawnmower_path[:, 0]) * self.measurements) / mv
        cy = np.sum(np.squeeze(self.lawnmower_path[:, 1]) * self.measurements) / mv
        new_center = [cx+self.grid_resolution/1000, cy+self.grid_resolution/1000, self.altitude]
        self.voronoi_center_tracker.append(new_center)
        self.position = np.array(new_center)

if __name__ == "__main__":

    # Example usage
    size = 50
    grid_resolution = 1 
    drone_count = 16
    weed_centers = [[size/4, -size/4], [size/4, size/4]]
    weed_cov = [[5, 0], [0, 5]]

    field = Field(size, grid_resolution, drone_count, weed_centers, weed_cov)
    field.plot_field()

    drones = [Drone(pos, field, id) for id,pos in enumerate(field.drone_positions)]

    for i, drone in enumerate(drones[:4]):
        drone.compute_voronoi(plot=True)
        drone.plan()
        print(f"Drone {i+1} Path Length: {len(drone.lawnmower_path)}")

        drone.sense()
        print(f"Drone {i+1} Measurements: {drone.measurements[:10]}")

        drone.update_voronoi()
        print(f"Drone {i+1} Centers: {drone.voronoi_center_tracker}")


