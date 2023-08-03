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
        self.true_sensor = sense_field
        self.measurements = []
        
    def compute_voronoi(self, plot=False):
        voronoi_calculator = voronoi.VoronoiCalculator(self.drone_positions, 'square', self.field_size)
        self.voronoi_region = voronoi_calculator.compute_voronoi()
        if plot:
            voronoi_calculator.plot_voronoi()

    def plan(self):
        planner = planpath.Planner(self)
        planner.compute_lawnmower_path()
        planner.plot_lawnmower_path()
        self.lawnmower_path_tracker.append(self.lawnmower_path)

    def sense(self):
        self.measurements = np.array([self.true_sensor(point, self.grid_points, self.true_weed_distribution) for point in self.lawnmower_path])

    def update_voronoi(self):
        scaling_factor = self.grid_resolution*1000
        mv = np.sum(self.measurements)*scaling_factor
        cx = np.sum(self.lawnmower_path[:, 0] * self.measurements) / mv
        cy = np.sum(self.lawnmower_path[:, 1] * self.measurements) / mv
        new_center = [cx+self.grid_resolution/1000, cy+self.grid_resolution/1000, self.altitude]
        self.voronoi_center_tracker.append(new_center)
        self.position = np.array(new_center)

if __name__ == "__main__":

    # Example usage
    size = 50
    grid_resolution = 1 
    drone_count = 16
    weed_centers = [[-size/4, -size/4], [size/4, size/4]]
    weed_cov = [[5, 0], [0, 5]]

    field = Field(size, grid_resolution, drone_count, weed_centers, weed_cov)

    drones = [Drone(pos, field, id) for id,pos in enumerate(field.drone_positions)]

    for i, drone in enumerate(drones[:4]):
        drone.compute_voronoi(plot=True)
        drone.plan()
        print(f"Drone {i+1} Path Length: {len(drone.lawnmower_path)}")

        drone.sense()
        print(f"Drone {i+1} Measurements: {drone.measurements[:10]}")

        drone.update_voronoi()
        print(f"Drone {i+1} Centers: {drone.voronoi_center_tracker}")


