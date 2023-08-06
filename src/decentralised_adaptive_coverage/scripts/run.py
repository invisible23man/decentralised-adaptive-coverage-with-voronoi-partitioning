from models import Environment,UAV
from tools import utils
from tqdm import tqdm
import os
import rospy

import warnings
from sklearn.exceptions import ConvergenceWarning

if __name__ == "__main__":

    # Example usage
    size = 50
    grid_resolution = 1 
    drone_count = 6
    formation_pattern = "circle"
    # weed_centers = [[-size/4, size/4], [size/4, -size/4]]
    # weed_centers = [[-15, 15], [10, -10]]
    # weed_centers = [[-8, -15], [15, 15]] # 16 Drones
    weed_centers = [[-8, -5], [20, 22]] # 8 Drones
    weed_cov = [[5, 0], [0, 5]]
    iterations = 5
    sampling_time = 3000
    disable_warnings = True

    if disable_warnings:
        warnings.filterwarnings("ignore", category=ConvergenceWarning)

    planner_config = {
        # "reordermode": "SpiralOutward", # Doesen't Work. Need more proper TSP solver, planning
        # "reordermode": "SpiralOutSimple",
        # "reordermode":"NearestNeighbor",
        # "reordermode": "SpiralOutA*",
        "reordermode": None,
        "formation_pattern": formation_pattern
    }

    estimator_config = {
        # "weigh_uncertainity":"individually",
        # "weigh_uncertainity":"partitionwise",
        "weigh_uncertainity":None,
        
        # "name": "Particle Filter",
        # "num_particles":2000,
        # "temperature": 1.0,
        # "cooling": 0.99,
        
        "name": "GPR",
        "kernel": "C(1.0, (1e-2, 1e2)) * RBF(10, (1e-2, 1e2))"        
    }

    EXPERIMENT_LOGGING_DIR = '/home/invisible23man/Robotics/Simulations/decentralised-adaptive-coverage-with-voronoi-partitioning/src/decentralised_adaptive_coverage/outputs/experiment_logging'
    EXPERIMENT_TIMESTAMP = ''
    EXPERIMENT_FILTERTAG = f's-{sampling_time}-it{iterations}-{utils.generate_experiment_tag(estimator_config)}'
    EXPERIMENT_FILENAME = os.path.join(EXPERIMENT_LOGGING_DIR,EXPERIMENT_TIMESTAMP,
                                       f'{EXPERIMENT_FILTERTAG}-data.pkl')
    ANIMATION2D_FILENAME = os.path.join(EXPERIMENT_LOGGING_DIR,EXPERIMENT_TIMESTAMP,
                                        f'{EXPERIMENT_FILTERTAG}-animation2d.gif')
    ANIMATION3D_FILENAME = os.path.join(EXPERIMENT_LOGGING_DIR,EXPERIMENT_TIMESTAMP,
                                        f'{EXPERIMENT_FILTERTAG}-animation3d.gif')

    field = Environment.Field(size, grid_resolution, drone_count, formation_pattern, weed_centers, weed_cov, sampling_time)
    # field.plot_field()

    drones = [UAV.Drone(id, pos, field, planner_config, estimator_config) for id,pos in enumerate(field.drone_positions)]

    for iteration in tqdm(range(iterations)):
        print(f"\nIteration {iteration+1}")

        for i, drone in enumerate(drones):
            drone.compute_voronoi(plot=False)
            drone.plan(plot=False)
            # print(f"Drone {i+1} Path Length: {len(drone.lawnmower_path)}")

            if len(drone.lawnmower_path) == 0:
                drone.voronoi_center_tracker.append(drone.voronoi_center)
                drone.measurements = []
                field.drone_positions[i] = drone.voronoi_center
                continue   

            drone.sense()
            # print(f"Drone {i+1} Measurements: {drone.measurements[:10]}")

            drone.estimate()

            drone.update_voronoi()
            # print(f"Drone {i+1} Centers: {drone.voronoi_center_tracker}")

            # Drone Posiiton update to be moved ot outer loop for asynchronous update later
            field.drone_positions[i] = drone.voronoi_center

        # Update the drone positions and path measurements in the field
        field.update_drone_positions([drone.voronoi_center for drone in drones])
        field.update_path_and_measurements(drones)

        # field.plot_field()

    # Save data
    field.save_data(EXPERIMENT_FILENAME)

    field.animate_field_2d(plot_voronoi=True, filename=ANIMATION2D_FILENAME)
    field.animate_field_3d(plot_voronoi=True, filename=ANIMATION3D_FILENAME)
