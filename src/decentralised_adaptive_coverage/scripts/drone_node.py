#! /usr/bin/python

import rospy
import configparser
import time
import pickle
import os
import sys
from tqdm import tqdm

from models.ros import UAV
from models import Environment
from tools import utils

import warnings
from sklearn.exceptions import ConvergenceWarning
    

def main():
    # Initializing ROS node.
    rospy.init_node("drone_controller" + rospy.get_namespace().replace('/', ''), anonymous=True)

    # Example usage
    size = 50
    grid_resolution = 1 
    drone_count = 5
    formation_pattern = "circle"
    # weed_centers = [[-size/4, size/4], [size/4, -size/4]]
    # weed_centers = [[-15, 15], [10, -10]]
    # weed_centers = [[-8, -15], [15, 15]] # 16 Drones
    weed_centers = [[-8, -5], [20, 22]] # 8 Drones
    weed_cov = [[5, 0], [0, 5]]
    iterations = 5
    sampling_time = 30
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

    EXPERIMENT_LOGGING_DIR = '/home/invisible23man/Robotics/Simulations/decentralised-adaptive-coverage-with-voronoi-partitioning/src/decentralised_adaptive_coverage/outputs/experiment_logging_ros'
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

    # Initlialize the Drone
    drone = UAV.Drone(field, planner_config, estimator_config)

    # while not rospy.is_shutdown():
    for iteration in tqdm(range(iterations)):
        rospy.loginfo(f"\nIteration {iteration+1}, Drone {drone.drone_id+1}")


        drone.compute_voronoi(plot=False)
        drone.plan(plot=False)

        if len(drone.lawnmower_path) == 0:
            drone.voronoi_center_tracker.append(drone.voronoi_center)
            drone.measurements = []
            drone.publish_payload(drone.voronoi_center)
            continue   

        # rospy.loginfo(f"Started movement and sensing:drone{drone.id}, iter:{iteration}")
        drone.sense()

        drone.estimate()

        # rospy.loginfo(f"Calculating New Center:drone{drone.drone_id}, iter:{i}")
        drone.update_voronoi()

        drone.service.wait_for_update_voronoi_for_all_drones()
        field.update_drone_positions(drone.drone_positions)


    # if drone.enable_physics_simulation:
    #     drone.drone.land()

    # Save data
    if drone.drone_id == 0:
        field.save_data(EXPERIMENT_FILENAME)

        field.animate_field_2d(plot_voronoi=True, filename=ANIMATION2D_FILENAME)
        field.animate_field_3d(plot_voronoi=True, filename=ANIMATION3D_FILENAME)

    # # Add a delay before shutdown to ensure that all service calls have been made
    # rospy.sleep(10)

    # rospy.signal_shutdown(reason="Completed Run")
    rospy.loginfo(f"{drone.drone_id} completed run.")
    while True:
        rospy.sleep(1)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSException:
        print("Service not available after waiting for 5 seconds.")
    except KeyboardInterrupt:
        exit()
