#! /usr/bin/python

import rospy
import configparser
import time
import pickle
import os
from tqdm import tqdm
from std_srvs.srv import SetBool
from models import Drones

from models import Environment,UAV
from tools import utils

import warnings
from sklearn.exceptions import ConvergenceWarning


def main():
    # Initializing ROS node.
    rospy.init_node("drone_controller" + rospy.get_namespace().replace('/', ''), anonymous=True)

    # Load configuration
    # config = configparser.ConfigParser()
    # config.read(
    #     '/home/invisible23man/Robotics/Simulations/decentralised-adaptive-coverage-with-voronoi-partitioning/src/decentralised_adaptive_coverage/scripts/config.ini')
    # n_iter = config.getint('ITERATIVE_PROCESS', 'n_iterations')
    # n_drones = config.getint('INITIAL_SETUP','n_drones') 

    # Example usage
    size = 50
    grid_resolution = 1 
    drone_count = 16
    # weed_centers = [[-size/4, size/4], [size/4, -size/4]]
    weed_centers = [[-15, 15], [10, -10]]
    weed_cov = [[5, 0], [0, 5]]
    iterations = 50
    sampling_time = 50
    disable_warnings = True

    if disable_warnings:
        warnings.filterwarnings("ignore", category=ConvergenceWarning)

    planner_config = {
        # "reordermode": "SpiralOutward"
        # "reordermode":"NearestNeighbor",
        "reordermode": None
    }

    estimator_config = {
        # "weigh_uncertainity":"individually",
        "weigh_uncertainity":"partitionwise",
        
        # "name": "PF",
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

    field = Environment.Field(size, grid_resolution, drone_count, weed_centers, weed_cov, sampling_time)
    field.plot_field()

    id, pos = 1, (1,1)

    # Initlialize the Drone
    drone = UAV.Drone(id, pos, field, planner_config, estimator_config)

    # Wait until the drone is ready
    time.sleep(3)
    # while not drone.ready and not rospy.is_shutdown():
    #     rospy.loginfo("Waiting for initial messages...")
    #     time.sleep(1)

    # Now start the main loop
    # while not rospy.is_shutdown():
    for iteration in tqdm(range(iterations)):
        rospy.loginfo(f"\nIteration {iteration+1}")

        drone.compute_voronoi(plot=False)
        drone.plan(plot=False)

        if len(drone.lawnmower_path) == 0:
            drone.voronoi_center_tracker.append(drone.position)
            drone.measurements = []
            # field.drone_positions[i] = drone.position
            continue   

        # rospy.loginfo(f"Started movement and sensing:drone{drone.id}, iter:{iteration}")
        drone.sense()

        drone.estimate()

        drone.update_voronoi()

        # Wait until all drones have published their state and covariance
        if config.getboolean('FILTER_SETUP','apply_consensus'):
            drone.publish_state_and_covariance()
            counter = 0 
            while len(drone.other_states) < n_drones- 1 or len(drone.other_covariances) < n_drones- 1:
                rospy.sleep(0.1)  # Sleep for a short duration before checking again
                if counter % 15 == 0:
                    rospy.loginfo(f"Waiting for other drones to send x,P:drone{drone.drone_id}:: \
                                    x,P:{len(drone.other_states)},{len(drone.other_covariances)}")
                counter +=1
                time.sleep(1)  # Sleep for a short time to avoid busy waiting

        rospy.loginfo(f"Calculating New Center:drone{drone.drone_id}, iter:{i}")
        drone.calculate_new_voronoi_center()

        # Wait until all drones have updated their centers
        counter = 0 
        while len(drone.other_centers) != drone.config.getint('INITIAL_SETUP','n_drones') - 1:
            if counter % 15 == 0:
                rospy.loginfo(f"Waiting for other drones to update their centers:drone{drone.drone_id}, o_centers:{len(drone.other_centers)},iter:{i}")
            counter +=1
            time.sleep(1)  # Sleep for a short time to avoid busy waiting

        rospy.loginfo(f"Calculating Voronoi Partitions:drone{drone.drone_id}, iter:{i}")
        drone.calculate_voronoi_partitions()
        rospy.loginfo(f"Voronoi Centres for drone{drone.drone_id}: {drone.voronoi_center_tracker}")
        
    # Save the run info
    with open(os.path.join(config.get('RESULTS', 'save_directory'), f"{drone.drone_id}_centers.pkl"), "wb") as f:
        pickle.dump(drone.voronoi_center_tracker, f)
    with open(os.path.join(config.get('RESULTS', 'save_directory'), f"{drone.drone_id}_all_centers.pkl"), "wb") as f:
        pickle.dump(drone.all_voronoi_center_tracker, f)

    if drone.enable_physics_simulation:
        drone.drone.land()
    # time.sleep(5)
    # sys.exit(1)

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        exit()
