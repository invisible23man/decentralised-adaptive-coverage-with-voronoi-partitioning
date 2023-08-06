#! /usr/bin/python

import rospy
import configparser
import time
import os
from tqdm import tqdm

from models.ros import UAV
from models import Environment
from tools.utils import read_config_file

import warnings
from sklearn.exceptions import ConvergenceWarning

def main():
    # Initializing ROS node.
    rospy.init_node("drone_controller" + rospy.get_namespace().replace('/', ''), anonymous=True)

    # Read configuration file
    config, size, grid_resolution, weed_centers, weed_cov, drone_count, \
        iterations, disable_warnings, planner_config, sampling_time, estimator_config, \
            experiment_filename, animation2d_filename, animation3d_filename = \
                read_config_file('/home/invisible23man/Robotics/Simulations/decentralised-adaptive-coverage-with-voronoi-partitioning/src/decentralised_adaptive_coverage/scripts/config.ini')

    if disable_warnings:
        warnings.filterwarnings("ignore", category=ConvergenceWarning)

    field = Environment.Field(size, grid_resolution, drone_count, planner_config["formation_pattern"], weed_centers, weed_cov, sampling_time)
    # field.plot_field()

    # Initlialize the Drone
    if config.getboolean('GAZEBO','enable_gazebo_simulation'):
        drone = UAV.GNCDrone(field, planner_config, estimator_config)
    else:
        drone = UAV.Drone(field, planner_config, estimator_config)

    # while not rospy.is_shutdown():
    for iteration in tqdm(range(iterations)):
        rospy.loginfo(f"\nIteration {iteration+1}, Drone {drone.drone_id+1}")

        drone.initialize_iteration(iteration)
        drone.compute_voronoi(plot=False)
        drone.plan(plot=False)

        if len(drone.lawnmower_path) == 0:
            drone.voronoi_center_tracker.append(drone.voronoi_center)
            drone.measurements = []
            drone.publisher.publish_payload(drone.voronoi_center)
            continue   

        # rospy.loginfo(f"Started movement and sensing:drone{drone.id}, iter:{iteration}")
        drone.sense()

        drone.estimate()

        # rospy.loginfo(f"Calculating New Center:drone{drone.drone_id}, iter:{i}")
        drone.update_voronoi()

        drone.service.wait_for_update_voronoi_for_all_drones()
        field.update_drone_positions(drone.drone_positions)


    if config.getboolean('GAZEBO','enable_gazebo_simulation'):
        drone.gnc_drone.land()

    # Save data
    if drone.drone_id == 0:
        field.save_data(experiment_filename)

        field.animate_field_2d(plot_voronoi=True, filename=animation2d_filename)
        field.animate_field_3d(plot_voronoi=True, filename=animation3d_filename)

    rospy.loginfo(f"{drone.drone_id} completed run.")
    # rospy.signal_shutdown(reason="Completed Run")
    while not rospy.is_shutdown():
        rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSException:
        print("Service not available after waiting for 5 seconds.")
    except KeyboardInterrupt:
        exit()
