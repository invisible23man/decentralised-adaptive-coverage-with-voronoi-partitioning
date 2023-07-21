#! /home/invisibleman/anaconda3/envs/drones/bin/python
from typing import List
from move import voronoi_coverage_with_rectangular_spirals
import rospy
from std_msgs.msg import String
from initialization import initial_setup
from robots import Robot
from utils import voronoi, rosutils
import configparser
import time

def main():
    # Initializing ROS node.
    rospy.init_node("drone_controller" + rospy.get_namespace().replace('/', ''), anonymous=True)

    # Load configuration
    config = configparser.ConfigParser()
    config.read(
        '/home/invisibleman/Robotics/adaptive-coverage-with-voronoi/src/decentralised_adaptive_coverage/scripts/config.ini')

    # Get values from the config file
    n_drones = config.getint('INITIAL_SETUP', 'n_drones')
    r_area = config.getfloat('INITIAL_SETUP', 'r_area')
    grid_resolution = config.getfloat('INITIAL_SETUP', 'grid_resolution')
    filter_type = config.get('INITIAL_SETUP', 'filter_type')
    num_particles = config.getint('INITIAL_SETUP', 'num_particles')

    sampling_time = config.getint('SIMULATION_SETUP', 'sampling_time')
    time_per_step = config.getfloat('SIMULATION_SETUP', 'time_per_step')

    drone = rosutils.Drone()

    # # Wait until the drone is ready
    time.sleep(3)
    # while not drone.ready and not rospy.is_shutdown():
    #     rospy.loginfo("Waiting for initial messages...")
    #     time.sleep(1)


    # Now start the main loop
    while not rospy.is_shutdown():
        # rospy.loginfo("Listening {0} {1} for {2}".format(drone.voronoi_centers, drone.boundary_points, drone.ns))
        # vor, finite_vertices, finite_regions, voronoi_centers, all_vertices = voronoi.compute_voronoi_with_boundaries(
        #     drone.voronoi_centers, drone.boundary_points, plot=False)

        # plots.visualize_initial_state(vor, finite_vertices, finite_regions, xx, yy, grid_points, weed_density, voronoi_centers, [], r_area)

        # Plot Spiral Coverage for initial state
        spiral_paths, sensor_values = voronoi_coverage_with_rectangular_spirals(
            drone.all_vertices, 
            drone.finite_regions, 
            drone.voronoi_centers, 
            grid_resolution, 
            drone.grid_points, 
            drone.weed_density,
            sampling_time, time_per_step)

        rospy.loginfo(f"{len(spiral_paths)},{len(sensor_values)}")


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        exit()
