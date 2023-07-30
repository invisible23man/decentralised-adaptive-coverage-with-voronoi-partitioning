#! /usr/bin/python

import configparser
from multiprocessing import Pool, Value
from typing import List
from initialization import initial_setup
from move import voronoi_coverage_with_rectangular_spirals
from infra.parallelism import parallelize_iterations
from review.robots import Robot
from utils import plots
from tqdm import tqdm

# Load configuration
config = configparser.ConfigParser()
config.read(
    '/home/invisible23man/Robotics/Simulations/decentralised-adaptive-coverage-with-voronoi-partitioning/src/decentralised_adaptive_coverage/scripts/config.ini')

# Get values from the config file
n_drones = config.getint('INITIAL_SETUP', 'n_drones')
r_area = config.getfloat('INITIAL_SETUP', 'r_area')
grid_resolution = config.getfloat('INITIAL_SETUP', 'grid_resolution')
filter_type = config.get('INITIAL_SETUP', 'filter_type')
num_particles = config.getint('INITIAL_SETUP', 'num_particles')

sampling_time = config.getint('SIMULATION_SETUP', 'sampling_time')
time_per_step = config.getfloat('SIMULATION_SETUP', 'time_per_step')

# Perform initial setup
vor, finite_vertices, finite_regions, voronoi_centers, xx, yy, \
    grid_points, weed_density, initial_estimates, boundary_point, all_vertices = \
        initial_setup(config, plots=False)

# Visualize the initial state
plots.visualize_initial_state(all_vertices, finite_vertices, finite_regions, xx, yy, grid_points, weed_density, voronoi_centers, [], r_area)

# Plot Spiral Coverage for initial state
spiral_paths, sensor_values = voronoi_coverage_with_rectangular_spirals(all_vertices, finite_regions, voronoi_centers, 
    grid_resolution, grid_points, weed_density,
    sampling_time, time_per_step)

# Vizualize coverage path
plots.plot_voronoi_and_spirals(all_vertices, finite_vertices, finite_regions, voronoi_centers, spiral_paths, grid_resolution)

# drones:List[Robot] = []
# # Initialize the drones 
# for drone in range(n_drones):
#     drones.append(Robot(all_vertices, voronoi_centers[drone], finite_regions[drone], finite_vertices[drone], config))

# new_centers = []
# for drone in tqdm(drones, desc="Drone Progress"):
#     n_iter = config.getint('ITERATIVE_PROCESS', 'n_iterations')
#     for i in tqdm(range(n_iter)):
#         drone.move_and_sense(vor, grid_points, weed_density)
#         drone.update()
        # new_centers.append(drone.calculate_new_voronoi_center())


# parallelize_iterations(drones, vor, grid_points, weed_density, config)
    
# def control_algorithm(drone:Robot, drones:List[Robot], sampling_time):
#     while True:
#         start_time = time.time()  # Start of the sampling period

#         # The robot moves and takes measurements until the sampling time is over
#         while time.time() - start_time < sampling_time:
#             drone.move_and_sense(vor, grid_points, weed_density)
#             # Take measurements and update parameters based on the robot's own sensor information
#             drone.update()

        # After the sampling time is over, the robot checks its neighbors
        # and uses their information to update parameters
        # neighbor_parameters = [neighbor.parameters for neighbor in drones if neighbor != drone]
        # drone.perform_consensus(neighbor_parameters)    

# threads = []
# for drone in drones:
#     thread = threading.Thread(target=control_algorithm, args=(drone, drones, sampling_time))
#     thread.start()
#     threads.append(thread)

# # Wait for all threads to finish
# for thread in threads:
#     thread.join()


# # Start the iterative process of optimizing Voronoi centers
# t_step = config.getfloat('ITERATIVE_PROCESS', 'time_step')
# t_limit = config.getfloat('ITERATIVE_PROCESS', 'time_limit')
# beta = config.getfloat('ITERATIVE_PROCESS', 'beta')
# t_span = (0, config.getfloat('ITERATIVE_PROCESS', 't_final'))

# # Initialize the Voronoi center list
# voronoi_centers_list = [voronoi_centers]

# # Perform the optimization in a loop
# n_iter = config.getint('ITERATIVE_PROCESS', 'n_iterations')
# for i in tqdm(range(n_iter)):
#     # Find neighbors
#     neighbors = voronoi.compute_voronoi_neighbours(voronoi_centers, distance_threshold=0.25)

#     # Perform the optimization
#     voronoi_centers = optimize_voronoi_centers_consensus(
#         voronoi_centers=voronoi_centers,
#         finite_vertices=finite_vertices,
#         finite_regions=finite_regions,
#         grid_points=grid_points,
#         weed_density=weed_density,
#         sensor_func=sense,
#         neighbours=neighbors,
#         time_step=t_step,
#         time_limit=t_limit,
#         beta=beta,
#         t_span=t_span
#     )

#     # Update Voronoi tesselation
#     vor, finite_vertices, finite_regions, voronoi_centers = voronoi.compute_voronoi_with_boundaries(voronoi_centers, boundary_points)

#     # Update the Voronoi center list
#     voronoi_centers_list.append(voronoi_centers)

# # Visualize the final state
# plots.visualize_final_state(vor, finite_vertices, finite_regions, xx, yy, grid_points, weed_density, voronoi_centers_list)
