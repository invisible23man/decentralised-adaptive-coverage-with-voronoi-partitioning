#! /usr/bin/python

import configparser
from initialization import initial_setup
from move import voronoi_coverage
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
coverage_method = config.get('VORONOI_SETUP', 'coverage_method')

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
spiral_paths, sensor_values = voronoi_coverage(all_vertices, finite_regions, voronoi_centers, 
    grid_resolution, grid_points, weed_density,
    sampling_time, time_per_step, coverage_method)

# Vizualize coverage path
plots.plot_voronoi_and_spirals(all_vertices, finite_vertices, finite_regions, voronoi_centers, spiral_paths, grid_resolution)

# # Visualize the final state
# plots.visualize_final_state(vor, finite_vertices, finite_regions, xx, yy, grid_points, weed_density, voronoi_centers_list)
