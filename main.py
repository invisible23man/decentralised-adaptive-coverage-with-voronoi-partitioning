import configparser
from initialization import initial_setup
from move import voronoi_coverage_with_spirals
from utils import plots, voronoi
from sensor import apply_gaussian_sensor_model, sense
import numpy as np
from optimize import optimize_voronoi_centers_consensus
from tqdm import tqdm


# Load configuration
config = configparser.ConfigParser()
config.read('/home/invisibleman/Robotics/adaptive-coverage-with-voronoi/config.ini')

# Get values from the config file
n_drones = config.getint('INITIAL_SETUP', 'n_drones')
r_area = config.getfloat('INITIAL_SETUP', 'r_area')
filter_type = config.get('INITIAL_SETUP', 'filter_type')
num_particles = config.getint('INITIAL_SETUP', 'num_particles')

# Perform initial setup
vor, finite_vertices, finite_regions, voronoi_centers, xx, yy, grid_points, weed_density, initial_estimates, boundary_points = \
    initial_setup(n=n_drones, r=r_area, filter_type=filter_type, num_particles=num_particles)

# Initial guess for the weed concentration centers
initial_guesses = initial_estimates

# Initialize the sensor model
initial_covariance = 0.1 * np.eye(2)

# # Compute initial sensor readings
# sensor_readings = [apply_gaussian_sensor_model(voronoi_centers, guess, initial_covariance, r_area)
#                    for guess in initial_guesses]

# Visualize the initial state
# plots.visualize_initial_state(vor, finite_vertices, finite_regions, xx, yy, grid_points, weed_density, voronoi_centers, [], r_area)

# Plot Spiral Coverage for initial state 
sampling_time = 10.0  # Total sampling time
time_per_step = 0.1  # Time per step
grid_resolution = 0.1  # Grid resolution
spirals = voronoi_coverage_with_spirals(vor, finite_regions, voronoi_centers, sampling_time, time_per_step, grid_resolution)
plots.plot_voronoi_and_spirals(vor, finite_vertices, finite_regions, voronoi_centers, spirals)    

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
