import configparser
from initialization import initial_setup
from utils import plots
from sensor import apply_gaussian_sensor_model
import numpy as np

# Load configuration
config = configparser.ConfigParser()
config.read('/home/invisibleman/Robotics/adaptive-coverage-with-voronoi/config.ini')

# Get values from the config file
n_drones = config.getint('INITIAL_SETUP', 'n_drones')
r_area = config.getfloat('INITIAL_SETUP', 'r_area')
filter_type = config.get('INITIAL_SETUP', 'filter_type')
num_particles = config.getint('INITIAL_SETUP', 'num_particles')

# Perform initial setup
vor, finite_vertices, finite_regions, voronoi_centers, xx, yy, grid_points, weed_density, initial_estimates = \
    initial_setup(n=n_drones, r=r_area, filter_type=filter_type, num_particles=num_particles)

# Initial guess for the weed concentration centers
initial_guesses = voronoi_centers

# Initialize the sensor model
initial_covariance = 0.1 * np.eye(2)

# Compute initial sensor readings
sensor_readings = [apply_gaussian_sensor_model(voronoi_centers, guess, initial_covariance, r_area)
                   for guess in initial_guesses]

# # Visualize the initial state
plots.visualize_initial_state(vor, finite_vertices, finite_regions, xx, yy, grid_points, weed_density, voronoi_centers, sensor_readings)
