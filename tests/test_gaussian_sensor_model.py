import sys
sys.path.append(r'/home/invisibleman/Robotics/adaptive-coverage-with-voronoi')

from utils.plots import plot_gaussian_sensor_model
from sensor import apply_gaussian_sensor_model
import numpy as np

# Test function
def test_gaussian_sensor_model():
    # Generate a set of points in a grid
    x = np.linspace(-10, 10, 100)
    y = np.linspace(-10, 10, 100)
    points = np.array([[xi, yi] for xi in x for yi in y])

    # Define the mean and covariance of the Gaussian distribution
    mean = np.array([0, 0])
    covariance = np.array([[1, 0], [0, 1]])

    sensor_values = apply_gaussian_sensor_model(points, mean, covariance)

    # Plot the Gaussian sensor model
    plot_gaussian_sensor_model(points, sensor_values)

test_gaussian_sensor_model()
