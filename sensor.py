import numpy as np
from sklearn.neighbors import KernelDensity
from scipy.interpolate import griddata
from utils import plots


def generate_kde_centers(num_centers, radius):
    """
    Generate random centers within a circular boundary.

    Args:
        num_centers (int): Number of centers to generate.
        radius (float): Radius of the circular boundary.

    Returns:
        numpy.ndarray: Array of shape (num_centers, 2) containing the generated centers.

    """
    centers = []
    for _ in range(num_centers):
        # Generate random polar coordinates and convert to Cartesian
        angle = np.random.uniform(0, 2*np.pi)
        # Prevents clustering at center
        rad = radius * np.sqrt(np.random.uniform(0, 1))
        x = rad * np.cos(angle)
        y = rad * np.sin(angle)
        centers.append([x, y])
    return np.array(centers)


def generate_weed_distribution(r, num_gaussians=3, bandwidth=0.085, plot=False):
    """
    Generate a weed concentration distribution within a circular boundary.

    Args:
        r (float): Radius of the circular boundary.
        num_gaussians (int): Number of Gaussian distributions to generxsate.
        bandwidth (float): Bandwidth parameter for the kernel density estimation. Change to modify spread.

    Returns:
        numpy.ndarray: Array of shape (n, 2) containing the grid points.
        numpy.ndarray: Array of shape (n,) containing the corresponding weed densities.

    """
    # Define the grid for estimation
    grid_resolution = 0.01
    x_min, x_max = -r, r
    y_min, y_max = -r, r
    xx, yy = np.meshgrid(np.arange(x_min, x_max, grid_resolution),
                         np.arange(y_min, y_max, grid_resolution))
    grid_points = np.c_[xx.ravel(), yy.ravel()]

    # Generate sample weed concentration data
    weed_concentration = np.concatenate([np.random.normal(loc=center, scale=[0.1, 0.05], size=(100, 2))
                                        for center in generate_kde_centers(num_gaussians, r)])

    # Perform kernel density estimation
    kde = KernelDensity(bandwidth=bandwidth, kernel='gaussian')
    kde.fit(weed_concentration)
    weed_density = np.exp(kde.score_samples(grid_points))

    # Reshape the density values to match the grid shape
    density_map = weed_density.reshape(xx.shape)

    if plot:
        plots.plot_weed_distribution(xx, yy, density_map)

    return grid_points, weed_density



def sensor_func(point, grid_points, weed_density):
    """
    Get the weed concentration at a specific coordinate.

    Args:
        point (numpy.ndarray): Array of shape (2,) containing the coordinate.
        grid_points (numpy.ndarray): Array of shape (n, 2) containing the grid points.
        weed_density (numpy.ndarray): Array of shape (n,) containing the corresponding weed densities.

    Returns:
        float: Weed concentration at the given coordinate.

    """
    return griddata(grid_points, weed_density, point, method='linear')

import numpy as np

def gaussian_sensor_model(point, mean, covariance):
    """
    Evaluate the Gaussian sensor model at a given point.
    
    Parameters:
    point (np.array): The point at which to evaluate the function.
    mean (np.array): The current estimate of the weed concentration center.
    covariance (np.array): The covariance matrix representing the uncertainty in the estimate.
    
    Returns:
    float: The sensor value at the given point.
    """
    k = len(point)
    normalization_factor = 1 / np.sqrt((2 * np.pi) ** k * np.linalg.det(covariance))
    exp_argument = -1/2 * (point - mean).T @ np.linalg.inv(covariance) @ (point - mean)
    
    return normalization_factor * np.exp(exp_argument)

def apply_gaussian_sensor_model(points, mean, covariance,radius=1.5):
    """
    Apply the gaussian sensor model to a set of points.

    Parameters:
    points (np.array): An array of points where each point is an array [x, y].
    mean (np.array): Mean of the Gaussian distribution.
    covariance (np.array): Covariance matrix of the Gaussian distribution.
    radius (float): Radius of the circular area (centered at [0, 0])

    Returns:
    np.array: An array of sensor values.
    """

    sensor_values = []

    for point in points:
        # Check if the point is inside the circular area
        if np.linalg.norm(point - mean) <= radius:
            sensor_value = gaussian_sensor_model(point, mean, covariance)
        else:
            sensor_value = 0  # or any other value that indicates no sensor data

        sensor_values.append(sensor_value)

    return np.array(sensor_values)


