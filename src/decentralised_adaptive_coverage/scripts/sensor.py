import numpy as np
from scipy.interpolate import griddata
from perlin_noise import PerlinNoise

def sense(point, grid_points, weed_density):
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

def generate_perlin_noise(shape, scale=0.1):
    """
    Generate a 2D Perlin noise array.

    Parameters:
    shape (tuple): Shape of the output array.
    scale (float): Scaling factor for the noise.

    Returns:
    np.array: Array containing the generated Perlin noise.
    """
    output = np.zeros(shape)

    noise = PerlinNoise(octaves=6, seed=np.random.randint(0, 100))

    for i in range(shape):
            output[i] = noise([i * scale])

    return output

def sample_weed_density(sensor_func,  points, grid_points, weed_density, sensor_noise_std_dev=0.1, noise_model='gaussian'):
    """
    Sample the weed density at the given points.

    Parameters:
    sensor_func (callable): Function that describes the sensor measurement process.
    points (np.array): Array of points at which to take measurements.
    sensor_noise_std_dev (float): Standard deviation of the sensor noise.
    noise_model (str): The model of noise to use. Supported values are 'gaussian' and 'perlin'.

    Returns:
    np.array: Sensor measurements at the given points.
    """
    # Calculate the true sensor values
    true_values = sensor_func(points, grid_points, weed_density)

    # Add noise to simulate real-world conditions
    if noise_model == 'gaussian':
        noise = np.random.normal(scale=sensor_noise_std_dev, size=true_values.shape)
    elif noise_model == 'perlin':
        noise = generate_perlin_noise(true_values.shape[0], scale=sensor_noise_std_dev)
    else:
        raise ValueError("Invalid noise model. Supported models are 'gaussian' and 'perlin'.")

    # Return the noisy sensor measurements
    return true_values + noise

