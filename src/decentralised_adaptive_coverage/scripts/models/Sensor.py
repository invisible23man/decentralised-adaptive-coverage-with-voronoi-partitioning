from scipy.interpolate import griddata
from scipy.spatial.distance import cdist
import numpy as np

def sense_field(coordinate, grid_points, weed_distribution, method='nearest'):
    """
    Function to sense the weed density at a given coordinate in the field.
    Args:
    coordinate : Coordinate at which the weed density needs to be sensed.
    grid_points : The grid points of the field.
    weed_distribution : The corresponding weed distribution at each grid point.
    method : The method to be used for interpolation. Default is 'nearest'.
    
    Returns:
    sensed_density : The sensed weed density at the given coordinate.
    """
    # Use the scipy griddata function for interpolation
    sensed_density = griddata(grid_points, weed_distribution, coordinate, method=method)
    
    return sensed_density

def estimate_field(drone, path, grid_points, weed_distribution, estimator, mode='Gaussian', sigma=1.0):
    """
    Function to estimate the weed density at a given coordinate in the field.
    Args:
    path : Path for which the weed density needs to be estimated.
    grid_points : The grid points of the field.
    weed_distribution : The corresponding weed distribution at each grid point.
    estimator : An instance of an estimator object, e.g. GaussianProcessRegressorEstimator.
    mode : The mode of estimation to be used. Options are 'Gaussian', 'GPR', and 'Particle Filter'.
    sigma : Standard deviation for Gaussian kernel.

    Returns:
    estimated_density : The estimated weed density at the given coordinate.
    """

    if mode == 'Gaussian':
        estimated_distribution = []
        for coordinate in path:
            # Calculate Gaussian kernel
            distances = cdist(grid_points, np.atleast_2d(coordinate), 'euclidean')
            kernel_values = np.exp(-distances ** 2 / (2 * sigma ** 2))

            # Use the kernel values to calculate a weighted average of the weed distribution
            estimated_density_at_coordinate = np.average(weed_distribution, weights=kernel_values.flatten())
            estimated_distribution.append(estimated_density_at_coordinate)
        estimated_distribution_uncertainties = None  # Gaussian method doesn't provide uncertainty

    elif mode == 'GPR':
        estimated_distribution, estimated_distribution_uncertainties = estimator.predict(np.atleast_2d(path))
    
    elif mode == "Particle Filter":
        estimated_distribution, estimated_distribution_uncertainties = [], []
        for coordinate in path:
            grid_x, grid_y = drone.get_grid_coordinates(coordinate)
            index_1d = grid_x * int(drone.field_size/drone.grid_resolution) + grid_y

            # Compute the weighted average (i.e., estimate) for the current grid index
            estimated_weed_density = np.average(estimator.particles[index_1d], weights=estimator.particle_weights[index_1d])
            estimated_distribution.append(estimated_weed_density)

            # Compute the uncertainty for the current grid index
            uncertainty = np.sqrt(np.average((estimator.particles[index_1d] - estimated_weed_density)**2, weights=estimator.particle_weights[index_1d]))
            estimated_distribution_uncertainties.append(uncertainty)
        estimated_distribution_uncertainties = np.array(estimated_distribution_uncertainties)
        
    else:
        raise ValueError(f"Invalid mode: {mode}. Options are 'Gaussian', 'GPR', and 'Particle Filter'.")

    return estimated_distribution, estimated_distribution_uncertainties

def systematic_resample(weights):
    N = weights.shape[0]
    positions = (np.arange(N) + np.random.random()) / N
    indexes = np.zeros(N, 'i')
    cumulative_sum = np.cumsum(weights)
    i, j = 0, 0
    while i < N:
        if positions[i] < cumulative_sum[j]:
            indexes[i] = j
            i += 1
        else:
            j += 1
    return indexes

if __name__ == '__main__':

    import sys
    sys.path.append(r'/home/invisible23man/Robotics/Simulations/decentralised-adaptive-coverage-with-voronoi-partitioning/src/decentralised_adaptive_coverage/scripts/models')
    from Environment import Field

    # Example usage
    size = 50
    grid_resolution = 1
    drone_count = 16
    weed_centers = [[-15, -15], [15, 15]]
    weed_cov = [[5, 0], [0, 5]]

    field = Field(size, grid_resolution, drone_count, weed_centers, weed_cov)
    field.plot_field()
 
    # Now, let's sense the weed density at a few points
    points_to_sense = [[-12, -12], [0, 0], [10, 10], [-10, -10], [15, 15] , [-15, -15]]
    sensed_densities = [sense_field(point, field.grid_points, field.weed_distribution) for point in points_to_sense]

    print(sensed_densities)
