import numpy as np
from utils import plots, voronoi
from scipy.stats import multivariate_normal

def create_square_grid(size, grid_resolution):
    x_values = np.arange(-size/2, size/2 + grid_resolution, grid_resolution)
    y_values = np.arange(-size/2, size/2  + grid_resolution, grid_resolution)
    grid_points = np.array([(x, y) for y in y_values for x in x_values])
    return x_values, y_values, grid_points

def create_circular_grid(center=(0,0), radius=25, resolution=1):
    """Create a grid covering the circular field."""
    x_values = np.arange(-radius, radius + resolution, resolution)
    y_values = np.arange(-radius, radius + resolution, resolution)
    grid_points = np.array([(x, y) for x in x_values for y in y_values if np.sqrt(x**2 + y**2) <= radius])

    return  x_values, y_values, grid_points

def generate_weed_distribution(weed_centers, weed_cov, X, Y, sensing_radius, plot=False):
    # Save the current state of the random number generator
    current_random_state = np.random.get_state()
    
    weed_distribution = np.zeros(X.shape)
    for center in weed_centers:
        rv = multivariate_normal(center, weed_cov)
        for i in range(X.shape[0]):
            for j in range(X.shape[1]):
                if np.sqrt(X[i, j]**2 + Y[i, j]**2) <= sensing_radius:
                    weed_distribution[i, j] = rv.pdf([X[i, j], Y[i, j]])
                else:
                    weed_distribution[i, j] = np.nan

    np.random.set_state(current_random_state)
    return weed_distribution

def distribute_drones(n, r, plot=False, grid_resolution=0.1):
    """
    Distributes 'n' drones evenly within a circular area of radius 'r' and aligns their positions with the center of the grid.

    Args:
        n (int): Number of drones.
        r (float): Radius of the circular area.
        grid_resolution (float): Resolution of the grid.
        plot (bool): Whether to visualize the initial positions. Defaults to False.

    Returns:
        numpy.ndarray: Array of shape (n, 2) containing the Cartesian coordinates of the drones.

    """
    total_area = np.pi * r**2
    area_per_drone = total_area / n
    radii_per_drone = np.sqrt(area_per_drone / np.pi)

    angles = np.linspace(0, 2*np.pi, n+1)[:-1]  # equally spaced angles
    radii = np.full(n, radii_per_drone)  # all drones have the same radius

    # convert to cartesian coordinates
    x = radii * np.cos(angles)
    y = radii * np.sin(angles)

    # Adjust positions to align with the center of the grid
    x = np.round(x / grid_resolution) * grid_resolution + (grid_resolution / 2)
    y = np.round(y / grid_resolution) * grid_resolution + (grid_resolution / 2)

    positions = np.column_stack((x, y))

    # Visualize initial positions
    if plot:
        plots.visualize_swarm(positions, r)

    return positions

def initial_setup(config, **kwargs):
    """
    Performs the initial setup for the drone swarm optimization algorithm.

    Args:
        n (int, optional): Number of drones. Defaults to 20.
        r (float, optional): Radius of the circular area. Defaults to 1.
        filter_type (str, optional): Type of the filter to be used. Should be either 'Kalman' or 'Particle'.
        **kwargs: Variable length argument list. 'num_particles' should be provided if filter_type is 'Particle'.

    Returns:
        tuple: A tuple containing the following elements:
            - vor (scipy.spatial.Voronoi): Voronoi diagram computed within the circular boundaries.
            - finite_vertices (numpy.ndarray): Array of shape (m, 2) containing the vertices of finite Voronoi regions.
            - finite_regions (list): List of finite Voronoi regions.
            - voronoi_centers (numpy.ndarray): Array of shape (n, 2) containing the Voronoi centers.
            - xx (numpy.ndarray): 2D array of shape (n, m) representing the X coordinates of grid points.
            - yy (numpy.ndarray): 2D array of shape (n, m) representing the Y coordinates of grid points.
            - grid_points (numpy.ndarray): Array of shape (k, 2) containing the grid points within the circular boundary.
            - weed_density (numpy.ndarray): Array of shape (k,) containing the weed density at each grid point.
            - initial_estimates (list): List of initial estimates. Each element is either a tuple of mean and covariance (for Kalman filter), or an array of particles (for particle filter).

    """
    plot = kwargs['plots'] if 'plots' in kwargs else False
    n = config.getint('INITIAL_SETUP', 'n_drones')
    r = config.getfloat('INITIAL_SETUP', 'r_area')
    grid_resolution = config.getfloat('INITIAL_SETUP', 'grid_resolution')
    num_gaussians = config.getint('INITIAL_SETUP', 'num_gaussians')
    bandwidth = config.getfloat('INITIAL_SETUP', 'bandwidth')
    weed_centers = [[-r/2, -r/2], [r/2, r/2]]
    weed_cov = [[r/4, 0], [0, r/4]]

    # Get initial positions
    initial_positions = distribute_drones(n, r, plot, grid_resolution)

    # Compute Voronoi diagram within the circular boundaries
    r_exterior = 1.7*r 
    boundary_points = np.array([[r_exterior*np.cos(theta), r_exterior*np.sin(theta)] for theta in np.linspace(0, 2*np.pi, 100)])

    vor, finite_vertices, finite_regions, voronoi_centers, all_vertices = \
        voronoi.compute_voronoi_with_boundaries(
            initial_positions, boundary_points, plot, r)

    x_values, y_values, grid_points = create_circular_grid(center=(0,0), radius=r, resolution=grid_resolution)
    xx, yy = np.meshgrid(x_values, y_values)

    # Function generate weed distribution(AOIs) within the sensing area
    weed_distribution = generate_weed_distribution(weed_centers, weed_cov, xx, yy, r, plot=plot)

    return vor, finite_vertices, finite_regions, voronoi_centers, xx, yy, grid_points, weed_distribution, boundary_points, all_vertices
