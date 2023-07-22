# %%
import numpy as np
from utils import plots, voronoi
from sklearn.neighbors import KernelDensity

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

def generate_weed_distribution(r, num_gaussians=3, bandwidth=0.085, grid_resolution = 0.1, plot=False):
    """
    Generate a weed concentration distribution within a circular boundary.

    Args:
        r (float): Radius of the circular boundary.
        num_gaussians (int): Number of Gaussian distributions to generxsate.
        bandwidth (float): Bandwidth parameter for the kernel density estimation. Change to modify spread.
        grid_resolution (float): factor determining how fine or coarse your grid is

    Returns:
        numpy.ndarray: 2D array of shape (n, m) representing the X coordinates of grid points.
        numpy.ndarray: 2D array of shape (n, m) representing the Y coordinates of grid points.
        numpy.ndarray: 2D array of shape (n, m) representing the grid points.
        numpy.ndarray: 2D array of shape (n, m) representing the weed densities.


    """
    # Define the grid for estimation
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

    return xx, yy, grid_points, weed_density

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
    
    filter_type = config.get('INITIAL_SETUP', 'filter_type')
    num_particles = config.getint('INITIAL_SETUP', 'num_particles')


    # Get initial positions
    initial_positions = distribute_drones(n, r, plot, grid_resolution)

    # Compute Voronoi diagram within the circular boundaries
    boundary_points = np.array(
        [[r*np.cos(theta), r*np.sin(theta)] for theta in np.linspace(0, 2*np.pi, 100)])

    vor, finite_vertices, finite_regions, voronoi_centers, all_vertices = \
        voronoi.compute_voronoi_with_boundaries(initial_positions, boundary_points, plot)

    # Function generate weed distribution(AOIs) within the circular boundary
    xx, yy, grid_points, weed_density = generate_weed_distribution(
        r, num_gaussians, bandwidth, grid_resolution=grid_resolution, plot=plot)

    # Initialize estimates
    if filter_type == 'Kalman':
        initial_estimates = [(position, r**2 * np.eye(2)) for position in initial_positions]
    elif filter_type == 'Particle':
        if 'num_particles' in kwargs:
            num_particles = kwargs['num_particles']
            initial_estimates = [distribute_drones(num_particles, r) for _ in range(n)]
        else:
            raise ValueError("The number of particles ('num_particles') should be provided for the Particle filter.")
    else:
        raise ValueError("filter_type should be either 'Kalman' or 'Particle'.")

    return vor, finite_vertices, finite_regions, voronoi_centers, xx, yy, grid_points, weed_density, initial_estimates, boundary_points, all_vertices
