# %%
import numpy as np
from utils import plots, voronoi
from sensor import generate_weed_distribution


def distribute_drones(n, r):
    """
    Distributes 'n' drones evenly within a circular area of radius 'r'.

    Args:
        n (int): Number of drones.
        r (float): Radius of the circular area.

    Returns:
        numpy.ndarray: Array of shape (n, 2) containing the Cartesian coordinates of the drones.

    """
    angles = np.random.uniform(0, 2*np.pi, n)
    # square root is used to prevent clustering at the center
    radii = r * np.sqrt(np.random.uniform(0, 1, n))

    # convert to cartesian coordinates
    x = radii * np.cos(angles)
    y = radii * np.sin(angles)

    positions = np.column_stack((x, y))

    return positions


def initial_setup(n=20, r=1, filter_type='Kalman', **kwargs):
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
            - grid_points (numpy.ndarray): Array of shape (k, 2) containing the grid points within the circular boundary.
            - weed_density (numpy.ndarray): Array of shape (k,) containing the weed density at each grid point.
            - initial_estimates (list): List of initial estimates. Each element is either a tuple of mean and covariance (for Kalman filter), or an array of particles (for particle filter).

    """
    # Get initial positions
    initial_positions = distribute_drones(n, r)

    # Visualize initial positions
    plots.visualize_swarm(initial_positions, r)

    # Compute Voronoi diagram within the circular boundaries
    boundary_points = np.array(
        [[r*np.cos(theta), r*np.sin(theta)] for theta in np.linspace(0, 2*np.pi, 100)])

    vor, finite_vertices, finite_regions, voronoi_centers = voronoi.compute_voronoi_with_boundaries(
        initial_positions, boundary_points, plot=True)

    # Function generate weed distribution(AOIs) within the circular boundary
    grid_points, weed_density = generate_weed_distribution(r, plot=True)

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

    return vor, finite_vertices, finite_regions, voronoi_centers, grid_points, weed_density, initial_estimates
