from scipy.spatial import Voronoi
import numpy as np
from utils import plots
from scipy.spatial import KDTree

def compute_voronoi_with_boundaries(points, boundary_points, plot=False):
    # Add the boundary points to the input points
    points_with_boundary = np.concatenate([points, boundary_points])

    # Compute Voronoi diagram
    vor = Voronoi(points_with_boundary)

    # Filter out infinite regions and regions associated with boundary points
    finite_regions = [region for region in vor.regions if -1 not in region and len(region) > 0 ]
    finite_vertices = [vor.vertices[region] for region in finite_regions]
 
    # Create list of original points corresponding to finite regions
    voronoi_centers = vor.points[vor.point_region[:len(points)]]

    if plot:
        # Plot
        plots.plot_voronoi(vor, finite_vertices, points)

    return vor, finite_vertices, finite_regions, voronoi_centers

def compute_voronoi_neighbours(voronoi_centers, distance_threshold):
    """
    Compute Voronoi neighbours for each Voronoi center based on a distance metric.

    Parameters:
    voronoi_centers (np.array): Coordinates of Voronoi centers.
    distance_threshold (float): Threshold distance for determining neighbours.

    Returns:
    list: A list of lists where each sublist contains the indices of neighbouring Voronoi centers for a given center.
    """
    # Build a KDTree from Voronoi centers
    tree = KDTree(voronoi_centers)

    # Query the tree for neighbours within distance_threshold
    neighbours = tree.query_ball_point(voronoi_centers, distance_threshold)

    return neighbours

def get_voronoi_centroids(vor, finite_regions):
    """
    Returns the centroids of the specified Voronoi regions.

    Parameters:
    vor (scipy.spatial.Voronoi): Voronoi object.
    finite_regions (list): List of finite regions' indices.

    Returns:
    numpy.ndarray: Array of centroids of the Voronoi regions.
    """
    centroids = []

    for region_index in finite_regions:
        # Get the vertices of the region
        region_vertices = vor.vertices[vor.regions[region_index]]
        
        # Calculate and append the centroid
        centroid = np.mean(region_vertices, axis=0)
        centroids.append(centroid)

    return np.array(centroids)