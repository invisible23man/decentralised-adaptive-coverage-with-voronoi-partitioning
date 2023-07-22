from scipy.spatial import Voronoi, Delaunay
import numpy as np
from utils import plots
from scipy.spatial import KDTree


def compute_voronoi_with_boundaries(points, boundary_points, plot=False):
    """
    Compute a Voronoi diagram with specified boundary points and filter out infinite regions and 
    regions associated with boundary points.

    Parameters:
    points (np.array): An array of points in the plane. Each point is an array of two coordinates.
    boundary_points (np.array): An array of points that form the boundary of the area of interest.
    plot (bool, optional): If True, plot the Voronoi diagram. Defaults to False.

    Returns:
    scipy.spatial.Voronoi: The computed Voronoi diagram.
    list of np.array: A list of vertices for each finite Voronoi region.
    list of list of int: A list of finite Voronoi regions. Each region is represented as a list of indices into the Voronoi vertices array.
    np.array: An array of Voronoi centers corresponding to the finite regions.
    """

    # Add the boundary points to the input points
    points_with_boundary = np.concatenate([points, boundary_points])

    # Compute Voronoi diagram
    vor = Voronoi(points_with_boundary)

    # Filter out infinite regions and regions associated with boundary points
    finite_regions = [region for region in vor.regions if -
                      1 not in region and len(region) > 0]
    finite_vertices = [vor.vertices[region] for region in finite_regions]

    # Create list of original points corresponding to finite regions
    voronoi_centers = np.array([points_with_boundary[i] for i in vor.point_region[:len(points)] if vor.regions[i] in finite_regions])
    all_vertices = vor.vertices

    if plot:
        plots.plot_voronoi(vor, finite_vertices, points)

    return vor, finite_vertices, finite_regions, voronoi_centers, all_vertices


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

def compute_graph_laplacian(points):
    """
    Computes the graph Laplacian from Delaunay triangulation.

    Args:
        points (numpy.ndarray): Array of shape (n, 2) containing the coordinates of the points.

    Returns:
        numpy.ndarray: The graph Laplacian matrix.

    """
    # Perform Delaunay triangulation
    tri = Delaunay(points)

    # Get the edges of the triangulation
    edges = tri.convex_hull

    # Initialize Laplacian matrix
    n = len(points)
    L = np.zeros((n, n))

    # Compute shared edge lengths as weights
    for r1, r2 in edges:
        points_indices = np.intersect1d(tri.neighbors[r1], tri.neighbors[r2])
        if len(points_indices) == 2:
            edge_len = np.linalg.norm(points[points_indices[1]] - points[points_indices[0]])
        else:
            edge_len = 0
        L[r1, r2] = -edge_len
        L[r2, r1] = -edge_len

    # Compute diagonal elements
    L = L + np.diag(-np.sum(L, axis=1))

    return L
