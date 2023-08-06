import numpy as np
from scipy.spatial import Delaunay
import networkx as nx

def compute_laplacian(points):
    # Compute Delaunay triangulation
    tri = Delaunay(points)

    # Create a graph from the Delaunay triangulation
    G = nx.Graph()
    for simplex in tri.simplices:
        for i in range(3):
            for j in range(i+1, 3):
                # Add an edge between the i-th and j-th points of the simplex
                # The weight of the edge is the Euclidean distance between the points
                G.add_edge(simplex[i], simplex[j], weight=np.linalg.norm(points[simplex[i]] - points[simplex[j]]))

    # Compute Laplacian matrix
    L = nx.laplacian_matrix(G).toarray()

    return L

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

# Define points
points = np.array([[0, 0], [1, 0], [0, 1], [1, 1]])

# Compute Laplacian using compute_laplacian function
L1 = compute_laplacian(points)
print("Laplacian matrix computed by compute_laplacian:")
print(L1)

# Compute Laplacian using compute_graph_laplacian function
L2 = compute_graph_laplacian(points)
print("Laplacian matrix computed by compute_graph_laplacian:")
print(L2)
