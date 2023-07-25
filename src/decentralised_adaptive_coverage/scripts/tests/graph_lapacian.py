import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from scipy.spatial import Delaunay

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

# Generate random points
n = 50
points = np.random.rand(n, 2)

# Compute graph Laplacian
L = compute_graph_laplacian(points)
print(L)

# Create figure and axes for animation
fig, ax = plt.subplots()
line, = ax.plot([], [], 'r-')

def update(frame):
    # Compute eigenvalues and eigenvectors
    eigenvalues, eigenvectors = np.linalg.eig(L)
    sorted_indices = np.argsort(eigenvalues)
    x = np.arange(n)
    y = eigenvectors[:, sorted_indices[frame]]

    # Update the plot
    line.set_data(x, y)
    ax.set_xlim(0, n-1)
    ax.set_ylim(np.min(y), np.max(y))
    ax.set_xlabel('Node')
    ax.set_ylabel('Eigenvector')
    ax.set_title(f'Graph Laplacian Eigenvector {frame+1}/{n}')

    return line,

# Animate the plot
animation = FuncAnimation(fig, update, frames=n, interval=200, blit=True)

# Display the animation
plt.show()
