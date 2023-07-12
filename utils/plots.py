import matplotlib.pyplot as plt
from matplotlib.patches import Circle
from scipy.spatial import Voronoi
import mplcursors
import numpy as np

def visualize_swarm(positions, r):
    """
    Visualize the positions of drones in 2D plot.

    Parameters:
    positions (numpy array): 2D array of drone positions.
    r (float): Radius of the coverage area.

    Returns:
    None
    """
    fig, ax = plt.subplots()
    sc = ax.scatter(positions[:, 0], positions[:, 1], label='Drones')
    ax.add_patch(Circle((0, 0), r, fill=False, color='black',
                 linestyle='--', label='Coverage Area'))
    ax.legend()
    ax.set_title('Initial Positions of Drones')
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.grid(True)

    cursor = mplcursors.cursor(sc, hover=True)
    cursor.connect(
        "add", lambda sel: sel.annotation.set_text(
            f'Position ({positions[sel.target.index][0]:.2f}, {positions[sel.target.index][1]:.2f})')
    )

    plt.show()


def plot_voronoi(vor: Voronoi, finite_vertices, points):
    """
    Visualize Voronoi diagram of drone positions in 2D plot.

    Parameters:
    vor (scipy.spatial.Voronoi): Voronoi diagram.
    finite_vertices (list): List of vertices for finite Voronoi regions.
    points (numpy array): 2D array of drone positions.

    Returns:
    None
    """
    fig, ax = plt.subplots()

    for region in finite_vertices:
        ax.fill(*zip(*region), alpha=0.4)

    # Plot the points
    sc = ax.scatter(points[:, 0], points[:, 1], color='black', label='Points')
    # Plot the vertices
    ax.scatter(vor.vertices[:, 0], vor.vertices[:, 1],
               color='blue', marker='s', label='Vertices')

    ax.legend()

    # Calculate the maximum range of the points
    max_range = np.max(np.abs(np.append(points,vor.vertices)))
    buffer = 0.0  # Set the buffer value as per your requirement
    limit = np.ceil(max_range + buffer)

    ax.set_xlim(-limit, limit)
    ax.set_ylim(-limit, limit)

    cursor = mplcursors.cursor(sc, hover=True)
    cursor.connect(
        "add", lambda sel: sel.annotation.set_text(
            f'Position ({points[sel.target.index][0]:.2f}, {points[sel.target.index][1]:.2f})')
    )

    # Add a caption
    caption = f"Bounded Voronoi Diagram of {points.shape[0]} Random Points"
    fig.text(0.5, 0.01, caption, ha='center', fontsize=10)

    plt.show()


def plot_weed_distribution(xx, yy, density_map):
    """
    Plot the estimated weed concentration distribution in a 3D surface plot.

    Parameters:
    xx (numpy array): Meshgrid along the X-axis.
    yy (numpy array): Meshgrid along the Y-axis.
    density_map (numpy array): 2D array of weed concentration values.

    Returns:
    None
    """
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.plot_surface(xx, yy, density_map, cmap='viridis')
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Weed Concentration')
    ax.set_title('Estimated Weed Concentration with Diverse Distribution')
    plt.show()

def plot_gaussian_sensor_model(points, sensor_values):
    """
    Plot the Gaussian sensor model.

    Parameters:
    points (np.array): An array of points where each point is an array [x, y].
    sensor_values (np.array): An array of sensor values.
    """

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.scatter(points[:, 0], points[:, 1], sensor_values)
    plt.show()