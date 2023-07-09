import matplotlib.pyplot as plt
from matplotlib.patches import Circle
import mplcursors
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

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
    ax.add_patch(Circle((0, 0), r, fill=False, color='black', linestyle='--', label='Coverage Area'))
    ax.legend()
    ax.set_title('Initial Positions of Drones')
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.grid(True)

    cursor = mplcursors.cursor(sc, hover=True)
    cursor.connect(
        "add", lambda sel: sel.annotation.set_text(f'Position ({positions[sel.target.index][0]:.2f}, {positions[sel.target.index][1]:.2f})')
    )

    plt.show()


def plot_voronoi(vor, finite_points, points):
    """
    Visualize Voronoi diagram of drone positions in 2D plot.

    Parameters:
    vor (scipy.spatial.Voronoi): Voronoi diagram.
    finite_points (list): List of finite Voronoi regions.
    points (numpy array): 2D array of drone positions.

    Returns:
    None
    """
    fig, ax = plt.subplots()

    for region in finite_points:
        ax.fill(*zip(*region), alpha=0.4)

    # Plot the points
    sc = ax.scatter(points[:, 0], points[:, 1], color='black', label='Points')
    # Plot the vertices
    ax.scatter(vor.vertices[:, 0], vor.vertices[:, 1], color='blue', marker='s', label='Vertices')

    ax.legend()

    ax.set_xlim(-2, 3)
    ax.set_ylim(-2, 3)

    cursor = mplcursors.cursor(sc, hover=True)
    cursor.connect(
        "add", lambda sel: sel.annotation.set_text(f'Position ({points[sel.target.index][0]:.2f}, {points[sel.target.index][1]:.2f})')
    )

    # Add a caption
    caption = f"Bounded Voronoi Diagram of {points.shape[0]} Random Points"
    fig.text(0.5, 0.01, caption, ha='center', fontsize=10)

    plt.show()


def plot_weed_distribution(xx, yy, density_map):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.plot_surface(xx, yy, density_map, cmap='viridis')
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Weed Concentration')
    ax.set_title('Estimated Weed Concentration with Diverse Distribution')
    plt.show()

