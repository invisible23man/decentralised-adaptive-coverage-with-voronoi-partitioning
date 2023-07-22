import os
import pickle
import matplotlib.pyplot as plt
from matplotlib.patches import Circle
from scipy.spatial import Voronoi
import mplcursors
import numpy as np
from shapely.geometry import Polygon
import matplotlib.animation as animation

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


def plot_voronoi(all_vertices, finite_vertices, points):
    """
    Visualize Voronoi diagram of drone positions in 2D plot.

    Parameters:
    vor (scipy.spatial.Voronoi): Voronoi diagram.
    finite_vertices (list): List of vertices for finite Voronoi regions.
    points (numpy array): 2D array of drone positions.

    Returns:
    fig
    """
    fig, ax = plt.subplots()

    for region in finite_vertices:
        ax.fill(*zip(*region), alpha=0.4)

    # Plot the points
    sc = ax.scatter(points[:, 0], points[:, 1], color='black', label='Points')
    # Plot the points with labels
    for i, (x, y) in enumerate(points):
        ax.scatter(x, y, color='black')
        ax.text(x, y, f'Drone {i}', color='red')
    # Plot the vertices
    ax.scatter(all_vertices[:, 0], all_vertices[:, 1],
               color='blue', marker='s', label='Vertices')

    ax.legend()

    # Calculate the maximum range of the points
    max_range = np.max(np.abs(np.append(points, all_vertices)))
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

    return fig, ax


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


def visualize_initial_state(all_vertices, finite_vertices, finite_regions, xx, yy, grid_points, weed_density, voronoi_centers, sensor_readings, r):
    """
    Visualize the initial state of the drone swarm.

    Parameters:
    vor (scipy.spatial.Voronoi): Voronoi diagram.
    finite_vertices (np.ndarray): Array of vertices for finite Voronoi regions.
    finite_regions (list): List of finite Voronoi regions.
    grid_points (np.ndarray): Array of grid points within the circular boundary.
    xx (numpy.ndarray): 2D array of shape (n, m) representing the X coordinates of grid points.
    yy (numpy.ndarray): 2D array of shape (n, m) representing the Y coordinates of grid points.
    weed_density (np.ndarray): Array of weed density at each grid point.
    voronoi_centers (np.ndarray): Array of Voronoi centers.
    sensor_readings (list): List of initial sensor readings for each drone.
    r (float): Radius of the coverage area.

    Returns:
    None
    """
    # Visualize initial positions
    visualize_swarm(voronoi_centers, r)

    # Visualize the Voronoi diagram
    plot_voronoi(all_vertices, finite_vertices, voronoi_centers)

    # Visualize the weed distribution
    plot_weed_distribution(xx, yy, weed_density.reshape(xx.shape))

    # Visualize the sensor readings
    # for i, readings in enumerate(sensor_readings):
    #     plot_gaussian_sensor_model(voronoi_centers[i], readings)
    # plot_gaussian_sensor_model(voronoi_centers, sensor_readings)


def visualize_final_state(all_vertices, finite_vertices, finite_regions, xx, yy, grid_points, weed_density, voronoi_centers_list):
    """
    Visualize the final state after the optimization of Voronoi centers.
    """
    for i, voronoi_centers in enumerate(voronoi_centers_list):
        plt.figure(figsize=(10, 10))
        plt.title(f"Iteration {i+1}")

        # Plot the weed density
        plt.contourf(xx, yy, weed_density, cmap='YlGn')

        # Plot Voronoi tesselation
        plot_voronoi(all_vertices, finite_vertices, finite_regions)

        # Plot Voronoi centers
        plt.scatter(*voronoi_centers.T, color='red')

        # Calculate the maximum range of the points
        max_range = np.max(np.abs(np.append(all_vertices)))
        buffer = 0.0  # Set the buffer value as per your requirement
        limit = np.ceil(max_range + buffer)

        plt.xlim([-limit, limit])
        plt.ylim([-limit, limit])
        plt.show()

def plot_voronoi_and_spirals(all_vertices, finite_vertices, finite_regions, centers, spirals, grid_resolution=0.1):
    """
    Plot the Voronoi diagram along with the spiral paths.

    Parameters:
    vor (scipy.spatial.Voronoi): Voronoi object.
    finite_regions (list): A list of finite regions in the Voronoi diagram.
    finite_vertices (list): A list of finite vertices in the Voronoi diagram.
    centers (np.array): The centers of the Voronoi partitions.
    spirals (list): List of spiral paths for each Voronoi partition.
    """
    fig, ax = plt.subplots()

    # Calculate linewidth based on grid resolution
    linewidth = max(0.2, grid_resolution / 10)

    for region in finite_vertices:
        ax.fill(*zip(*region), alpha=0.4)

    # Plot the Voronoi vertices
    ax.scatter(all_vertices[:, 0], all_vertices[:, 1], color='blue', marker='s', label='Vertices')

    # Plot the spiral paths
    for spiral in spirals:
        ax.plot(spiral[:, 0], spiral[:, 1], 'r-')

    # Plot the Voronoi points
    ax.scatter(centers[:, 0], centers[:, 1], color='black', label='Points')

    ax.legend()

    # Calculate the maximum range of the points
    max_range = np.max(np.abs(np.append(centers, all_vertices)))
    buffer = 0.0  # Set the buffer value as per your requirement
    limit = np.ceil(max_range + buffer)

    # Add grid with specified resolution
    x_ticks = np.arange(-limit, limit, grid_resolution)
    y_ticks = np.arange(-limit, limit, grid_resolution)
    ax.set_xticks(np.arange(-limit, limit + 1, grid_resolution*10), minor=False)
    ax.set_yticks(np.arange(-limit, limit + 1, grid_resolution*10), minor=False)
    ax.set_xticks(x_ticks, minor=True)
    ax.set_yticks(y_ticks, minor=True)
    ax.grid(True, which='minor', color='gray', linestyle='-', linewidth=linewidth)

    ax.set_xlim(-limit, limit)
    ax.set_ylim(-limit, limit)



    plt.show()

def plot_results(config):
    all_centers = []
    for file in os.listdir(config.get('RESULTS', 'save_directory')):
        with open(os.path.join(config.get('RESULTS', 'save_directory'), file), "rb") as f:
            centers = pickle.load(f)
            all_centers.append(centers)

    data = {i: {'x': [], 'y': []} for i in range(len(all_centers))}

    fig, ax = plt.subplots()

    def update(frame):
        for i, centers in enumerate(all_centers):
            x, y = centers[frame]
            data[i]['x'].append(x)
            data[i]['y'].append(y)
            ax.clear()
            for i, d in data.items():
                ax.plot(d['x'], d['y'])

    ani = animation.FuncAnimation(fig, update, frames=len(all_centers[0]), repeat=False)
    plt.show()



    