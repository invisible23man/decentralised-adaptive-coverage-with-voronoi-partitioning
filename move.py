from shapely.geometry import Point, Polygon
from shapely.ops import unary_union
import numpy as np

def boustrophedon_points(center, vertices, grid_resolution, max_time, start_time=0):
    """
    Generate points in a Voronoi partition in a boustrophedon pattern.

    Parameters:
    center (np.array): The center of the Voronoi partition.
    vertices (np.array): The vertices of the Voronoi partition.
    grid_resolution (float): The distance between adjacent points in the grid.
    max_time (float): The maximum time for which to generate points.
    start_time (float): The time at which to start generating points (default is 0).

    Returns:
    np.array: Points in a boustrophedon pattern.
    """
    # Define the Voronoi cell as a Shapely Polygon
    voronoi_cell = Polygon(vertices)

    # Generate a grid of points within the bounding box of the Voronoi cell
    minx, miny, maxx, maxy = voronoi_cell.bounds
    x_coords = np.arange(minx, maxx, grid_resolution)
    y_coords = np.arange(miny, maxy, grid_resolution)
    grid_points = np.array([(x, y) for x in x_coords for y in y_coords])

    # Filter out points that are not inside the Voronoi cell
    grid_points = [point for point in grid_points if voronoi_cell.contains(Point(point))]

    # Sort points by their y-coordinate, and then by their x-coordinate
    points_sorted = np.array(sorted(grid_points, key=lambda p: (p[1], p[0])))

    # Split points into rows
    y_values = np.unique(points_sorted[:, 1])
    rows = [points_sorted[points_sorted[:, 1] == y] for y in y_values]

    # Reverse the order of points in every second row to get a boustrophedon pattern
    for i in range(1, len(rows), 2):
        rows[i] = rows[i][::-1]

    # Concatenate rows to get a single array of points
    points_boustrophedon = np.concatenate(rows)

    # Find the closest point to the Voronoi center and rotate the array so it starts there
    start_index = np.argmin(np.sum((points_boustrophedon - center) ** 2, axis=1))
    points_boustrophedon = np.roll(points_boustrophedon, -start_index, axis=0)

    # Simulate the passage of time and only return points up to max_time
    current_time = start_time
    points_in_time = []
    for point in points_boustrophedon:
        if current_time > max_time:
            break
        points_in_time.append(point)
        current_time += grid_resolution  # Increase time based on the distance to the next point

    return np.array(points_in_time)

def generate_spiral_path(center, vertices, resolution, sampling_time, time_per_step, spiral_resolution=0.01):
    """
    Generate a spiral path from the center of a Voronoi partition within a given sampling time.

    Parameters:
    center (tuple): The (x, y) coordinates of the center of the partition.
    vertices (np.array): The vertices of the Voronoi partition.
    resolution (float): The distance between successive points on the spiral.
    sampling_time (float): Total available time for sampling.
    time_per_step (float): Time taken for each step.

    Returns:
    np.array: Points on the spiral path.
    """
    # Compute maximum radius as half the length of the longest side of the bounding box
    x_max, y_max = np.max(vertices, axis=0)
    x_min, y_min = np.min(vertices, axis=0)
    max_radius = max(x_max - x_min, y_max - y_min) / 20

    theta = 0  # Angle from the x-axis
    points = []
    max_steps = int(sampling_time / time_per_step)  # Compute maximum steps from sampling time

    for _ in range(max_steps):
        x = center[0] + max_radius * np.cos(theta)
        y = center[1] + max_radius * np.sin(theta)
        points.append([x, y])
        theta += spiral_resolution / max_radius  # Increase the angle
        max_radius -= spiral_resolution / (2 * np.pi)  # Decrease the radius

    return np.array(points)

def voronoi_coverage_with_spirals(vor, finite_regions, centers, sampling_time, time_per_step, grid_resolution):
    """
    Compute spiral paths for each Voronoi partition and return them.

    Parameters:
    vor (scipy.spatial.Voronoi): Voronoi object.
    finite_regions (list): A list of finite regions in the Voronoi diagram.
    centers (np.array): The centers of the Voronoi partitions.
    sampling_time (float): Total time for sampling.
    time_per_step (float): Time taken for each step.
    grid_resolution (float): Resolution of the grid.

    Returns:
    list: List of spiral paths for each Voronoi partition.
    """
    spirals = []
    for region, center in zip(finite_regions, centers):
        spiral_path = generate_spiral_path(center, np.array([vor.vertices[i] for i in region]), grid_resolution, sampling_time, time_per_step)
        spirals.append(spiral_path)

    return spirals

