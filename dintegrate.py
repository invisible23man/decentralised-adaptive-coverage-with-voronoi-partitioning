from scipy.integrate import quad
import numpy as np

def boustrophedon_points(partition):
    """
    Generate points in a Voronoi partition in a boustrophedon pattern.
    
    Parameters:
    partition (np.array): Points in the Voronoi partition.

    Returns:
    np.array: Points in a boustrophedon pattern.
    """
    # Sort points by their y-coordinate, and then by their x-coordinate
    points_sorted = partition[np.lexsort((partition[:, 0], partition[:, 1]))]

    # Split points into rows
    y_values = np.unique(points_sorted[:, 1])
    rows = [points_sorted[points_sorted[:, 1] == y] for y in y_values]

    # Reverse the order of points in every second row to get a boustrophedon pattern
    for i in range(1, len(rows), 2):
        rows[i] = rows[i][::-1]

    # Concatenate rows to get a single array of points
    points_boustrophedon = np.concatenate(rows)

    return points_boustrophedon


def integrate_sensor_values(sensor_func, partition, voronoi_center, grid_points, weed_density, time_step, time_limit):
    """
    Integrate the sensor values in the given Voronoi partition until the specified time.
    
    Parameters:
    sensor_func (function): Sensor function.
    partition (np.array): List of points in Voronoi partition.
    voronoi_center (np.array): Voronoi center coordinates.
    time_step (float): Time step for integration.
    time_limit (float): Time limit for integration.

    Returns:
    float, float: mv and lv values.
    """
    # Generate points in a boustrophedon pattern
    points = boustrophedon_points(partition)

    # Calculate the number of points that can be visited within the time limit
    num_points = int(time_limit / time_step)

    # Trim the list of points if necessary
    if num_points < len(points):
        points = points[:num_points]

    # Compute distances from each point to the Voronoi center
    distances = np.linalg.norm(points - voronoi_center, axis=1)

    # Generate array of sensor values at each point
    sensor_values = np.array([sensor_func(point, grid_points, weed_density) for point in points])

    # Integrate sensor values for mv
    mv = np.sum(sensor_values)

    # Calculate lv as distance weighted integral of the sensor values
    lv = np.sum((distances*sensor_values.T))
    
    # Calculate cv
    cv = lv / mv if mv != 0 else np.inf

    return mv, lv, cv
