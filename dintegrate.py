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
    sensor_values = np.array(
        [sensor_func(point, grid_points, weed_density) for point in points])

    # Integrate sensor values for mv
    mv = np.sum(sensor_values)

    # Calculate lv as distance weighted integral of the sensor values
    lv = np.sum((distances*sensor_values.T))

    # Calculate cv
    cv = lv / mv if mv != 0 else np.inf

    return mv, lv, cv


def optimize_voronoi_centers_consensus(voronoi_centers, finite_vertices, finite_regions,
                                       grid_points, weed_density, sensor_func, neighbours,
                                       time_step=0.05, time_limit=5, alpha=0.1, beta=0.1,
                                       epsilon=1e-6, max_iter=100):
    """
    Optimize Voronoi centers using gradient descent and consensus algorithm.

    Parameters:
    voronoi_centers (np.array): Coordinates of Voronoi centers.
    finite_vertices (list): List of vertices of finite Voronoi regions.
    finite_regions (list): List of finite Voronoi regions.
    sensor_func (function): Sensor function.
    neighbours (list): List of lists where each sublist contains the indices of neighbouring Voronoi centers for a given center.
    time_step (float): Time step for integration. Default is 0.05.
    time_limit (float): Time limit for integration. Default is 5.
    alpha (float): Learning rate for gradient descent. Default is 0.1.
    beta (float): Weight for the consensus term. Default is 0.1.
    epsilon (float): Threshold for determining convergence. Default is 1e-6.
    max_iter (int): Maximum number of iterations. Default is 100.

    Returns:
    np.array: Updated coordinates of Voronoi centers.
    """
    for iter in range(max_iter):
        # Initialize change to zero
        change = 0

        # For each Voronoi center
        for i, center in enumerate(voronoi_centers):
            # Get the corresponding partition
            partition = finite_vertices[i]

            # Calculate the moment integrals
            mv, lv, cv = integrate_sensor_values(sensor_func, partition, center,
                                                 grid_points, weed_density,
                                                 time_step, time_limit)

            # Calculate the consensus term
            consensus_term = sum(
                voronoi_centers[j] - center for j in neighbours[i])

            # Calculate the gradient (negative of the derivative of the cost function, with consensus term)
            gradient = -mv * (cv - center) + beta * consensus_term

            # Update the Voronoi center
            new_center = center - alpha * gradient

            # Calculate the change in position
            change = max(change, np.linalg.norm(new_center - center))

            # Update the Voronoi center
            voronoi_centers[i] = new_center

        # Check for convergence
        if change < epsilon:
            print("Converged after", iter, "iterations.")
            break

    if iter == max_iter - 1:
        print("Reached maximum number of iterations without convergence.")

    return voronoi_centers
