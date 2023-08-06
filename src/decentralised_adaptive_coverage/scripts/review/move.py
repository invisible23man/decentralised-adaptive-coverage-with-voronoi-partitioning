from typing import Optional
from shapely.geometry import Point, Polygon
import numpy as np
from iq_gnc.py_gnc_functions_swarm import rospy, gnc_api, Odometry
from decentralised_adaptive_coverage.scripts.review.sensor import sample_weed_density, sense as sense
from utils.transformations import apply_transformation_matrix

def move_to_point(x, y, grid_resolution, dx, dy, voronoi_region, boundary_tolerance):
    # Move the drone in the specified direction (dx, dy) from the current point (x, y) within the Voronoi region.
    x_new, y_new = x + dx * grid_resolution, y + dy * grid_resolution
    if voronoi_region.contains(Point(x_new, y_new)):
        return x_new, y_new
    else:
        if dx != 0:
            x_truncated = np.round(x / grid_resolution + boundary_tolerance) * grid_resolution
            return x_truncated, y
        else:
            y_truncated = np.round(y / grid_resolution + boundary_tolerance) * grid_resolution
            return x, y_truncated


def generate_rectangular_spiral_trajectory(center, vertices, grid_resolution, grid_points, weed_density,
    sampling_time, time_per_step, boundary_tolerance=0.02, gnc_drone: Optional[gnc_api]= None):
    """
    Generate a rectangular spiral path starting from the center within the Voronoi region and perform the sensing process.

    Args:
        center (tuple): The (x, y) coordinates of the center.
        grid_resolution (float): The distance between adjacent points in the grid.
        sampling_time (float): Total available time for sampling.
        time_per_step (float): Time taken for each step.
        vertices (np.array): Vertices of the Voronoi region.
        tolerance (float): Tolerance value to truncate coordinates outside the polygon (default is 0.02).
        weed_density (np.array): 2D array representing the weed densities (optional).

    Returns:
        np.array: Points on the rectangular spiral path within the Voronoi region.
        np.array: Sensor values (weed concentrations) at each point on the path.

    """
    x, y = center
    stride = 1  # Initial stride length
    strides_since_stride_increment = 0  # Counter for the number of strides taken since the last stride increment
    spiral_path = []
    sensor_values = []
    current_time = 0

    voronoi_region = Polygon(vertices)  # Create the Voronoi region polygon

    while current_time < sampling_time:
        spiral_path.append([x, y])
        if gnc_drone:
            navigate_to_destination(gnc_drone, x, y)

        sensor_measurements = sample_weed_density(sense, spiral_path[-1], grid_points, 
                                                  weed_density, sensor_noise_std_dev=0.1, noise_model='gaussian')
        sensor_values.append(sensor_measurements)

        # List of direction vectors (right, up, left, down)
        directions = [(grid_resolution, 0), (0, grid_resolution), (-grid_resolution, 0), (0, -grid_resolution)]

        for _ in range(4):
            dx, dy = directions.pop(0)
            for _ in range(stride):
                x_new, y_new = move_to_point(x, y, grid_resolution, dx, dy, voronoi_region, boundary_tolerance)
                x, y = x_new, y_new
                spiral_path.append([x, y])

                if gnc_drone:
                    navigate_to_destination(gnc_drone, x, y)

                sensor_measurements = sample_weed_density(sense, spiral_path[-1], grid_points, 
                                        weed_density, sensor_noise_std_dev=0.1, noise_model='gaussian')
                sensor_values.append(sensor_measurements)

                current_time += time_per_step
                if current_time >= sampling_time:
                    break

            strides_since_stride_increment += 1
            if strides_since_stride_increment >= 2:  # Increment stride length every 2 strides
                stride += 1
                strides_since_stride_increment = 0


            if current_time >= sampling_time:
                break

    return np.array(spiral_path), np.array(sensor_values)

def generate_lawnmower_trajectory(center, vertices, grid_resolution, grid_points, weed_density,
                                  sampling_time, time_per_step, boundary_tolerance=0.2,
                                  gnc_drone: Optional[gnc_api]= None):
    """
    Generate a lawnmower pattern for coverage within the Voronoi region.

    Args:
        center (tuple): The (x, y) coordinates of the center.
        grid_resolution (float): The distance between adjacent points in the grid.
        vertices (np.array): Vertices of the Voronoi region.

    Returns:
        np.array: Points on the lawnmower path within the Voronoi region.
    """
    x_min, y_min = np.min(vertices, axis=0)
    x_max, y_max =np.max(vertices, axis=0)

    # x_range = np.arange(int(x_min), int(x_max), grid_resolution)
    # y_range = np.arange(int(y_min), int(y_max), grid_resolution)

    x_range = np.arange(x_min, x_max, grid_resolution)
    y_range = np.arange(y_min, y_max, grid_resolution)

    lawnmower_path = []
    sensor_values = []

    current_time = 0

    for i, x in enumerate(x_range):
        if i % 2 == 0:  # Move up
            for y in y_range:
                point = np.array([x, y])
                if Polygon(vertices).contains(Point(point)):
                    lawnmower_path.append(point)
                    sensor_measurements = sample_weed_density(sense, lawnmower_path[-1], grid_points, 
                                            weed_density, sensor_noise_std_dev=0.1, noise_model='gaussian')
                    sensor_values.append(sensor_measurements)

                    if gnc_drone:
                        navigate_to_destination(gnc_drone, x, y)
                    current_time += time_per_step
                    if current_time >= sampling_time:
                                        break

        else:  # Move down
            for y in reversed(y_range):
                point = np.array([x, y])
                if Polygon(vertices).contains(Point(point)):
                    lawnmower_path.append(point)
                    sensor_measurements = sample_weed_density(sense, lawnmower_path[-1], grid_points, 
                                            weed_density, sensor_noise_std_dev=0.1, noise_model='gaussian')
                    sensor_values.append(sensor_measurements)
                    if gnc_drone:
                        navigate_to_destination(gnc_drone, x, y)
                    current_time += time_per_step
                    if current_time >= sampling_time:
                                        break
        if current_time >= sampling_time:
                    break

    return np.array(lawnmower_path), np.array(sensor_values)

def voronoi_coverage(all_vertices, finite_regions, centers, 
    grid_resolution, grid_points, weed_density,
    sampling_time, time_per_step, method = 'spiral'):
    """
    Compute rectangular spiral paths and weed concentartions for each 
    Voronoi partition and return them.

    Parameters:
    vor (scipy.spatial.Voronoi): Voronoi object.
    finite_regions (list): A list of finite regions in the Voronoi diagram.
    centers (np.array): The centers of the Voronoi partitions.
    grid_resolution (float): Resolution of the grid.
    sampling_time (float): Total time for sampling.
    time_per_step (float): Time taken for each step.

    Returns:
    list: List of rectangular spiral paths for each Voronoi partition.
    list: List of sensor values (weed concentrations) for each spiral path.
    """
    paths = []
    sensor_values = []
    for region, center in zip(finite_regions, centers):
        if method == 'spiral':
            path, sensor_values_from_path = generate_rectangular_spiral_trajectory(
                center, [all_vertices[i] for i in region], 
                grid_resolution, grid_points, weed_density,
                sampling_time, time_per_step, boundary_tolerance=0.2)
        if method == 'lawn_mover':
            path, sensor_values_from_path = generate_lawnmower_trajectory(
                center, [all_vertices[i] for i in region], 
                grid_resolution, grid_points, weed_density,
                sampling_time, time_per_step, boundary_tolerance=0.2)
        paths.append(path)
        sensor_values.append(sensor_values_from_path)

    return paths, sensor_values



def navigate_to_destination(gnc_drone, x, y, z=3):
    """
    Navigate the drone to a specific destination in the Gazebo world.

    Parameters:
        gnc_drone (YourDroneController): An instance of your drone controller class.
        x (float): Gazebo x-coordinate (East) of the destination point in meters.
        y (float): Gazebo y-coordinate (North) of the destination point in meters.
        z (float): Gazebo z-coordinate (Up) of the destination point in meters.

    Returns:
        None

    Description:
        This function sets the drone's target destination using the provided drone controller (gnc_drone).
        The drone will navigate to the specified destination point in the Gazebo world.

    Note:
        - Make sure you have already initialized the ROS node and created the drone object (gnc_drone)
          before calling this function.
        - The function assumes that the drone's orientation (quaternion) is set to (0, 0, 0, 1)
          for simplicity (no rotation).
    """
    # Apply the transformation matrix to convert Gazebo coordinates to MAVROS coordinates
    enu_x, enu_y, enu_z = apply_transformation_matrix(x, y, z, gnc_drone.transformation_matrix)

    # Create the Odometry message with ENU coordinates
    destination = Odometry()

    # Set the position (ENU coordinates)
    destination.pose.pose.position.x = enu_x
    destination.pose.pose.position.y = enu_y
    destination.pose.pose.position.z = enu_z

    # Set the orientation (quaternion - assuming you have orientation information)
    destination.pose.pose.orientation.x = 0.0
    destination.pose.pose.orientation.y = 0.0
    destination.pose.pose.orientation.z = 0.0
    destination.pose.pose.orientation.w = 1.0  # Assuming no rotation for simplicity

    destination_pose_local = gnc_drone.enu_2_local(destination, gnc_drone.local_offset_g)
    gnc_drone.set_destination(
        destination_pose_local.x,
        destination_pose_local.y,
        destination_pose_local.z,
        psi=0,
    )

    rate = rospy.Rate(3)
    rate.sleep()
    # while not gnc_drone.check_waypoint_reached():
    #     pass
