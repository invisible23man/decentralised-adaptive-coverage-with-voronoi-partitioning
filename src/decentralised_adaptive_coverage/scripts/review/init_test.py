#%%
from decentralised_adaptive_coverage.scripts.review.initialization import initial_setup
from decentralised_adaptive_coverage.scripts.review.sensor import sensor_func
from dintegrate import integrate_sensor_values
from utils import voronoi
import numpy as np

# Number of drones
n = 20

# Radius of the circular area to be covered
r = 1

vor, finite_vertices, finite_regions, voronoi_centers, grid_points, weed_density = initial_setup(n, r)
#%%
# Test Sensor Function Output
# Get weed concentration at a specific point
x_coord = 0.1673
y_coord = 0.8680
print(sensor_func((x_coord, y_coord), grid_points, weed_density))
# %%

def test_integrate_sensor_values(vor, finite_vertices, finite_regions, voronoi_centers, time_step=0.05, time_limit=5):
    """
    Test the function integrate_sensor_values.
    """
    # Define a simple sensor function that ignores time and returns the sum of coordinates
    # def sensor_func(t, point):
    #     return sum(point)

    # Select a random region for testing
    region_index = np.random.randint(len(finite_regions))
    region = finite_regions[region_index]
    partition = finite_vertices[region_index]

    # Calculate Voronoi center
    voronoi_center = voronoi_centers[region_index]

    # Call the function
    mv, lv, cv = integrate_sensor_values(sensor_func, partition, voronoi_center, grid_points, weed_density, time_step, time_limit)
    
    print(f"mv: {mv}, lv: {lv}, cv: {cv}")

#%%
test_integrate_sensor_values(vor, finite_vertices, finite_regions, voronoi_centers, time_step=0.05, time_limit=5)

# %%
distance_threshold = 0.25
neighbours = voronoi.compute_voronoi_neighbours(voronoi_centers, distance_threshold)
# %%
def test_integrate_sensor_values_with_neighbours(vor, finite_vertices, finite_regions, voronoi_centers, 
                                                    time_step=0.05, time_limit=5, distance_threshold=0.5):
    """
    Test the function integrate_sensor_values by performing integration for each Voronoi center and its neighbours.
    
    Parameters:
    vor (scipy.spatial.qhull.Voronoi): Voronoi diagram.
    finite_vertices (list): List of vertices of finite Voronoi regions.
    finite_regions (list): List of finite Voronoi regions.
    voronoi_centers (np.array): Coordinates of Voronoi centers.
    time_step (float): Time step for integration. Default is 0.05.
    time_limit (float): Time limit for integration. Default is 5.
    distance_threshold (float): Threshold distance for determining neighbours. Default is 0.5.

    Returns:
    None
    """

    # Compute Voronoi neighbours for each Voronoi center
    neighbours = voronoi.compute_voronoi_neighbours(voronoi_centers, distance_threshold)

    # For each Voronoi center
    for i, center in enumerate(voronoi_centers):
        print(f"\nProcessing Voronoi center {i}...")

        # Get the corresponding partition
        partition = finite_vertices[i]

        # Integrate sensor values for the center
        mv, lv, cv = integrate_sensor_values(sensor_func, partition, center, grid_points, weed_density, time_step, time_limit)
        print(f"For the center itself, mv: {mv}, lv: {lv}, cv: {cv}")

        # For each neighbour of the current center
        for neighbour_index in neighbours[i]:
            # Get the corresponding partition
            partition = finite_vertices[neighbour_index]

            # Integrate sensor values for the neighbour
            mv, lv, cv = integrate_sensor_values(sensor_func, partition, voronoi_centers[neighbour_index], grid_points, weed_density, time_step, time_limit)
            print(f"For neighbour {neighbour_index}, mv: {mv}, lv: {lv}, cv: {cv}")

            # Locational Optmization - Applying Control Law
            

#%%
test_integrate_sensor_values_with_neighbours(vor, finite_vertices, finite_regions, voronoi_centers, 
                                                    time_step=0.05, time_limit=5, distance_threshold=0.5)
# %%
