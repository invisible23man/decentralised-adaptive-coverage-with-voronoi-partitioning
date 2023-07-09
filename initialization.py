#%%
from dintegrate import integrate_sensor_values
import numpy as np
from utils import plots, voronoi
from sensor import generate_weed_distribution, sensor_func

def distribute_drones(n, r):
    angles = np.random.uniform(0, 2*np.pi, n)
    radii = r * np.sqrt(np.random.uniform(0, 1, n)) # square root is used to prevent clustering at the center

    # convert to cartesian coordinates
    x = radii * np.cos(angles)
    y = radii * np.sin(angles)

    positions = np.column_stack((x, y))

    return positions

#%%
# Number of drones
n = 20

# Radius of the circular area to be covered
r = 1

# Get initial positions
initial_positions = distribute_drones(n, r)

# Visualize initial positions
plots.visualize_swarm(initial_positions, r)

#%%
# Compute Voronoi diagram within the circular boundaries
boundary_points = np.array([[r*np.cos(theta), r*np.sin(theta)] for theta in np.linspace(0, 2*np.pi, 100)])

vor, finite_vertices, finite_regions, voronoi_centers = voronoi.compute_voronoi_with_boundaries(initial_positions, boundary_points, plot=True)
#%%

# Function generate weed distribution(AOIs) within the circular boundary
grid_points, weed_density = generate_weed_distribution(r)

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
distance_threshold = 0.05
neighbours = compute_voronoi_neighbours(voronoi_centers, distance_threshold)