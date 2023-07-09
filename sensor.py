import numpy as np
from sklearn.neighbors import KernelDensity
from utils import plots
from scipy.interpolate import griddata

# Function to generate KDE centers within the circular boundary
def generate_kde_centers(num_centers, radius):
    centers = []
    for _ in range(num_centers):
        # Generate random polar coordinates and convert to Cartesian
        angle = np.random.uniform(0, 2*np.pi)
        rad = radius * np.sqrt(np.random.uniform(0, 1))  # Prevents clustering at center
        x = rad * np.cos(angle)
        y = rad * np.sin(angle)
        centers.append([x, y])
    return np.array(centers)

# Function generate weed distribution(AOIs) within the circular boundary
def generate_weed_distribution(r, num_gaussians = 3, bandwidth = 0.085 ):
    """
    bandwidth:  Adjust this value to control the spread/variance

    """
    # Define the grid for estimation
    grid_resolution = 0.01
    x_min, x_max = -r, r
    y_min, y_max = -r, r
    xx, yy = np.meshgrid(np.arange(x_min, x_max, grid_resolution),
                        np.arange(y_min, y_max, grid_resolution))
    grid_points = np.c_[xx.ravel(), yy.ravel()]

    # Generate sample weed concentration data
    weed_concentration = np.concatenate([np.random.normal(loc=center, scale=[0.1, 0.05], size=(100, 2)) 
                                        for center in generate_kde_centers(num_gaussians, r)])

    # Perform kernel density estimation
    kde = KernelDensity(bandwidth=bandwidth, kernel='gaussian')
    kde.fit(weed_concentration)
    weed_density = np.exp(kde.score_samples(grid_points))

    # Reshape the density values to match the grid shape
    density_map = weed_density.reshape(xx.shape)

    # Visualization (example using Matplotlib)
    plots.plot_weed_distribution(xx, yy, density_map)

    return grid_points, weed_density

# Function to return weed concentration given coordinates
def sensor_func(point, grid_points, weed_density):
    return griddata(grid_points, weed_density, point, method='linear')
