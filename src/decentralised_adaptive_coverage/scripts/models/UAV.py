import numpy as np
from scipy.spatial import Voronoi, voronoi_plot_2d
import matplotlib.pyplot as plt
from shapely.geometry import Point, Polygon
from scipy.stats import multivariate_normal
from scipy.spatial import Delaunay

from shapely.geometry import Polygon
from shapely.ops import cascaded_union
import matplotlib.path as mplPath

class Drone:
    def __init__(self, position):
        self.position = position
        self.voronoi_region = None
        self.lawnmower_path = None

    def compute_voronoi(self, drone_positions, field_size, plot=False):
        vor = Voronoi(drone_positions)
        regions, vertices = voronoi_finite_polygons_2d(vor, radius=field_size)

        # Define a square representing the field boundaries
        field_boundary = Polygon([(-field_size/2, -field_size/2), (-field_size/2, field_size/2), 
                                  (field_size/2, field_size/2), (field_size/2, -field_size/2)])

        for region in regions:
            polygon = vertices[region]
            # Clipping polygon
            poly = Polygon(polygon)
            poly = poly.intersection(field_boundary)
            if poly.is_empty:
                continue
            if not self.voronoi_region:
                self.voronoi_region = poly
            else:
                self.voronoi_region = cascaded_union([self.voronoi_region, poly])

        if plot:
            self.plot_voronoi(drone_positions, field_size)

    def plot_voronoi(self, drone_positions, field_size):
        fig, ax = plt.subplots()
        ax.set_xlim([-field_size/2, field_size/2])
        ax.set_ylim([-field_size/2, field_size/2])

        # Draw Voronoi region
        if isinstance(self.voronoi_region, Polygon):
            self.voronoi_region = [self.voronoi_region]
        for poly in self.voronoi_region:
            ax.fill(*zip(*poly.exterior.coords), alpha=0.4)

        # Draw drone positions
        ax.plot(drone_positions[:,0], drone_positions[:,1], 'ko')
        ax.set_aspect('equal')
        plt.show()

    def compute_lawnmower_path(self):
        # Compute a lawnmower path within the Voronoi region
        # For simplicity, we'll just use a grid-based approach
        if isinstance(self.voronoi_region, Polygon):
            self.voronoi_region = [self.voronoi_region]
        minx, miny, maxx, maxy = cascaded_union(self.voronoi_region).bounds
        x = np.arange(minx, maxx, 1)
        y = np.arange(miny, maxy, 1)
        path = []
        for i, xi in enumerate(x):
            for yi in y:
                point = Point(xi, yi)
                for poly in self.voronoi_region:
                    if poly.contains(point):
                        path.append((xi, yi))
        self.lawnmower_path = path

    def plot_lawnmower_path(self):
        # Plot the lawnmower path
        path = np.array(self.lawnmower_path)
        plt.plot(path[:, 0], path[:, 1])
        plt.show()


if __name__ == "__main__":

    import sys
    sys.path.append(r'/home/invisible23man/Robotics/Simulations/decentralised-adaptive-coverage-with-voronoi-partitioning/src/decentralised_adaptive_coverage/scripts/')
    from models.Environment import Field

    # Example usage
    size = 50
    grid_resolution = 1
    drone_count = 16
    weed_centers = [[-15, -15], [15, 15]]
    weed_cov = [[5, 0], [0, 5]]

    field = Field(size, grid_resolution, drone_count, weed_centers, weed_cov)

    drones = [Drone(pos) for pos in field.drone_positions]

    for drone in drones[:2]:
        drone.compute_voronoi(field.drone_positions[:, :2], size)
