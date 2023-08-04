from shapely.ops import unary_union
import numpy as np
import matplotlib.pyplot as plt
from shapely.geometry import Polygon, Point
from scipy.spatial import distance_matrix
import networkx as nx

class Planner:
    def __init__(self, drone):
        self.drone = drone
        
    def compute_lawnmower_path(self):
        # Compute a lawnmower path within the Voronoi region
        # For simplicity, we'll just use a grid-based approach
        if isinstance(self.drone.voronoi_region, Polygon):
            self.drone.voronoi_region = [self.drone.voronoi_region]
        minx, miny, maxx, maxy = self.drone.voronoi_region[self.drone.id].bounds
        x = np.arange(minx, maxx, self.drone.grid_resolution)
        y = np.arange(miny, maxy, self.drone.grid_resolution)
        path = []
        for i, xi in enumerate(x):
            if i % 2 == 0:  # if i is even
                y_order = y  # iterate through y in ascending order
            else:
                y_order = y[::-1]  # iterate through y in descending order
            for yi in y_order:
                point = Point(xi, yi)
                for poly in self.drone.voronoi_region:
                    if poly.contains(point):
                        path.append((xi, yi))
        self.drone.lawnmower_path = np.array(path)
        if self.drone.planner_config["reordermode"]:
            self.drone.lawnmower_path = self.reorder_path(self.drone.position, self.drone.lawnmower_path, mode=self.drone.planner_config["reordermode"])

    def reorder_path(self, start_point, path, mode="NearestNeighbor"):
        if mode == "NearestNeighbor":
            return self.reorder_nearest_neighbor(start_point, path)
        elif mode == "SpiralOutward":
            return self.reorder_path_spiral_outward(start_point, path)
        else:
            raise ValueError("Invalid mode. Choose 'NearestNeighbor' or 'SpiralOutward'.")

    def reorder_nearest_neighbor(self, start_point, path):
        # Nearest neighbor approach as before

        # Create a copy of the path
        remaining_points = path.copy()

        # Initialize the reordered path with the start point
        reordered_path = np.array([[start_point[0],start_point[1]]])

        # While there are still points to visit
        while remaining_points.size > 0:
            # Compute the distance from the current point to all remaining points
            distances = distance_matrix([reordered_path[-1]], remaining_points)

            # Find the closest point
            closest_point_index = np.argmin(distances)

            # Add the closest point to the reordered path
            reordered_path = np.append(reordered_path, [remaining_points[closest_point_index]], axis=0)

            # Remove the closest point from the list of remaining points
            remaining_points = np.delete(remaining_points, closest_point_index, axis=0)

        return reordered_path

    def create_graph(self, path):
        """
        Function to create a graph from a path.
        Each point in the path becomes a node in the graph, and edges are created between adjacent points.
        """
        G = nx.Graph()
        for point in path:
            G.add_node(tuple(point))
        for i in range(len(path) - 1):
            for j in range(i+1, len(path)):
                if np.linalg.norm(np.array(path[i])-np.array(path[j])) == self.drone.grid_resolution:
                    G.add_edge(tuple(path[i]), tuple(path[j]))
        return G

    def reorder_path_spiral_outward(self, start_point, path):
        """
        Function to reorder a path to spiral outward from a specific start point.
        Uses breadth-first search on a graph representation of the path.

        Args:
        start_point : The starting point for the path.
        path : The original path as an N x 2 numpy array.

        Returns:
        reordered_path : The reordered path as an N x 2 numpy array.
        """
        # Create a graph from the path
        G = self.create_graph(path.round())

        # Initialize the reordered path with the start point
        reordered_path = [[start_point[0],start_point[1]]]

        # Use BFS to visit all points, starting from the start point
        for point in nx.bfs_tree(G, source=tuple([round(start_point[0]),round(start_point[1])])):
            reordered_path.append(point)

        return np.array(reordered_path)


    def plot_lawnmower_path(self):
        # Plot the lawnmower path
        fig, ax = plt.subplots()
        ax.set_xlim([-self.drone.field_size/2, self.drone.field_size/2])
        ax.set_ylim([-self.drone.field_size/2, self.drone.field_size/2])

        # Draw Voronoi region
        if isinstance(self.drone.voronoi_region, Polygon):
            self.drone.voronoi_region = [self.drone.voronoi_region]
        for poly in self.drone.voronoi_region:
            ax.fill(*zip(*poly.exterior.coords), alpha=0.4)

        # Draw lawnmower path
        path = np.array(self.drone.lawnmower_path)
        ax.plot(path[:, 0], path[:, 1], 'r')

        # Draw drone positions
        ax.plot(self.drone.position[0], self.drone.position[1], 'ko')
        ax.text(self.drone.position[0], self.drone.position[1], f'Drone {self.drone.id}', fontsize=12)

        # Draw true weed distributions
        plt.scatter(self.drone.grid_points[:, 0], self.drone.grid_points[:, 1], c=self.drone.true_weed_distribution, cmap='YlGn', s=5)

        ax.grid(True, which='both', color='k', linestyle='--', linewidth=0.5)
        ax.set_xticks(np.arange(-self.drone.field_size/2, self.drone.field_size/2 + 1, self.drone.grid_resolution), minor=True)
        ax.set_yticks(np.arange(-self.drone.field_size/2, self.drone.field_size/2 + 1, self.drone.grid_resolution), minor=True)

        ax.set_aspect('equal')
        plt.show()


