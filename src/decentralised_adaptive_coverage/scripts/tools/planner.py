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
        minx, miny, maxx, maxy = self.drone.voronoi_region[self.drone.drone_id].bounds
        x = np.arange(round(minx), round(maxx), self.drone.grid_resolution)
        y = np.arange(round(miny), round(maxy), self.drone.grid_resolution)
        path = []
        for i, xi in enumerate(x):
            if i % 2 == 0:  # if i is even
                y_order = y  # iterate through y in ascending order
            else:
                y_order = y[::-1]  # iterate through y in descending order
            for yi in y_order:
                point = Point(xi, yi)
                if self.drone.voronoi_region[self.drone.drone_id].contains(point):
                    path.append((xi.round(), yi.round(), self.drone.altitude))

        self.drone.lawnmower_path = np.array(path)
        if self.drone.planner_config["reordermode"]:
            self.drone.lawnmower_path = self.reorder_path(self.drone.voronoi_center.round(), self.drone.lawnmower_path, mode=self.drone.planner_config["reordermode"])

    def reorder_path(self, start_point, path, mode="NearestNeighbor"):
        if mode == "NearestNeighbor":
            return self.reorder_nearest_neighbor(start_point, path)
        elif mode == "SpiralOutward":
            return self.reorder_path_spiral_outward(start_point, path)
        elif mode == "SpiralOutA*":
            return self.reorder_path_spiral_outward_astar(start_point, path)
        elif mode == "SpiralOutSimple":
            return self.reorder_path_spiral_outward_simple(start_point, path)
        else:
            raise ValueError("Invalid mode. Choose 'NearestNeighbor', 'SpiralOutward', 'SpiralOutA*, 'SpiralOutSimple")

    def reorder_path_spiral_outward_astar(self, start_point, path):
        """
        Function to reorder a path to spiral outward from a specific start point.
        Uses A* search algorithm on a graph representation of the path.

        Args:
        start_point : The starting point for the path.
        path : The original path as an N x 2 numpy array.

        Returns:
        reordered_path : The reordered path as an N x 2 numpy array.
        """

        # Set A* settings 
        from tools.astar import heuristic, shortest_path

        self.heuristic = heuristic
        self.shortest_path = shortest_path

        # Create a graph from the path
        self.graph = self.create_graph(path.round())
        
        # Initialize the reordered path with the start point
        reordered_path = [[start_point[0],start_point[1]]]

        # Use A* search to visit all points, starting from the start point
        for point in self.full_path(tuple(start_point),path):
            reordered_path.append(point)

        return np.array(reordered_path)

    def reorder_path_spiral_outward_simple(self, start_point, path):
        """
        Function to reorder a path to spiral outward from a specific start point.
        Uses a spiral movement pattern and a graph representation of the path.

        Args:
        start_point : The starting point for the path.
        path : The original path as an N x 2 numpy array.

        Returns:
        reordered_path : The reordered path as an N x 2 numpy array.
        """
        def point_in_path(point, path):
            return any((point == p).all() for p in path)

        # Define directions (right, down, left, up)
        directions = np.array([[1, 0, 0], [0, -1, 0], [-1, 0, 0], [0, 1, 0]])

        # Initialize variables for spiral movement
        direction_index = 0  # Start with moving right
        steps_in_current_direction = 1  # Start with one step
        steps_taken_in_current_direction = 0

        # Current position starts as the start point
        current_position = start_point.copy()

        # Initialize the reordered path with the start point
        reordered_path = [current_position]

        # Repeat until all points have been visited
        while len(reordered_path) < len(path):
            # If we have taken the planned number of steps in this direction, switch direction
            if steps_taken_in_current_direction == steps_in_current_direction:
                # Switch to the next direction
                direction_index = (direction_index + 1) % 4
                steps_taken_in_current_direction = 0

                # Increase steps in current direction every two directions
                if direction_index % 2 == 0:
                    steps_in_current_direction += 1

            # Calculate next position
            next_position = current_position + directions[direction_index]

            # Check if the next position is within the polygon and on the path
            while not point_in_path(next_position, path):
                next_position += directions[direction_index]

            # Add the next position to the path
            reordered_path.append(next_position)

            # Update the current position
            current_position = next_position

            # Count the step
            steps_taken_in_current_direction += 1

        return np.array(reordered_path)


    def reorder_nearest_neighbor(self, start_point, path):
        # Nearest neighbor approach as before

        # Create a copy of the path
        remaining_points = path.copy()

        # Initialize the reordered path with the start point
        reordered_path = np.array([[start_point[0],start_point[1],start_point[2]]])

        # While there are still points to visit
        while remaining_points.size > 0:
            # Compute the distance from the current point to all remaining points
            distances = distance_matrix([reordered_path[-1]], remaining_points, p=reordered_path.shape[1])

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
        node_weights = {}
        for point in path:
            node = tuple(point)
            G.add_node(node)
            node_weights[node] = np.linalg.norm(np.array(point)-np.array(self.drone.voronoi_center))
        for i in range(len(path) - 1):
            for j in range(i+1, len(path)):
                if np.linalg.norm(np.array(path[i])-np.array(path[j])) == self.drone.grid_resolution:
                    weight = max(node_weights[tuple(path[i])], node_weights[tuple(path[j])])
                    G.add_edge(tuple(path[i]), tuple(path[j]), weight=weight)
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
        reordered_path = [[start_point[0],start_point[1], start_point[2]]]

        # Use BFS to visit all points, starting from the start point
        for point in nx.bfs_tree(G, source=tuple(start_point)):
            reordered_path.append(point)

        return np.array(reordered_path)

    def get_nearest_unvisited_node(self, current, visited, nodes):
        visited_tuples = [tuple(node) for node in visited]
        unvisited_nodes = [node for node in nodes if tuple(node) not in visited_tuples]
        distances = [self.heuristic(current, node) for node in unvisited_nodes]
        return unvisited_nodes[np.argmin(distances)]


    def full_path(self, start, nodes):
        visited = []
        current = start
        while len(visited) < len(nodes):
            goal = tuple(self.get_nearest_unvisited_node(current, visited, nodes))
            path_segment = self.shortest_path(self.graph, current, goal)
            visited.extend(path_segment)
            current = goal
        return np.array(visited)

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
        ax.plot(self.drone.voronoi_center[0], self.drone.voronoi_center[1], 'ko')
        ax.text(self.drone.voronoi_center[0], self.drone.voronoi_center[1], f'Drone {self.drone.drone_id}', fontsize=12)

        # Draw true weed distributions
        plt.scatter(self.drone.grid_points[:, 0], self.drone.grid_points[:, 1], c=self.drone.true_weed_distribution, cmap='YlGn', s=5)

        ax.grid(True, which='both', color='k', linestyle='--', linewidth=0.5)
        ax.set_xticks(np.arange(-self.drone.field_size/2, self.drone.field_size/2 + 1, self.drone.grid_resolution), minor=True)
        ax.set_yticks(np.arange(-self.drone.field_size/2, self.drone.field_size/2 + 1, self.drone.grid_resolution), minor=True)

        ax.set_aspect('equal')
        plt.show()


