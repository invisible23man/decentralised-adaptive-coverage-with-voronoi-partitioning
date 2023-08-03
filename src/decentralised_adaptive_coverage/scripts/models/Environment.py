import numpy as np
from scipy.stats import multivariate_normal
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

class Field:
    def __init__(self, size, grid_resolution, drone_count, weed_centers, weed_cov):
        self.size = size
        self.grid_resolution = grid_resolution
        self.drone_count = drone_count
        self.weed_centers = weed_centers
        self.weed_cov = weed_cov
        
        # Create square grid
        self.x_values, self.y_values, self.grid_points = self.create_square_grid()
        
        # Distribute drones
        self.drone_positions = self.distribute_drones()
        
        # Generate weed distribution
        self.weed_distribution = self.generate_weed_distribution()
        
    def create_square_grid(self):
        x_values = np.arange(-self.size/2, self.size/2 + self.grid_resolution, self.grid_resolution)
        y_values = np.arange(-self.size/2, self.size/2  + self.grid_resolution, self.grid_resolution)
        grid_points = np.array([(x, y) for y in y_values for x in x_values])
        return x_values, y_values, grid_points

    def distribute_drones(self):
        # Distribute drones equidistant in square pattern
        side_length = int(np.sqrt(self.drone_count))
        drone_positions = []
        step = self.size / side_length
        for i in range(side_length):
            for j in range(side_length):
                x = -self.size/2 + step * i + step/2
                y = -self.size/2 + step * j + step/2
                drone_positions.append((x, y, 5)) # Z-coordinate of 5m
        return np.array(drone_positions)

    def generate_weed_distribution(self):
        # Generate weed concentration distribution within the grid
        weed_distribution = np.zeros(self.grid_points.shape[0])
        for center in self.weed_centers:
            rv = multivariate_normal(center, self.weed_cov)
            weed_distribution += rv.pdf(self.grid_points)
        return weed_distribution
    
    def plot_field(self):
        # Plot the field with drone positions and weed distribution
        plt.scatter(self.grid_points[:, 0], self.grid_points[:, 1], c=self.weed_distribution, cmap='YlGn', s=5)
        plt.scatter(self.drone_positions[:, 0], self.drone_positions[:, 1], color='red', label='Drones')
        plt.title('Field with Drones and Weed Distribution')
        plt.legend()
        plt.colorbar(label='Weed Concentration')
        plt.axis('equal')
        plt.show()

    def plot_field_3d(self):
        """
        Function to plot a 3D representation of the field.
        """
        fig = plt.figure(figsize=(10, 8))
        ax = fig.add_subplot(111, projection='3d')
        X, Y = np.meshgrid(self.x_values, self.y_values)
        ax.plot_surface(X, Y, self.weed_distribution.reshape(X.shape), cmap='viridis')
        ax.scatter(self.drone_positions[:, 0], self.drone_positions[:, 1], self.drone_positions[:, 2], color='red', label='Drones', alpha=0.5, s=10)
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Weed density')
        ax.set_title('Field with Drones and Weed Distribution (3D)')
        plt.show()

if __name__ == '__main__':
    # Example usage
    size = 50
    grid_resolution = 1
    drone_count = 16
    weed_centers = [[-15, -15], [15, 15]]
    weed_cov = [[5, 0], [0, 5]]

    # plt.ion()  # Turn on interactive mode
    field = Field(size, grid_resolution, drone_count, weed_centers, weed_cov)
    field.plot_field()
    field.plot_field_3d()

    # Updating drone positions
    field.drone_positions = [(x+2, y+2, z) for x, y, z in field.drone_positions]
    field.plot_field()
    field.plot_field_3d()
