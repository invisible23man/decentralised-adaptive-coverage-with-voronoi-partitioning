from matplotlib import animation
import numpy as np
from scipy.stats import multivariate_normal
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import pickle
import os
from tools import voronoi
import copy

class Field:
    def __init__(self, size, grid_resolution, drone_count, formation_pattern, weed_centers, weed_cov, sampling_time = 10):
        self.size = size
        self.grid_resolution = grid_resolution
        self.drone_count = drone_count
        self.weed_centers = weed_centers
        self.weed_cov = weed_cov
        self.sampling_time = sampling_time
        
        # Create square grid
        self.x_values, self.y_values, self.grid_points = self.create_square_grid()
        
        # Distribute drones
        self.drone_positions = self.distribute_drones(formation_pattern)
        
        # Generate weed distribution
        self.weed_distribution = self.generate_weed_distribution()
        
        # Experiment Logging
        self.drone_position_tracker = [self.drone_positions]
        self.path_tracker = [[] for _ in range(drone_count)]
        self.measurement_tracker = [[] for _ in range(drone_count)]        

    def create_square_grid(self):
        x_values = np.arange(-self.size/2, self.size/2 + self.grid_resolution, self.grid_resolution)
        y_values = np.arange(-self.size/2, self.size/2  + self.grid_resolution, self.grid_resolution)
        grid_points = np.array([(x, y) for y in y_values for x in x_values])
        return x_values, y_values, grid_points

    def get_grid_coordinates(self, point):
        i = int((point[0] + self.size / 2) / self.grid_resolution)
        j = int((point[1] + self.size / 2) / self.grid_resolution)
        return i, j

    def distribute_drones(self, pattern="circle"):
        if pattern == "circle":
            return self.distribute_drones_circle()
        elif pattern == "grid":
            return self.distribute_drones_grid()
        else:
            raise ValueError("Unrecogonised patter. Specify from circle or grid.")

    def distribute_drones_circle(self):
        # Radius of the circle on which drones will be positioned
        radius = self.size / 4  # You can adjust this as needed

        # Angle step for each drone
        angle_step = 2 * np.pi / self.drone_count

        # Distribute drones along the circumference of a circle
        drone_positions = []
        for i in range(self.drone_count):
            angle = i * angle_step
            x = radius * np.cos(angle)
            y = radius * np.sin(angle)

            # Make sure the position is a valid grid coordinate
            grid_x, grid_y = self.get_grid_coordinates((x, y))
            valid_x = self.grid_points[grid_x * len(self.y_values) + grid_y, 0]
            valid_y = self.grid_points[grid_x * len(self.y_values) + grid_y, 1]

            drone_positions.append((valid_x, valid_y, 3))  # Z-coordinate of 3m

        return np.array(drone_positions)

    def distribute_drones_grid(self):
        # Distribute drones in grid pattern
        total_drones = self.drone_count
        rows = int(np.sqrt(total_drones))
        cols = rows

        # Adjust rows and columns if not a perfect square
        while rows * cols < total_drones:
            if rows < cols:
                rows += 1
            else:
                cols += 1

        drone_positions = []
        step_x = self.size / rows
        step_y = self.size / cols
        for i in range(rows):
            for j in range(cols):
                if len(drone_positions) < total_drones:  # Ensure we don't exceed the total number of drones
                    x = -self.size/2 + step_x * i + step_x/2
                    y = -self.size/2 + step_y * j + step_y/2
                    drone_positions.append((x, y, 3))  # Z-coordinate of 3m

        return np.array(drone_positions)


    def generate_weed_distribution(self):
        # Generate weed concentration distribution within the grid
        weed_distribution = np.zeros(self.grid_points.shape[0])
        for center in self.weed_centers:
            rv = multivariate_normal(center, self.weed_cov)
            weed_distribution += rv.pdf(self.grid_points)
        return weed_distribution

    def update_drone_positions(self, drone_positions):
        # self.drone_positions = drone_positions
        self.drone_position_tracker.append(copy.deepcopy(drone_positions))

    def update_path_and_measurements(self, drones):
        for i, drone in enumerate(drones):
            self.path_tracker[i].append(drone.lawnmower_path)
            self.measurement_tracker[i].append(drone.measurements)

    def save_data(self, filename):
        data = {
            'positions': self.drone_position_tracker,
            'paths': self.path_tracker,
            'measurements': self.measurement_tracker
        }
        with open(filename, 'wb') as f:
            pickle.dump(data, f)

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

    def animate_field_2d(self, plot_voronoi=False, filename=None):
        fig, ax = plt.subplots()

        # Initial plot setup
        ax.set_xlim([-self.size/2, self.size/2])
        ax.set_ylim([-self.size/2, self.size/2])
        ax.scatter(self.grid_points[:, 0], self.grid_points[:, 1], c=self.weed_distribution, cmap='YlGn', s=5)
        drone_positions = np.array(self.drone_position_tracker[0])
        scat = ax.scatter(drone_positions[:, 0], drone_positions[:, 1], color='red')

        if plot_voronoi:
            voronoi_calculator = voronoi.VoronoiCalculator(drone_positions[:, :2], 'square', self.size)
            voronoi_regions = voronoi_calculator.compute_voronoi()

            for poly in voronoi_regions:
                ax.fill(*zip(*poly.exterior.coords), alpha=0.4)                

        def animate(i):
            drone_positions = np.array(self.drone_position_tracker[i])
            ax.clear()
            ax.scatter(self.grid_points[:, 0], self.grid_points[:, 1], c=self.weed_distribution, cmap='YlGn', s=5)
            scat = ax.scatter(drone_positions[:, 0], drone_positions[:, 1], color='red')

            if plot_voronoi:
                voronoi_calculator = voronoi.VoronoiCalculator(drone_positions[:, :2], 'square', self.size)
                voronoi_regions = voronoi_calculator.compute_voronoi()

                for poly in voronoi_regions:
                    ax.fill(*zip(*poly.exterior.coords), alpha=0.4)                

            ax.set_xlim([-self.size/2, self.size/2])
            ax.set_ylim([-self.size/2, self.size/2])
            ax.set_title(f'Field with Drones and Weed Distribution - Iteration {i+1}')

        ani = animation.FuncAnimation(fig, animate, frames=len(self.drone_position_tracker), repeat=True)

        if filename:
            ani.save(filename)

        plt.show()

    def animate_field_3d(self, plot_voronoi= False, filename=None):
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')

        X, Y = np.meshgrid(self.x_values, self.y_values)
        drone_positions = np.array(self.drone_position_tracker[0])
        scat = ax.scatter(drone_positions[:, 0], drone_positions[:, 1], drone_positions[:, 2], color='red')
        ax.plot_surface(X, Y, self.weed_distribution.reshape(X.shape), cmap='viridis', alpha=0.5)
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Weed density')
        ax.set_title('Field with Drones and Weed Distribution (3D)')

        if plot_voronoi:
            voronoi_calculator = voronoi.VoronoiCalculator(drone_positions[:, :2], 'square', self.size)
            voronoi_regions = voronoi_calculator.compute_voronoi()

            for poly in voronoi_regions:
                region = np.array(poly.exterior.coords)
                Z = np.full(region.shape[0], drone_positions[:, 2][0]-0.2)
                ax.plot_trisurf(region[:,0], region[:,1], Z, alpha=0.4)

        def animate(i):
            drone_positions = np.array(self.drone_position_tracker[i])
            ax.clear()
            ax.plot_surface(X, Y, self.weed_distribution.reshape(X.shape), cmap='viridis', alpha=0.5)
            scat = ax.scatter(drone_positions[:, 0], drone_positions[:, 1], drone_positions[:, 2], color='red')

            if plot_voronoi:
                voronoi_calculator = voronoi.VoronoiCalculator(drone_positions[:, :2], 'square', self.size)
                voronoi_regions = voronoi_calculator.compute_voronoi()

                for poly in voronoi_regions:
                    region = np.array(poly.exterior.coords)
                    Z = np.full(region.shape[0], drone_positions[:, 2][0]-0.2)
                    ax.plot_trisurf(region[:,0], region[:,1], Z, alpha=0.4)

            ax.set_xlabel('X')
            ax.set_ylabel('Y')
            ax.set_zlabel('Weed density')
            ax.set_title(f'Field with Drones and Weed Distribution (3D) - Iteration {i+1}')

        ani = animation.FuncAnimation(fig, animate, frames=len(self.drone_position_tracker), repeat=True)

        if filename:
            ani.save(filename)

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
