import sys
sys.path.append(r'/home/invisible23man/Robotics/Simulations/decentralised-adaptive-coverage-with-voronoi-partitioning/src/decentralised_adaptive_coverage')

from utils.plots import plot_gaussian_sensor_model
from sensor import apply_gaussian_sensor_model, sample_weed_density
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

import numpy as np

# Test function
def test_gaussian_sensor_model():
    # Generate a set of points in a grid
    x = np.linspace(-10, 10, 100)
    y = np.linspace(-10, 10, 100)
    points = np.array([[xi, yi] for xi in x for yi in y])

    # Define the mean and covariance of the Gaussian distribution
    mean = np.array([0, 0])
    covariance = np.array([[1, 0], [0, 1]])

    sensor_values = apply_gaussian_sensor_model(points, mean, covariance)

    # Plot the Gaussian sensor model
    plot_gaussian_sensor_model(points, sensor_values)


def test_sample_weed_density():
    # Define the sensor function as a Gaussian
    def sensor_func(points):
        return np.exp(-np.sum(points**2, axis=1))

    # Create a grid of points
    x, y = np.meshgrid(np.linspace(-3, 3, 100), np.linspace(-3, 3, 100))
    points = np.column_stack((x.ravel(), y.ravel()))

    # Sample the weed density using Gaussian noise
    sampled_density_gaussian = sample_weed_density(sensor_func, points, noise_model='gaussian').reshape(x.shape)

    # Sample the weed density using Perlin noise
    sampled_density_perlin = sample_weed_density(sensor_func, points, noise_model='perlin').reshape(x.shape)

    # Plot the true density and the sampled densities
    fig = plt.figure(figsize=(18, 6))

    ax1 = fig.add_subplot(131, projection='3d')
    ax1.plot_surface(x, y, sensor_func(points).reshape(x.shape))
    ax1.set_title('True Density')

    ax2 = fig.add_subplot(132, projection='3d')
    ax2.plot_surface(x, y, sampled_density_gaussian)
    ax2.set_title('Sampled Density (Gaussian Noise)')

    ax3 = fig.add_subplot(133, projection='3d')
    ax3.plot_surface(x, y, sampled_density_perlin)
    ax3.set_title('Sampled Density (Perlin Noise)')

    plt.show()

if __name__ == '__main__':

    if len(sys.argv) > 1:
        test_option = sys.argv[1]

        if test_option == 'test-sensor-model':
            test_gaussian_sensor_model()
        elif test_option == 'test-sensor-sampling':
            test_sample_weed_density()
        else:
            print("Invalid test option. Available options: test-sensor-model, test-sensor-sampling")
    else:
        print("No test option provided. Available options: test-sensor-model, test-sensor-sampling")


