from sklearn.gaussian_process import GaussianProcessRegressor
from sklearn.gaussian_process.kernels import RBF, ConstantKernel as C
from scipy.stats import norm

import numpy as np

from models import Sensor

class GaussianProcessRegressorEstimator:
    def __init__(self, kernel = C(1.0, (1e-3, 1e3)) * RBF(10, (1e-2, 1e2)), n_restarts_optimizer=10, alpha=0.1):
        self.gp = GaussianProcessRegressor(kernel=eval(kernel), n_restarts_optimizer= n_restarts_optimizer, alpha=alpha)

    def train(self, X, y):
        self.gp.fit(X, y)

    def predict(self, X_new):
        y_pred, sigma = self.gp.predict(X_new, return_std=True)
        return y_pred, sigma

class ParticleFilterEstimator:
    def __init__(self, num_grid_points, num_particles=1000, temperature=1.0, cooling_rate=0.99):
        self.num_grid_points = num_grid_points
        self.num_particles = num_particles
        self.temperature = temperature
        self.cooling_rate = cooling_rate
        self.particles = np.ones((num_grid_points, num_particles)) / num_particles
        self.particle_weights = np.ones((num_grid_points, num_particles)) / num_particles

    def update(self, measurement, index_1d):
        # Update particle weights based on the sensed measurement
        self.particle_weights[index_1d] = norm.pdf(measurement, loc=self.particles[index_1d], scale=1.0)

        # Check if all weights are zero
        if np.all(self.particle_weights[index_1d] == 0):
            self.particle_weights[index_1d] = np.full_like(self.particle_weights[index_1d], 1e-10)
        else:
            self.particle_weights[index_1d] /= self.particle_weights[index_1d].sum()  # Normalize weights

        self.particle_weights[index_1d] = Sensor.systematic_resample(self.particle_weights[index_1d])

        # Update the estimated weed density for the current grid index
        estimated_weed_density = np.average(self.particles[index_1d], weights=self.particle_weights[index_1d])

        # Resample particles
        if np.random.uniform() < self.temperature:
            # Random resampling
            self.particle_weights[index_1d] = np.ones_like(self.particle_weights[index_1d]) / self.num_particles
        else:
            # Systematic resampling
            self.particle_weights[index_1d] = Sensor.systematic_resample(self.particle_weights[index_1d])

        # Randomly perturb particles
        self.particles[index_1d] += np.random.normal(scale=self.temperature, size=self.num_particles)

        # Decrease temperature
        self.temperature *= self.cooling_rate

        return estimated_weed_density, self.particles[index_1d], self.particle_weights[index_1d], self.temperature

def handle_estimate_uncertainities(drone, scaling_factor):

        if drone.estimator_config["weigh_uncertainity"]=="individually":
            epsilon = 1e-6  # very small uncertainty for true measurements
            uncertainties = np.append(drone.estimate_uncertainities, epsilon*np.ones(drone.lawnmower_sampling_path.shape[0]))
            drone.estimate_uncertainty_tracker.append(uncertainties)

            weights = 1 / (uncertainties ** 2)
            mv = np.sum(drone.measurements * weights) * scaling_factor
            cx = np.sum(np.squeeze(drone.lawnmower_path[:, 0]) * drone.measurements * weights) / mv
            cy = np.sum(np.squeeze(drone.lawnmower_path[:, 1]) * drone.measurements * weights) / mv
            return mv, cx, cy

        if drone.estimator_config["weigh_uncertainity"]=="partitionwise":
            epsilon = 1e-6  # very small uncertainty for true measurements
            uncertainties = np.append(drone.estimate_uncertainities, epsilon*np.ones(drone.lawnmower_sampling_path.shape[0]))
            drone.estimate_uncertainty_tracker.append(uncertainties)

            mv = np.sum(drone.measurements)*scaling_factor
            cx = np.sum(np.squeeze(drone.lawnmower_path[:, 0]) * drone.measurements) / mv *(1-drone.estimate_uncertainities.mean())
            cy = np.sum(np.squeeze(drone.lawnmower_path[:, 1]) * drone.measurements) / mv *(1-drone.estimate_uncertainities.mean())
            return mv, cx, cy
        