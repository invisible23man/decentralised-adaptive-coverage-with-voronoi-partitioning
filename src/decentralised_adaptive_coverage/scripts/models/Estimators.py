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
    @staticmethod
    def update(measurement, particles, particle_weights, num_particles, temperature, cooling_rate):
        # Update particle weights based on the sensed measurement
        particle_weights = norm.pdf(measurement, loc=particles, scale=1.0)
        particle_weights /= particle_weights.sum()  # Normalize weights
        particle_weights = Sensor.systematic_resample(particle_weights)

        # Update the estimated weed density for the current grid index
        estimated_weed_density = np.average(particles, weights=particle_weights)

        # Resample particles
        if np.random.uniform() < temperature:
            # Random resampling
            particle_weights = np.ones_like(particle_weights) / num_particles
        else:
            # Systematic resampling
            particle_weights = Sensor.systematic_resample(particle_weights)

        # Randomly perturb particles
        particles += np.random.normal(scale=temperature, size=num_particles)

        # Decrease temperature
        temperature *= cooling_rate

        return estimated_weed_density, particles, particle_weights

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
        