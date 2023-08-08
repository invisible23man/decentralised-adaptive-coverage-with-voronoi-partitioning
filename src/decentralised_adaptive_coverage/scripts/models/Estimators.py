from sklearn.gaussian_process import GaussianProcessRegressor
from sklearn.gaussian_process.kernels import RBF, ConstantKernel as C
from scipy.stats import norm

import numpy as np

from models import Sensor

class GaussianProcessRegressorEstimator:
    """
    Initialize the Gaussian Process Regressor Estimator.

    Args:
        kernel (GaussianProcessRegressor kernel, optional): The kernel for Gaussian Process Regression.
            Defaults to C(1.0, (1e-3, 1e3)) * RBF(10, (1e-2, 1e2)).
        n_restarts_optimizer (int, optional): The number of restarts for the optimizer. Defaults to 10.
        alpha (float, optional): Regularization parameter for Gaussian Process Regression. Defaults to 0.1.

    Returns:
        None

    Notes:
        This estimator uses the Gaussian Process Regressor from scikit-learn to model the weed density estimation.
        The Gaussian Process Regressor allows capturing the uncertainty in the estimated weed density, providing
        more reliable predictions.

    """
    def __init__(self, kernel = C(1.0, (1e-3, 1e3)) * RBF(10, (1e-2, 1e2)), n_restarts_optimizer=10, alpha=0.1):
        self.gp = GaussianProcessRegressor(kernel=eval(kernel), n_restarts_optimizer= n_restarts_optimizer, alpha=alpha)

    def train(self, X, y):
        """
        Train the Gaussian Process Regressor on the given training data.

        Args:
            X (array-like or matrix): The training data.
            y (array-like): The target values.

        Returns:
            None

        Notes:
            This function trains the Gaussian Process Regressor on the provided training data (X) and target values (y).

        """
        self.gp.fit(X, y)

    def predict(self, X_new):
        """
        Make predictions using the trained Gaussian Process Regressor.

        Args:
            X_new (array-like or matrix): The input data for making predictions.

        Returns:
            tuple: A tuple containing the predicted weed density (y_pred) and the corresponding uncertainties (sigma).

        Notes:
            This function uses the trained Gaussian Process Regressor to make predictions for the given input data (X_new).
            It returns the predicted weed density (y_pred) and the corresponding uncertainties (sigma) for the predictions.

        """
        y_pred, sigma = self.gp.predict(X_new, return_std=True)
        return y_pred, sigma

class ParticleFilterEstimator:
    """
    Initialize the Particle Filter Estimator.

    Args:
        num_grid_points (int): The number of grid points in the field.
        num_particles (int, optional): The number of particles for the particle filter. Defaults to 1000.
        temperature (float, optional): The initial temperature value for the particle filter. Defaults to 1.0.
        cooling_rate (float, optional): The cooling rate for decreasing the temperature. Defaults to 0.99.

    Returns:
        None

    Notes:
        This estimator uses a particle filter to estimate the weed density in the field. The particle filter
        represents the uncertainty in the estimation process by maintaining a set of particles with associated
        weights.

    """
    def __init__(self, num_grid_points, num_particles=1000, temperature=1.0, cooling_rate=0.99):
        self.num_grid_points = num_grid_points
        self.num_particles = num_particles
        self.temperature = temperature
        self.cooling_rate = cooling_rate
        self.particles = np.ones((num_grid_points, num_particles)) / num_particles
        self.particle_weights = np.ones((num_grid_points, num_particles)) / num_particles

    def update(self, measurement, index_1d):
        """
        Update the particle filter using the latest measurement.

        Args:
            measurement (float): The latest measured weed density.
            index_1d (int): The grid index corresponding to the measurement.

        Returns:
            Tuple: A tuple containing the estimated weed density (estimated_weed_density), the updated particles
                (self.particles), the updated particle weights (self.particle_weights), and the updated temperature
                (self.temperature).

        Notes:
            This function updates the particle filter using the latest measurement and grid index. It calculates the
            new particle weights based on the measured weed density, performs systematic resampling, and updates the
            estimated weed density. The function also perturbs the particles, decreases the temperature, and returns
            the updated values.

        """

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
    """
    Handles the estimation of weed density while considering measurement uncertainties.

    Args:
        drone (UAV.Drone): The drone object containing the measured data and estimator configuration.
        scaling_factor (float): A scaling factor to adjust the estimated weed density.

    Returns:
        Tuple: The tuple containing the estimated weed density (mv) and the centroid coordinates (cx, cy).

    Raises:
        None

    Notes:
        This function calculates the estimated weed density (mv) and centroid coordinates (cx, cy) based on the
        measured data and the chosen method for weighing uncertainty. The uncertainties are obtained from the drone's
        `estimate_uncertainties` attribute.

        If `weigh_uncertainty` in `estimator_config` is set to 'individually', each measurement's uncertainty is used
        to weigh the corresponding estimate. If set to 'partitionwise', the average uncertainty is applied to weigh the
        entire partition of measurements.

        The function updates the `estimate_uncertainty_tracker` attribute in the drone object, which tracks the
        uncertainties for each estimation.

    """

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
    