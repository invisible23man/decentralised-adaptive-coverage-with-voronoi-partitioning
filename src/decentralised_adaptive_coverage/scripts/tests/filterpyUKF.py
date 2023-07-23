import numpy as np
import matplotlib.pyplot as plt
from filterpy.kalman import UnscentedKalmanFilter, MerweScaledSigmaPoints
from filterpy.common import Q_discrete_white_noise

def f(state, dt):
    A = np.array([[1, dt], [0, 1]])
    return A @ state

def h(state):
    return state

dim = 2
dt = 1.0

# Initialize the sigma points
points = MerweScaledSigmaPoints(n=dim, alpha=1e-3, beta=2, kappa=0)

# Initialize the UKF
ukf = UnscentedKalmanFilter(dim_x=dim, dim_z=dim, dt=dt, points=points, fx=f, hx=h)

# Initial state and covariance
initial_state = np.zeros((dim))
initial_covariance = np.eye(dim) * 0.1

ukf.x = initial_state
ukf.P = initial_covariance

# Process noise
ukf.Q = Q_discrete_white_noise(dim=dim, dt=dt, var=0.1)

# Measurement noise
ukf.R = np.array([[0.1, 0], [0, 0.1]])

# Generate synthetic measurement data
true_positions = np.array([[i ** 2 + 10, 0] for i in range(100)])
measurements = true_positions + np.random.normal(loc=0, scale=np.sqrt(ukf.R[0, 0]), size=true_positions.shape)

# Perform the filtering for each measurement
filtered_states = []

for z in measurements:
    ukf.predict(dt=dt)
    ukf.update(z)
    filtered_states.append(ukf.x)

# Extract true positions and estimated positions
true_positions = np.array(true_positions)
filtered_states = np.array(filtered_states)

# Plot the true and estimated position vs. time
t = np.arange(100) * dt
plt.plot(t, true_positions[:, 0], label='True position')
plt.plot(t, filtered_states[:, 0], label='Estimated position')
plt.xlabel('Time (s)')
plt.ylabel('Position (m)')
plt.legend()
plt.grid(True)
plt.show()
