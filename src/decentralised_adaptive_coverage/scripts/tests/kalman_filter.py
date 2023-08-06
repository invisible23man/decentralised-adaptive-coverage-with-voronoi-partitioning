import sys
sys.path.append(r'/home/invisible23man/Robotics/Simulations/decentralised-adaptive-coverage-with-voronoi-partitioning/src/decentralised_adaptive_coverage/scripts')

import logging
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from filters.Kalman import KalmanFilter, ExtendedKalmanFilter, UnscentedKalmanFilter

# Configure logging
logging.basicConfig(level=logging.DEBUG)

# Create a simple 1D system for testing
# A = np.array([[1.0]])
# B = np.array([[1.0]])
# H = np.array([[1.0]])
# Q = np.array([[0.01]])
# R = np.array([[0.1]])
# initial_state = np.array([[0.0]])
# initial_covariance = np.array([[1.0]])

dim = 2

A = np.array([[1, 0], [0, 1]])
B =  None
H = np.array([[1, 0], [0, 1]])
Q = np.array([[0.1, 0], [0, 0.1]])
R = np.array([[0.1, 0], [0, 0.1]])

# noise = np.random.normal(loc=0, scale=0.1, size=A.shape)  # adjust loc and scale as needed
# A = np.sum([A,noise])
initial_state = np.zeros((dim))
initial_covariance = np.eye(dim)


# Define the state transition function for the EKF and UKF
def f(state: np.ndarray) -> np.ndarray:
    logging.debug(f"f() input shape: {state.shape}")
    return A @ state

# Define the observation function for the EKF and UKF
def h(state: np.ndarray) -> np.ndarray:
    logging.debug(f"h() input shape: {state.shape}")
    return state

# Define the Jacobian of f with respect to state
def A_jacobian(state: np.ndarray) -> np.ndarray:
    # logging.debug(f"A_jacobian() input shape: {state.shape}")
    return A

# Define the Jacobian of h with respect to predicted state
def H_jacobian(state: np.ndarray) -> np.ndarray:
    # logging.debug(f"H_jacobian() input shape: {state.shape}")
    return H

# Create instances of the filters
kf = KalmanFilter(A, B, H, Q, R, initial_state, initial_covariance)
ekf = ExtendedKalmanFilter(f, h, A_jacobian, B, H_jacobian, Q, R, initial_state, initial_covariance)
ukf = UnscentedKalmanFilter(f, h, initial_state, initial_covariance, Q, R)

# Generate some noisy measurements
np.random.seed(0)
num_steps = 100
true_states = np.zeros((num_steps, dim))
measurements = np.zeros((num_steps, dim))
for i in range(num_steps):
    true_state = f(true_states[i-1]) + np.sqrt(Q) @ np.random.randn(dim)
    measurement = h(true_state) + np.sqrt(R) @ np.random.randn(dim)
    true_states[i] = true_state
    measurements[i] = measurement

# Perform filtering and collect the estimated states
kf_states = np.zeros((num_steps, dim))
ekf_states = np.zeros((num_steps, dim))
ukf_states = np.zeros((num_steps, dim))
for i in range(num_steps):
    kf_states[i] = kf.update(measurements[i])
    ekf_states[i] = ekf.update(measurements[i])
    ukf_states[i] = ukf.update(measurements[i])

# Plot the true states and estimated states
fig, ax = plt.subplots()
for d in list(range(dim)):
    ax.plot(range(num_steps), true_states[:,d], label=f'True States: dim{d}')
    ax.plot(range(num_steps), kf_states[:,d], label=f'Kalman Filter: dim{d}')
    ax.plot(range(num_steps), ekf_states[:,d], label=f'Extended Kalman Filter: dim{d}')
    ax.plot(range(num_steps), ukf_states[:,d], label=f'Unscented Kalman Filter: dim{d}')
ax.set_xlabel('Time Step')
ax.set_ylabel('State')
ax.set_title('State Estimation Comparison')
ax.legend()
plt.show()

# Animation plot of the estimated states
fig, ax = plt.subplots()
line, = ax.plot([], [], lw=2)

def init():
    line.set_data([], [])
    return line,

def animate(i):
    x = range(i+1)
    y = ukf_states[:i+1].flatten()
    line.set_data(x, y)
    return line,

ani = FuncAnimation(fig, animate, frames=num_steps, init_func=init, blit=True)
plt.xlabel('Time Step')
plt.ylabel('Estimated State')
plt.title('Unscented Kalman Filter State Estimation Animation')
# plt.show()
