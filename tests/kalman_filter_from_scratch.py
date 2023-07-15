import numpy as np

class KalmanFilter:
    def __init__(self, F, B, H, Q, R, x0, P0):
        self.F = F
        self.B = B
        self.H = H
        self.Q = Q
        self.R = R
        self.x = x0
        self.P = P0

    def predict(self, u):
        self.x = self.F @ self.x + self.B @ u
        self.P = self.F @ self.P @ self.F.T + self.Q

    def update(self, z):
        K = self.P @ self.H.T @ np.linalg.inv(self.H @ self.P @ self.H.T + self.R)
        self.x = self.x + K @ (z - self.H @ self.x)
        self.P = (np.eye(self.P.shape[0]) - K @ self.H) @ self.P

    def run(self, states, measurements, input):
        for i in range(len(states)):
            self.predict(input)
            self.update(measurements[i])
            states[i] = self.x.flatten()
        return states

# Define the system matrices
F = np.array([[1, 0.1], [0, 1]])
B = np.array([[0.005], [0.1]])
H = np.array([[1, 0]])
Q = np.array([[0.1, 0], [0, 0.1]])
R = 1

# Define the initial state estimate and covariance
x0 = np.array([[10], [0]])
P0 = np.array([[1, 0], [0, 1]])

# Initialize the Kalman Filter
kf = KalmanFilter(F, B, H, Q, R, x0, P0)

# Define the acceleration profile
a = np.ones(100) * 2

# Generate noisy measurements
z = np.array([np.random.normal((i / 10) ** 2 + 10, np.sqrt(R)) for i in range(100)])

# Define Control Input
u = np.array([[0]])

# Run the Kalman Filter
x_est = kf.run(np.zeros((100, 2)), z, u)

# Plot the true and estimated position vs. time
import matplotlib.pyplot as plt

t = np.arange(100) * 0.1
plt.plot(t, (t ** 2) + 10, label='True position')
plt.plot(t, x_est[:, 0], label='Estimated position')
plt.xlabel('Time (s)')
plt.ylabel('Position (m)')
plt.legend()
plt.show()
