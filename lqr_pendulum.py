import numpy as np
import matplotlib.pyplot as plt
from scipy.linalg import solve_continuous_are
import matplotlib.animation as animation

# Pendulum parameters
m = 1.0  # mass of the pendulum
l = 1.0  # length of the pendulum
g = 9.81  # acceleration due to gravity
b = 0.1  # damping coefficient

# State-space representation
A = np.array([[0, 1],
              [g/l, -b/(m*l**2)]])
B = np.array([[0],
              [1/(m*l**2)]])
Q = np.diag([10, 1])  # State cost matrix
R = np.array([[0.1]])  # Control cost matrix

# Solve the continuous-time algebraic Riccati equation
P = solve_continuous_are(A, B, Q, R)

# Compute the LQR gain
K = np.linalg.inv(R) @ B.T @ P

# Simulation parameters
dt = 0.01  # time step
T = 10  # total simulation time
N = int(T / dt)  # number of time steps

# Initial state
x = np.array([[np.pi - 0.1],  # initial angle (slightly perturbed from upright)
              [0]])  # initial angular velocity

# Arrays to store the results
time = np.linspace(0, T, N)
theta = np.zeros(N)
theta_dot = np.zeros(N)
u = np.zeros(N)

# Simulation loop
for i in range(N):
    u[i] = -K @ x  # LQR control law
    x_dot = A @ x + B * u[i]  # state-space equations
    x += x_dot * dt  # Euler integration
    theta[i] = x[0]
    theta_dot[i] = x[1]

# Set up the figure and axis for live plot
fig, ax = plt.subplots()
line, = ax.plot([], [], 'o-', lw=2)
ax.set_xlim(-l-0.5, l+0.5)
ax.set_ylim(l+0.5, -l-0.5)  # Flipping the y-axis
ax.set_aspect('equal')  # Make aspect ratio equal
ax.set_xlabel('X position')
ax.set_ylabel('Y position')
ax.set_title('Live Pendulum Simulation')

# Function to update the plot
def update(frame):
    x_pos = l * np.sin(theta[frame])
    y_pos = -l * np.cos(theta[frame])
    line.set_data([0, x_pos], [0, y_pos])
    return line,

# Create the animation
ani = animation.FuncAnimation(fig, update, frames=N, blit=True, interval=dt*1000)

# Show the plot
plt.show()

# Plot the results
plt.figure()
plt.subplot(3, 1, 1)
plt.plot(time, theta)
plt.title('Pendulum Angle')
plt.ylabel('Theta (rad)')

plt.subplot(3, 1, 2)
plt.plot(time, theta_dot)
plt.title('Pendulum Angular Velocity')
plt.ylabel('Theta_dot (rad/s)')

plt.subplot(3, 1, 3)
plt.plot(time, u)
plt.title('Control Input')
plt.ylabel('u (N*m)')
plt.xlabel('Time (s)')

plt.tight_layout()
plt.show()
