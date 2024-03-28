import numpy as np
import cvxpy as cp
import matplotlib.pyplot as plt
import matplotlib.animation as animation

# System parameters
dt = 0.1  # time step
N = 20    # prediction horizon
track_length = 100.0  # length of the track for demonstration

# Track definition (circular track for this example)
theta = np.linspace(0, 2*np.pi, 500)
track_x = track_length/2/np.pi * np.cos(theta) + track_length/2
track_y = track_length/2/np.pi * np.sin(theta)

# Initial state in the middle of the track
x0 = np.array([track_length/2, track_length/(2*np.pi)])  # initial position at the center of the track

# Initial state to the left of the track
x0 = np.array([0, track_length/(2*np.pi)])  # initial position to the left of the track

# State update matrix
A = np.array([[1, dt],
              [0, 1]])
B = np.array([[0.5 * dt**2],
              [dt]])

# MPC setup
x = cp.Variable((2, N+1))
u = cp.Variable((1, N))

# Animation setup
fig, ax = plt.subplots(figsize=(10, 6))
line, = ax.plot([], [], 'o-', lw=2, color='blue', markersize=10)  # Increased marker size for visibility
track_line, = ax.plot(track_x, track_y, 'k--', label='Track')
ax.set_xlim(-10, track_length + 10)
ax.set_ylim(-track_length/2 - 10, track_length/2 + 10)
ax.set_xlabel('X position')
ax.set_ylabel('Y position')
ax.legend()

# Initialize target position
target_index = 0
target_position = np.array([track_x[target_index], track_y[target_index]])

def update(i):
    global x0, target_index, target_position
    # Check if the car has reached the current target position
    if np.linalg.norm(x0 - target_position) < 1.0:  # Threshold to consider as reached
        target_index = (target_index + 1) % len(theta)  # Move to the next target
        target_position = np.array([track_x[target_index], track_y[target_index]])
    
    target_velocity = 0.0   # target velocity

    # Objective and constraints
    objective = cp.Minimize(sum(cp.square(x[0, k+1] - target_position[0]) + cp.square(x[1, k+1] - target_position[1]) for k in range(N)))
    constraints = [x[:, 0] == x0]
    for k in range(N):
        constraints += [x[:, k+1] == A @ x[:, k] + B @ u[:, k]]
        constraints += [cp.abs(u[0, k]) <= 1.0]  # acceleration limit

    # Solve MPC
    prob = cp.Problem(objective, constraints)
    prob.solve()

    # Update the car's position
    x0 = x.value[:, 1]

    # Clear the previous car positions to avoid blue trail
    # ax.collections.clear()

    # Update the plot
    line.set_data(x.value[0, :], x.value[1, :])
    return line,

ani = animation.FuncAnimation(fig, update, frames=np.linspace(0, 2*np.pi, 500), blit=False, repeat=True)  # Looping animation for continuous following without blitting
plt.show()
