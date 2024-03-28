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

# # Initial state in the middle of the track
# x0 = np.array([track_length/2, track_length/(2*np.pi), 0])  # initial position at the center of the track, added orientation

# Initial state to the left of the track
x0 = np.array([0, track_length/(2*np.pi), np.pi/2])  # initial position to the left of the track, added orientation

# State update matrix
A = np.array([[1, 0, dt],
              [0, 1, 0],
              [0, 0, 1]])
B = np.array([[0.5 * dt**2, 0],
              [dt, 0],
              [0, dt]])  # Added control input for rotation

# MPC setup
x = cp.Variable((3, N+1))  # Adjusted for added orientation state
u = cp.Variable((2, N))  # Adjusted for added control input for rotation

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

# For displaying distance to target
distance_text = ax.text(0.02, 0.95, '', transform=ax.transAxes)

def update(i):
    global x0, target_index, target_position
    # Check if the car has reached the current target position
    distance_to_target = np.linalg.norm(x0[:2] - target_position)  # Adjusted for 3D state
    if distance_to_target < 1.0:  # Threshold to consider as reached
        target_index = (target_index + 1) % len(theta)  # Move to the next target
        target_position = np.array([track_x[target_index], track_y[target_index]])
    
    target_velocity = 0.0   # target velocity

    # Objective and constraints
    objective = cp.Minimize(sum(cp.square(x[0, k+1] - target_position[0]) + cp.square(x[1, k+1] - target_position[1]) for k in range(N)))
    constraints = [x[:, 0] == x0]
    for k in range(N):
        constraints += [x[:, k+1] == A @ x[:, k] + B @ u[:, k]]
        constraints += [cp.abs(u[0, k]) <= 1.0]  # acceleration limit
        constraints += [cp.abs(u[1, k]) <= np.pi/4]  # rotation limit, added constraint for rotation control

    # Solve MPC
    prob = cp.Problem(objective, constraints)
    prob.solve()

    # Update the car's position
    x0 = x.value[:, 1]

    # Update the plot
    line.set_data(x.value[0, :], x.value[1, :])
    ax.scatter(*target_position, color='red', s=100, label='Target', zorder=5)  # Highlight target position
    distance_text.set_text(f'Distance to target: {distance_to_target:.2f}')
    return line, distance_text

ani = animation.FuncAnimation(fig, update, frames=np.linspace(0, 2*np.pi, 500), blit=False, repeat=True)  # Looping animation for continuous following without blitting
plt.show()
