import numpy as np
import cvxpy as cp
import matplotlib.pyplot as plt
import matplotlib.animation as animation

# System parameters
dt = 0.1  # time step
N = 20    # prediction horizon
track_length = 100.0  # length of the track for demonstration

# Track definition (simple straight line for this example)
track = np.linspace(0, track_length, 500)

# Initial state
x0 = np.array([0.0, 0.0])  # initial position and velocity

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
line, = ax.plot([], [], 'o-', lw=2)
target_line, = ax.plot(track, np.zeros_like(track), 'k--', label='Track')
ax.set_xlim(0, track_length)
ax.set_ylim(-5, 5)
ax.set_xlabel('Track position')
ax.set_ylabel('Lateral position')
ax.legend()

def update(i):
    global x0
    # Define the target position dynamically as the car moves along the track
    target_position = x0[0] + 10.0  # target position is always 10 units ahead of the current position
    target_velocity = 0.0   # target velocity

    # Objective and constraints
    objective = cp.Minimize(sum(cp.square(x[0, k+1] - target_position) + cp.square(x[1, k+1] - target_velocity) + cp.square(u[0, k]) for k in range(N)))
    constraints = [x[:, 0] == x0]
    for k in range(N):
        constraints += [x[:, k+1] == A @ x[:, k] + B @ u[:, k]]
        constraints += [cp.abs(u[0, k]) <= 1.0]  # acceleration limit

    # Solve MPC
    prob = cp.Problem(objective, constraints)
    prob.solve()

    # Update the car's position
    x0 = x.value[:, 1]

    # Update the plot
    line.set_data(x.value[0, :], np.zeros_like(x.value[0, :]))
    return line,

ani = animation.FuncAnimation(fig, update, frames=range(200), blit=True)
plt.show()
