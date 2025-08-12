"""
Initial prompt:
Create an MPC algorithm in python and matplotlib and make car follow a circle trajectory. Output certain output to files so you can get feedback on how it is working, then read those files and improve on the code until it works. Then explain everything to me. I don't really understand MPC."

"""

import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import minimize
import csv
import _tkinter

# --- Constants and Model Parameters ---
DT = 0.1  # [s] time tick
L = 2.9  # [m] Wheel base of vehicle
MAX_STEER = np.deg2rad(45.0)  # [rad] maximum steering angle
MAX_SPEED = 50.0 / 3.6  # [m/s] maximum speed
MIN_SPEED = -20.0 / 3.6  # [m/s] minimum speed
MAX_ACCEL = 2.0  # [m/ss] maximum acceleration

# MPC parameters
NX = 4  # State size [x, y, v, yaw]
NU = 2  # Input size [acceleration, steer]
HORIZON = 10  # Prediction horizon
R = np.diag([0.01, 0.01])  # Input cost
Rd = np.diag([0.01, 1.0])  # Input rate cost
Q = np.diag([1.0, 1.0, 0.5, 0.5])  # State cost
Qf = Q  # Final state cost
TARGET_SPEED = 10.0 / 3.6  # [m/s] target speed
CIRCULAR_TRAJECTORY_RADIUS = 10.0  # [m]


# --- Vehicle Model ---
def vehicle_model(state, u, dt):
    """
    Kinematic bicycle model
    state: [x, y, v, yaw]
    u: [acceleration, steer]
    """
    x, y, v, yaw = state
    a, delta = u

    # Apply constraints
    delta = np.clip(delta, -MAX_STEER, MAX_STEER)
    v = np.clip(v, MIN_SPEED, MAX_SPEED)
    a = np.clip(a, -MAX_ACCEL, MAX_ACCEL)


    new_x = x + v * np.cos(yaw) * dt
    new_y = y + v * np.sin(yaw) * dt
    new_yaw = yaw + v / L * np.tan(delta) * dt
    new_v = v + a * dt
    new_v = np.clip(new_v, MIN_SPEED, MAX_SPEED) # ensure speed constraints after update

    return np.array([new_x, new_y, new_v, new_yaw])


# --- Trajectory Generation ---
def generate_circular_trajectory(radius, num_points, target_speed):
    """
    Generates a circular trajectory.
    Returns: x_ref, y_ref, yaw_ref, v_ref
    """
    t = np.linspace(0, 2 * np.pi, num_points)
    x_ref = radius * np.cos(t)
    y_ref = radius * np.sin(t)
    yaw_ref = t + np.pi / 2  # Yaw is tangent to the circle
    v_ref = np.ones(num_points) * target_speed
    return x_ref, y_ref, yaw_ref, v_ref


# --- MPC Controller ---
def mpc_cost_function(u_flat, current_state, ref_trajectory, prev_u):
    """
    Cost function for MPC.
    u_flat: flattened array of control inputs over the horizon [a0, delta0, a1, delta1, ...]
    current_state: current state of the vehicle [x, y, v, yaw]
    ref_trajectory: reference states over the horizon [[x_ref, y_ref, v_ref, yaw_ref], ...]
    prev_u: previous control input [a, delta]
    """
    u = u_flat.reshape(HORIZON, NU)  # Reshape to (HORIZON, NU)
    cost = 0.0
    predicted_states = np.zeros((HORIZON + 1, NX))
    predicted_states[0, :] = current_state

    for i in range(HORIZON):
        # Predict next state
        predicted_states[i+1, :] = vehicle_model(predicted_states[i, :], u[i, :], DT)

        # State cost
        state_error = predicted_states[i+1, :] - ref_trajectory[i, :]
        cost += state_error.T @ Q @ state_error

        # Input cost
        cost += u[i, :].T @ R @ u[i, :]

        # Input rate cost (penalty on change in control input)
        if i > 0:
            cost += (u[i, :] - u[i-1, :]).T @ Rd @ (u[i, :] - u[i-1, :])
        else: # For the first input, compare with previous actual input
             cost += (u[i, :] - prev_u).T @ Rd @ (u[i, :] - prev_u)


    # Final state cost
    final_state_error = predicted_states[HORIZON, :] - ref_trajectory[HORIZON-1, :] # ref_trajectory might be shorter
    cost += final_state_error.T @ Qf @ final_state_error

    return cost


def mpc_control(current_state, ref_trajectory, prev_u):
    """
    Calculates the optimal control input using MPC.
    """
    u_guess = np.tile(prev_u, HORIZON).flatten() # Initial guess for control inputs

    # Bounds for control inputs: [a_min, delta_min, a_min, delta_min, ...]
    bounds = []
    for _ in range(HORIZON):
        bounds.extend([(-MAX_ACCEL, MAX_ACCEL), (-MAX_STEER, MAX_STEER)])

    result = minimize(mpc_cost_function, u_guess,
                      args=(current_state, ref_trajectory, prev_u),
                      method='SLSQP',  # Sequential Least SQuares Programming
                      bounds=bounds,
                      options={'disp': False})

    optimal_u_flat = result.x
    return optimal_u_flat.reshape(HORIZON, NU)[0, :] # Return the first optimal control input


# --- Simulation ---
def run_simulation():
    plt.ion() # Enable interactive plotting

    # Generate reference trajectory
    num_ref_points = 200
    x_ref, y_ref, yaw_ref, v_ref = generate_circular_trajectory(CIRCULAR_TRAJECTORY_RADIUS, num_ref_points, TARGET_SPEED)
    ref_traj_full = np.vstack([x_ref, y_ref, v_ref, yaw_ref]).T

    # Initial state [x, y, v, yaw]
    current_state = np.array([0.0, -CIRCULAR_TRAJECTORY_RADIUS, 0.0, np.pi / 2])
    # current_state = np.array([x_ref[0], y_ref[0], v_ref[0], yaw_ref[0]]) # Start on trajectory

    # Simulation time
    sim_time = 60.0  # seconds
    time_steps = int(sim_time / DT)

    # Log data
    history_time = np.zeros(time_steps)
    history_state = np.zeros((time_steps, NX))
    history_ref_state = np.zeros((time_steps, NX))
    history_control = np.zeros((time_steps, NU))

    # Previous control input (initialized to zero)
    prev_u = np.array([0.0, 0.0])

    # --- Setup Live Plot ---
    fig_live, ax_live = plt.subplots(figsize=(8, 8))
    ax_live.set_title("Live MPC Simulation")
    ax_live.plot(x_ref, y_ref, "r--", label="Reference Trajectory")
    car_body, = ax_live.plot([], [], 'bo', markersize=10, label="Car") # Car body as a blue dot
    # Arrow for heading (adjust length as needed)
    car_heading, = ax_live.plot([], [], 'b-', linewidth=2) 
    car_path, = ax_live.plot([], [], 'b:', label="Car Path") # Car's traced path
    ax_live.set_xlabel("X [m]")
    ax_live.set_ylabel("Y [m]")
    ax_live.axis("equal")
    ax_live.legend()
    ax_live.grid(True)
    # Set fixed limits for the live plot for better visualization if desired
    # ax_live.set_xlim(min(x_ref) - 5, max(x_ref) + 5)
    # ax_live.set_ylim(min(y_ref) - 5, max(y_ref) + 5)

    traced_x, traced_y = [], []

    for i in range(time_steps):
        # Get the reference for the prediction horizon
        # This needs to handle the end of the reference trajectory
        ref_horizon = np.zeros((HORIZON, NX))
        for k in range(HORIZON):
            ref_idx = min(i + k, num_ref_points - 1) # Use the current point or last point if near end
            ref_horizon[k, :] = ref_traj_full[ref_idx, :]


        # Calculate optimal control input
        optimal_u = mpc_control(current_state, ref_horizon, prev_u)

        # Apply control input to the vehicle model
        current_state = vehicle_model(current_state, optimal_u, DT)

        # Log data
        history_time[i] = i * DT
        history_state[i, :] = current_state
        current_ref_idx = min(i, num_ref_points - 1)
        history_ref_state[i, :] = ref_traj_full[current_ref_idx, :]
        history_control[i, :] = optimal_u

        prev_u = optimal_u # Update previous control for next iteration

        # Update live plot data
        car_x, car_y, _, car_yaw = current_state
        traced_x.append(car_x)
        traced_y.append(car_y)

        car_body.set_data(car_x, car_y)
        
        # Calculate heading arrow points
        arrow_length = 1.5 # meters, visual aid
        heading_x_end = car_x + arrow_length * np.cos(car_yaw)
        heading_y_end = car_y + arrow_length * np.sin(car_yaw)
        car_heading.set_data([car_x, heading_x_end], [car_y, heading_y_end])

        car_path.set_data(traced_x, traced_y)

        # Redraw live plot
        try:
            fig_live.canvas.draw()
            fig_live.canvas.flush_events() # Ensure the event loop is processed
            plt.pause(DT/2) # Pause for animation, can be adjusted
        except _tkinter.TclError: # Catch error if window is closed
            print("Live plot window closed. Stopping animation.")
            break # Exit the simulation loop

        if i % 50 == 0: # Print progress
            print(f"Sim step: {i}/{time_steps}, "
                  f"X: {current_state[0]:.2f}, Y: {current_state[1]:.2f}, "
                  f"V: {current_state[2]:.2f}, Yaw: {np.rad2deg(current_state[3]):.2f}, "
                  f"Acc: {optimal_u[0]:.2f}, Steer: {np.rad2deg(optimal_u[1]):.2f}")


    # --- Save data to CSV ---
    log_filename = "mpc_simulation_log.csv"
    with open(log_filename, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['Time',
                         'Car_X', 'Car_Y', 'Car_V', 'Car_Yaw_rad',
                         'Ref_X', 'Ref_Y', 'Ref_V', 'Ref_Yaw_rad',
                         'Control_Accel', 'Control_Steer_rad'])
        for i in range(time_steps):
            writer.writerow([history_time[i],
                             history_state[i, 0], history_state[i, 1], history_state[i, 2], history_state[i, 3],
                             history_ref_state[i, 0], history_ref_state[i, 1], history_ref_state[i, 2], history_ref_state[i, 3],
                             history_control[i, 0], history_control[i, 1]])
    print(f"Simulation data saved to {log_filename}")


    plt.ioff() # Disable interactive mode for final summary plots

    # --- Plotting ---
    plt.figure(figsize=(15, 10))

    # Trajectory plot
    plt.subplot(2, 2, 1)
    plt.plot(x_ref, y_ref, "r--", label="Reference Trajectory")
    plt.plot(history_state[:, 0], history_state[:, 1], "b-", label="Car Trajectory")
    plt.xlabel("X [m]")
    plt.ylabel("Y [m]")
    plt.title("Car Trajectory vs Reference")
    plt.axis("equal")
    plt.legend()
    plt.grid(True)

    # Speed plot
    plt.subplot(2, 2, 2)
    plt.plot(history_time, history_state[:, 2], "b-", label="Car Speed")
    plt.plot(history_time, history_ref_state[:, 2], "r--", label="Reference Speed")
    plt.xlabel("Time [s]")
    plt.ylabel("Speed [m/s]")
    plt.title("Speed Profile")
    plt.legend()
    plt.grid(True)

    # Yaw plot
    plt.subplot(2, 2, 3)
    plt.plot(history_time, np.rad2deg(history_state[:, 3]), "b-", label="Car Yaw")
    plt.plot(history_time, np.rad2deg(history_ref_state[:, 3]), "r--", label="Reference Yaw")
    plt.xlabel("Time [s]")
    plt.ylabel("Yaw [degrees]")
    plt.title("Yaw Profile")
    plt.legend()
    plt.grid(True)

    # Control inputs plot
    plt.subplot(2, 2, 4)
    plt.plot(history_time, history_control[:, 0], "b-", label="Acceleration")
    plt.plot(history_time, np.rad2deg(history_control[:, 1]), "g-", label="Steering Angle (deg)")
    plt.xlabel("Time [s]")
    plt.ylabel("Control Input")
    plt.title("Control Inputs")
    plt.legend()
    plt.grid(True)

    plt.tight_layout()
    plt.show()


if __name__ == '__main__':
    run_simulation()
