import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import minimize

# Define the manipulator parameters
link_lengths = [1.0, 1.0, 1.0, 1.0]  # Lengths of the links

# Forward kinematics
def forward_kinematics(theta):
    x = 0
    y = 0
    total_angle = 0
    points = [(x, y)]
    
    for i in range(len(theta)):
        total_angle += theta[i]
        x += link_lengths[i] * np.cos(total_angle)
        y += link_lengths[i] * np.sin(total_angle)
        points.append((x, y))
    
    return np.array(points)

# Objective function for optimization
def objective_function(theta, target):
    points = forward_kinematics(theta)
    end_effector = points[-1]
    return np.linalg.norm(end_effector - target)  # Minimize distance to target

# Constraints and bounds
def bounds():
    return [(-np.pi, np.pi) for _ in range(len(link_lengths))]  # Joint angle limits

# Inverse Kinematics Solver
def inverse_kinematics(target, initial_guess):
    result = minimize(
        objective_function, 
        initial_guess, 
        args=(target,),
        bounds=bounds(),
        method='SLSQP',  # Adding specific optimization method
        options={
            'ftol': 1e-8,  # Tighter function tolerance
            'maxiter': 1000  # More iterations allowed
        }
    )
    
    if not result.success:
        print(f"Warning: Optimization failed to converge. Final error: {result.fun}")
    
    return result.x

# Visualization
def plot_manipulator(theta, target):
    points = forward_kinematics(theta)
    x, y = zip(*points)
    plt.plot(x, y, '-o', label="Manipulator")
    plt.scatter(target[0], target[1], c='red', label="Target")
    plt.xlim(-sum(link_lengths), sum(link_lengths))
    plt.ylim(-sum(link_lengths), sum(link_lengths))
    plt.gca().set_aspect('equal', adjustable='box')
    plt.legend()
    plt.show()

# Main Loop: Move to multiple targets
targets = [(2.0, 2.0),
           (1.0, 3.0), 
           (-1.0, 2.0),
           (0.0, -1.0),
           (3.0, 1.0),
           (-2.0, 3.0),
           (2.5, -2.0),
           (-3.0, -1.0),
           (0.0, 3.5),
           (1.5, -3.0),
           (-2.5, 1.5),
           (3.0, 2.5),
           (0.0, 0.0),
           (-1.5, -2.5),
           (2.0, -1.0)]
initial_guess = [0.0, 0.0, 0.0, 0.0]

for target in targets:
    solution = inverse_kinematics(target, initial_guess)
    final_error = objective_function(solution, target)
    print(f"Target: {target}, Solution Angles: {solution}")
    print(f"Final position error: {final_error:.6f} units")
    plot_manipulator(solution, target)
    initial_guess = solution
