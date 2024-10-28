import mujoco
import mujoco.viewer
import numpy as np
import time


def pid_controller(target, current, prev_error, integral, kp=100.0, ki=0.1, kd=10.0):
    error = target - current
    integral += error
    derivative = error - prev_error
    return kp * error + ki * integral + kd * derivative, error, integral


def run_humanoid():
    # Load the humanoid model
    # model = mujoco.MjModel.from_xml_path('humanoid.xml')
    model = mujoco.MjModel.from_xml_path('../mujoco//model/humanoid/humanoid.xml')
    data = mujoco.MjData(model)

    # Initialize the viewer
    with mujoco.viewer.launch_passive(model, data) as viewer:
        # Set initial pose
        data.qpos[2] = 1.0  # Height above ground
        
        # Control parameters
        target_velocity = np.array([1.0, 0.0, 0.0])  # Forward velocity
        prev_error = np.zeros(3)
        integral = np.zeros(3)
        
        # Simulation loop
        while viewer.is_running():
            # Get current state
            current_velocity = data.qvel[:3]
            
            # Compute control action using PID
            control, error, integral = pid_controller(
                target_velocity, 
                current_velocity,
                prev_error,
                integral
            )
            prev_error = error
            
            # Apply control to hip joints for forward motion
            hip_joints = [i for i in range(model.nu) if 'hip' in model.actuator_names[i]]
            data.ctrl[hip_joints] = control[0]
            
            # Step simulation
            mujoco.mj_step(model, data)
            
            # Update viewer
            viewer.sync()
            time.sleep(model.opt.timestep)


if __name__ == "__main__":
    run_humanoid()
