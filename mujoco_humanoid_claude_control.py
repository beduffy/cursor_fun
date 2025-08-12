import mujoco
import mujoco.viewer
import numpy as np
import time


def pid_controller(target, current, prev_error, integral, kp=1.0, ki=0.01, kd=0.1):
    error = target - current
    integral += error
    derivative = error - prev_error
    return kp * error + ki * integral + kd * derivative, error, integral


def run_humanoid():
    # Load the humanoid model
    # model = mujoco.MjModel.from_xml_path('humanoid.xml')
    model = mujoco.MjModel.from_xml_path('../mujoco/model/humanoid/humanoid.xml')
    data = mujoco.MjData(model)

    # Initialize the viewer
    with mujoco.viewer.launch_passive(model, data) as viewer:
        # Set initial pose
        data.qpos[2] = 1.4  # Increased initial height for better stability
        
        # Control parameters
        target_velocity = np.array([0.3, 0.0, 0.0])  # Reduced target velocity
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
            
            # Apply different control to different joint groups
            hip_joints = []
            knee_joints = []
            ankle_joints = []
            
            for i in range(model.nu):
                name = model.joint(i).name
                if 'hip' in name:
                    hip_joints.append(i)
                elif 'knee' in name:
                    knee_joints.append(i)
                elif 'ankle' in name:
                    ankle_joints.append(i)
            
            # Apply balanced control to different joint groups
            data.ctrl[hip_joints] = 0.5 * control[0]  # Reduced hip control
            data.ctrl[knee_joints] = 0.2  # Constant small activation for stability
            data.ctrl[ankle_joints] = 0.1  # Small ankle torque for balance
            
            # Step simulation
            mujoco.mj_step(model, data)
            
            # Update viewer
            viewer.sync()
            time.sleep(model.opt.timestep)


if __name__ == "__main__":
    run_humanoid()
