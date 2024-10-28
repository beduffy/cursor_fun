import time

import numpy as np
import mujoco.viewer
import mujoco
from robot_descriptions import g1_description, g1_mj_description, go2_mj_description


# TODO get rest from https://github.com/robot-descriptions/robot_descriptions.py?tab=readme-ov-file#humanoids and have all of them together
model = mujoco.MjModel.from_xml_path(g1_mj_description.MJCF_PATH)
# model = mujoco.MjModel.from_xml_path(panda_mj_description.MJCF_PATH)
# model2 = mujoco.MjModel.from_xml_path(unitree_go2.MJCF_PATH)

unitree_go2_model = mujoco.MjModel.from_xml_path(go2_mj_description.MJCF_PATH)

data = mujoco.MjData(model)
# data = mujoco.MjData(unitree_go2_model)

# Initialize the viewer
with mujoco.viewer.launch_passive(model, data) as viewer:
    # Set initial pose
    data.qpos[2] = 1.4  # Increased initial height for better stability
    
    # # Control parameters
    # target_velocity = np.array([0.3, 0.0, 0.0])  # Reduced target velocity
    # prev_error = np.zeros(3)
    # integral = np.zeros(3)
    
    # Simulation loop
    while viewer.is_running():
        # Get current state
        current_velocity = data.qvel[:3]
        
        # Step simulation
        mujoco.mj_step(model, data)
        
        # Update viewer
        viewer.sync()
        time.sleep(model.opt.timestep)
