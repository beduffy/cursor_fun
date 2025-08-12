import time

import numpy as np
import mujoco.viewer
import mujoco
from robot_descriptions import g1_description, g1_mj_description, go2_mj_description

# TODO this is a strange different viewer API, not needed usually? or?
# TODO get rest from https://github.com/robot-descriptions/robot_descriptions.py?tab=readme-ov-file#humanoids and have all of them together

# Create ground plane XML with fixed position
ground_xml = """
<mujoco>
    <worldbody>
        <geom name="ground" type="plane" size="10 10 0.1" rgba="0.5 0.5 0.5 1" pos="0 0 0" contype="1" conaffinity="1"/>
    </worldbody>
</mujoco>
"""

# Load models
model = mujoco.MjModel.from_xml_path(g1_mj_description.MJCF_PATH)
# model = mujoco.MjModel.from_xml_path(panda_mj_description.MJCF_PATH)
# model2 = mujoco.MjModel.from_xml_path(unitree_go2.MJCF_PATH)

# Add ground plane to model
ground_model = mujoco.MjModel.from_xml_string(ground_xml)

# Get the current number of geoms
n_geoms = model.geom_pos.shape[0]

# Create new arrays with one extra slot for the ground
new_geom_pos = np.zeros((n_geoms + 1, 3))  # Add one extra slot
new_geom_size = np.zeros((n_geoms + 1, 3))
new_geom_rgba = np.zeros((n_geoms + 1, 4))
new_geom_type = np.zeros(n_geoms + 1, dtype=model.geom_type.dtype)

# Copy existing data to the new arrays, shifted by 1 to keep ground at index 0
new_geom_pos[1:] = model.geom_pos
new_geom_size[1:] = model.geom_size
new_geom_rgba[1:] = model.geom_rgba
new_geom_type[1:] = model.geom_type

# Add ground properties to first slot
new_geom_pos[0] = ground_model.geom_pos[0]
new_geom_size[0] = ground_model.geom_size[0]
new_geom_rgba[0] = ground_model.geom_rgba[0]
new_geom_type[0] = ground_model.geom_type[0]

# Assign new arrays back to model
model.geom_pos = new_geom_pos
model.geom_size = new_geom_size
model.geom_rgba = new_geom_rgba
model.geom_type = new_geom_type

# TODO
# Traceback (most recent call last):
#   File "mujoco_all_models.py", line 50, in <module>
#     model.geom_pos = new_geom_pos
# ValueError: could not broadcast input array from shape (90,3) into shape (89,3)

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
