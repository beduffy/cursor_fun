# https://github.com/rohanpsingh/mujoco-python-viewer
import time

import mujoco
import mujoco.viewer

model = mujoco.MjModel.from_xml_path('../mujoco/model/humanoid/humanoid.xml')
data = mujoco.MjData(model)

# create the viewer object
# viewer = mujoco.viewer.MujocoViewer(model, data)

with mujoco.viewer.launch_passive(model, data) as viewer:
    # simulate and render
    for _ in range(10000):
        # if viewer.is_alive:
        mujoco.mj_step(model, data)
        # viewer.render()
        # else:
            # break

        # Update viewer
        viewer.sync()
        # time.sleep(1)
        time.sleep(0.1)

    # close
    viewer.close()