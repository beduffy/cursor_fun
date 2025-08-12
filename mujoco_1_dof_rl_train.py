import mujoco as mj
import numpy as np
from mujoco.glfw import glfw
from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import DummyVecEnv
import gymnasium as gym
from gymnasium import spaces
import time


# Define a custom environment
class Simple2DReachEnv(gym.Env):
    def __init__(self):
        super().__init__()
        
        # Create a simple 2D point mass model with checkerboard floor
        self.model_xml = """
        <mujoco>
            <asset>
                <texture type="2d" name="checkerboard" builtin="checker" rgb1=".2 .3 .4" rgb2=".3 .4 .5" width="300" height="300"/>
                <material name="floor_mat" texture="checkerboard" texrepeat="5 5"/>
            </asset>
            <option gravity="0 0 0"/>
            <worldbody>
                <geom name="floor" type="plane" size="2 2 0.1" material="floor_mat"/>
                <body name="point_mass" pos="0 0 0">
                    <joint name="x_joint" type="slide" axis="1 0 0" pos="0 0 0"/>
                    <joint name="y_joint" type="slide" axis="0 1 0" pos="0 0 0"/>
                    <geom name="point_mass" type="sphere" size="0.1" rgba="1 0 0 1"/>
                </body>
                <body name="target" pos="1 1 0">
                    <geom name="target" type="sphere" size="0.1" rgba="0 1 0 0.5" contype="0" conaffinity="0"/>
                </body>
            </worldbody>
            <actuator>
                <motor joint="x_joint" name="x_motor" gear="5"/>
                <motor joint="y_joint" name="y_motor" gear="5"/> 
            </actuator>
        </mujoco>
        """
        
        # Load model and create data
        self.model = mj.MjModel.from_xml_string(self.model_xml)
        self.data = mj.MjData(self.model)
        
        # Define action and observation spaces
        self.action_space = spaces.Box(low=-1, high=1, shape=(2,))
        self.observation_space = spaces.Box(low=-np.inf, high=np.inf, shape=(4,))
        
        # Create viewer
        self.window = None
        self.context = None
        self.scene = None
        self.cam = mj.MjvCamera()
        self.opt = mj.MjvOption()
        
    def reset(self, seed=None):
        super().reset(seed=seed)
        # Reset point mass to origin
        self.data.qpos[:] = 0
        self.data.qvel[:] = 0
        return self._get_obs(), {}
        
    def _get_obs(self):
        # Return position and velocity
        return np.concatenate([self.data.qpos, self.data.qvel])
        
    def step(self, action):
        # Apply action (motor controls)
        self.data.ctrl[:] = action * 5.0  # Increase action scaling for more movement
        
        # Simulate one step
        mj.mj_step(self.model, self.data)
        
        # Get current position
        pos = self.data.qpos
        
        # Calculate reward based on distance to target
        target = np.array([1.0, 1.0])
        dist = np.linalg.norm(pos - target)
        reward = -dist  # Negative distance as reward
        
        # Check if done
        done = dist < 0.1  # Success if within 0.1 units of target
        truncated = False
        
        # Debugging print to check position
        print('Position:', pos, 'Distance to target:', dist)
        
        return self._get_obs(), reward, done, truncated, {}
        
    def render(self):
        if self.window is None:
            if not glfw.init():
                raise Exception("GLFW could not be initialized!")
            self.window = glfw.create_window(640, 480, "MuJoCo Simulation", None, None)
            glfw.make_context_current(self.window)
            self.context = mj.MjrContext(self.model, mj.mjtFontScale.mjFONTSCALE_150)
            self.scene = mj.MjvScene(self.model, maxgeom=1000)
        
        if glfw.window_should_close(self.window):
            return False
        
        # get framebuffer viewport
        viewport_width, viewport_height = glfw.get_framebuffer_size(self.window)
        viewport = mj.MjrRect(0, 0, viewport_width, viewport_height)

        # Update scene and render
        mj.mjv_updateScene(self.model, self.data, self.opt, None, self.cam, mj.mjtCatBit.mjCAT_ALL.value, self.scene)
        mj.mjr_render(viewport, self.scene, self.context)

        # swap OpenGL buffers (blocking call due to v-sync)
        glfw.swap_buffers(self.window)

        # process pending GUI events, call GLFW callbacks
        glfw.poll_events()
        
        return True
        
    def close(self):
        if self.window is not None:
            glfw.terminate()
            
def main():
    env = Simple2DReachEnv()
    obs = env.reset()[0]
    print('Initial observation:', obs)
    
    try:
        while True:
            action = env.action_space.sample()
            print('action:', action)
            obs, reward, done, truncated, info = env.step(action)
            print('obs:', obs, 'reward:', reward, 'done:', done)
            
            if not env.render():
                print('viewer closed')
                break
            
            if done:
                print('resetting')
                obs = env.reset()[0]
                time.sleep(0.1)
    finally:
        env.close()

if __name__ == "__main__":
    main()
