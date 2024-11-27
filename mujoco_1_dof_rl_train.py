import numpy as np
import mujoco
import mujoco.viewer
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
                <motor joint="x_joint" name="x_motor"/>
                <motor joint="y_joint" name="y_motor"/> 
            </actuator>
        </mujoco>
        """
        
        # Load model and create data
        self.model = mujoco.MjModel.from_xml_string(self.model_xml)
        self.data = mujoco.MjData(self.model)
        
        # Define action and observation spaces
        self.action_space = spaces.Box(low=-1, high=1, shape=(2,))
        self.observation_space = spaces.Box(low=-np.inf, high=np.inf, shape=(4,))
        
        # Create viewer
        self.viewer = None
        
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
        self.data.ctrl[:] = action * 0.5  # Scale down actions
        
        # Simulate one step
        mujoco.mj_step(self.model, self.data)
        
        # Get current position
        pos = self.data.qpos
        
        # Calculate reward based on distance to target
        target = np.array([1.0, 1.0])
        dist = np.linalg.norm(pos - target)
        reward = -dist  # Negative distance as reward
        
        # Check if done
        done = dist < 0.1  # Success if within 0.1 units of target
        truncated = False
        
        return self._get_obs(), reward, done, truncated, {}
        
    def render(self):
        if self.viewer is None:
            self.viewer = mujoco.viewer.launch(self.model, self.data)
        if self.viewer.is_running():
            self.viewer.sync()
            return True
        return False
        
    def close(self):
        if self.viewer is not None:
            self.viewer.close()


# Create and wrap the environment
env = Simple2DReachEnv()
env = DummyVecEnv([lambda: env])

# Create and train PPO model
# model = PPO("MlpPolicy", env, verbose=1)
# model.learn(total_timesteps=50_000)

# Test the trained model
obs = env.reset()[0]  # Correctly unpack the vectorized environment reset
time.sleep(1)  # Give viewer time to initialize

def render_environment(env):
    if env.viewer is None:
        env.viewer = mujoco.viewer.launch(env.model, env.data)
    if env.viewer.is_running():
        env.viewer.sync()
        return True
    return False

while True:
    # action, _ = model.predict(obs, deterministic=True)
    action = env.action_space.sample()  # Generate random action
    print('action:', action)  # Debugging print
    
    # Apply the action and step the environment
    obs, rewards, dones, info = env.step(action)  # DummyVecEnv returns 4 values, not 5
    obs = obs[0]  # Get the observation for the single environment
    done = dones[0]  # Get the done flag for the single environment
    
    # Add a small delay to make the movement visible
    time.sleep(0.01)
    
    # Render and check if the viewer is still running
    if not render_environment(env.envs[0]):
        print('viewer closed')  # Debugging print
        break
    
    # Print the observation and reward for debugging
    print('obs:', obs, 'reward:', rewards, 'done:', done)
    
    if done:  # No need to check truncated as it's included in 'info'
        print('resetting')  # Debugging print
        obs = env.reset()[0]  # Correctly unpack the reset
        time.sleep(0.1)  # Give a slight pause after reset

env.close()
