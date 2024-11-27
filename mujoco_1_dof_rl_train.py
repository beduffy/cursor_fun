import numpy as np
import mujoco
import mujoco.viewer
from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import DummyVecEnv
import gymnasium as gym
from gymnasium import spaces


# Define a custom environment
class Simple2DReachEnv(gym.Env):
    def __init__(self):
        super().__init__()
        
        # Create a simple 2D point mass model
        self.model_xml = """
        <mujoco>
            <option gravity="0 0 0"/>
            <worldbody>
                <body name="point_mass" pos="0 0 0">
                    <joint name="x_joint" type="slide" axis="1 0 0" pos="0 0 0"/>
                    <joint name="y_joint" type="slide" axis="0 1 0" pos="0 0 0"/>
                    <geom name="point_mass" type="sphere" size="0.1" rgba="1 0 0 1"/>
                </body>
                <site name="target" pos="1 1 0" size="0.1" rgba="0 1 0 1"/>
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
            self.viewer = mujoco.viewer.launch_passive(self.model, self.data)
        
        if self.viewer.is_running():
            self.viewer.sync()
            
    def close(self):
        if self.viewer is not None:
            self.viewer.close()


# Create and wrap the environment
env = Simple2DReachEnv()
env = DummyVecEnv([lambda: env])

# Create and train PPO model
model = PPO("MlpPolicy", env, verbose=1)
model.learn(total_timesteps=50_000)

# Test the trained model
obs = env.reset()
for _ in range(1000):
    action, _ = model.predict(obs, deterministic=True)
    obs, reward, done, info = env.step(action)
    env.render()
    if done:
        obs = env.reset()

env.close()
