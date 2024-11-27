import gymnasium as gym
import numpy as np

from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import DummyVecEnv, VecNormalize

# Create Mujoco Humanoid environment
env = gym.make("Humanoid-v4", render_mode="human")

# Wrap environment for RL training
env = DummyVecEnv([lambda: env])
env = VecNormalize(env, norm_obs=True, norm_reward=True, clip_obs=10.0)

# Create and train PPO model
model = PPO("MlpPolicy", env, verbose=1)
model.learn(total_timesteps=100_000)  # Increased timesteps for more complex environment

# Evaluation loop
vec_env = model.get_env()
obs = vec_env.reset()
print("Initial observation shape:", obs.shape)

# Get first action to print its shape
initial_action, _ = model.predict(obs, deterministic=True)
print("Action shape:", initial_action.shape)

for i in range(30000):
    action, _states = model.predict(obs, deterministic=True)
    obs, reward, done, info = vec_env.step(action)
    vec_env.render()
    # VecEnv resets automatically
    # AHH below only happens when 1000 frames are over. ok. or after a very long time
    if done:
        print('done, i:', i)
        print('reward:', reward)
        obs = env.reset()

env.close()