import os

# Patch and register pybullet envs
import rl_zoo3.gym_patches
# import pybullet_envs

from stable_baselines3 import PPO
from stable_baselines3.common.env_util import make_vec_env
from stable_baselines3.common.vec_env import VecNormalize, DummyVecEnv

# env = make_vec_env(
#     "HalfCheetahBulletEnv-v0",
#     env_kwargs=dict(apply_api_compatibility=True),
#     n_envs=1,
# )

# Pybullet doesn't support Gymnasium yet
import gym as gym26
# TODO BELOW IS NOT WORKING
env = gym26.make("HalfCheetahBulletEnv-v0", apply_api_compatibility=True)

env = DummyVecEnv([lambda: env])
env = VecNormalize(env, norm_obs=True, norm_reward=True, clip_obs=10.0)

model = PPO("MlpPolicy", env, verbose=1)
model.learn(total_timesteps=2000)

# Don't forget to save the VecNormalize statistics when saving the agent
log_dir = "/tmp/"
model.save(log_dir + "ppo_halfcheetah")
stats_path = os.path.join(log_dir, "vec_normalize.pkl")
env.save(stats_path)

# Load the agent
model = PPO.load(log_dir + "ppo_halfcheetah")

# Load the saved statistics
env = make_vec_env(
    "HalfCheetahBulletEnv-v0",
    env_kwargs=dict(apply_api_compatibility=True),
    n_envs=1,
)
env = VecNormalize.load(stats_path, env)
#  do not update them at test time
env.training = False
# reward normalization is not needed at test time
env.norm_reward = False

from stable_baselines3.common.evaluation import evaluate_policy

mean_reward, std_reward = evaluate_policy(model, env)

print(f"Mean reward = {mean_reward:.2f} +/- {std_reward:.2f}")