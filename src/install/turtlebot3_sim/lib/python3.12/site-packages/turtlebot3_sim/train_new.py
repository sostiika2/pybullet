import gymnasium as gym
from stable_baselines3 import PPO
from stable_baselines3.common.env_checker import check_env
from .myown_env import TurtleBotEnv

# Create environment
env = TurtleBotEnv()
check_env(env)  # sanity check

model = PPO(
    "MlpPolicy",
    env,
    verbose=1,
    learning_rate=1e-4,
    ent_coef=0.001,
    n_steps=2048,
    batch_size=64,
    gamma=0.99,
    gae_lambda=0.95,
    clip_range=0.2,
    tensorboard_log="./tensorboard_new/",
)

# Train PPO
model.learn(total_timesteps=2000_000)

# Save model
model.save("ppo_turtlebot3_24lidar")
print("Training complete!")