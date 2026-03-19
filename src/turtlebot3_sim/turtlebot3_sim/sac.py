from stable_baselines3 import SAC
import gymnasium as gym
from stable_baselines3 import PPO
from stable_baselines3.common.env_checker import check_env
from .myown_env import TurtleBotEnv

# Create environment
env = TurtleBotEnv()
check_env(env)  # sanity check
model = SAC(
    "MlpPolicy",
    env,
    learning_rate=3e-4,
    buffer_size=100_000,
    batch_size=256,
    tau=0.005,
    gamma=0.99,
    verbose=1,
    tensorboard_log="./tensorboard_min/",
)
model.learn(total_timesteps=500_000)

# Save model
model.save("ppo_turtlebot3_24lidar")
print("Training complete!")