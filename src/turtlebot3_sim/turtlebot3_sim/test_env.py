from stable_baselines3 import PPO
from .turtlebot_env import TurtlebotNavEnv

env = TurtlebotNavEnv()

model = PPO(
    "MlpPolicy",
    env,
    verbose=1
)

model.learn(total_timesteps=200000)

model.save("turtlebot_nav_model")