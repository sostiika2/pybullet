from stable_baselines3 import PPO
from env.rl_env import TurtlebotEnv

env = TurtlebotEnv()

model = PPO(
    "MlpPolicy",
    env,
    verbose=1,
    tensorboard_log="./tensorboard/"
)

model.learn(total_timesteps=500000)

model.save("turtlebot_navigation")