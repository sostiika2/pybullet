from stable_baselines3 import PPO
from env.rl_env import TurtlebotEnv

env = TurtlebotEnv()

model = PPO.load("turtlebot_navigation")

obs,_ = env.reset()

while True:

    action,_ = model.predict(obs)

    obs,reward,done,_,_ = env.step(action)

    if done:
        obs,_ = env.reset()