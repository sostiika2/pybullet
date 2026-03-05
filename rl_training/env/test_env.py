from rl_env import TurtlebotEnv
import numpy as np

env = TurtlebotEnv()

obs,_ = env.reset()

for i in range(1000):

    action = np.array([0.2,0.0])

    obs,reward,done,_,_ = env.step(action)

    print("reward:",reward)

    if done:
        obs,_ = env.reset()