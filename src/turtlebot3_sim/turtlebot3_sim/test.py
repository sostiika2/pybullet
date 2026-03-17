import rclpy
from stable_baselines3 import PPO
from .myown_env import TurtleBotEnv
import numpy as np
import math

env = TurtleBotEnv()
model = PPO.load("checkpoints_navigation_project/turtlebot_nav_530000_steps.zip", env=env)

obs, _ = env.reset()

done = False
step_count = 0

while not done and rclpy.ok():
        # Get action from the model
    action, _ = model.predict(obs, deterministic=True)

        # Take a step in the environment
    obs, reward, terminated, truncated, info = env.step(action)

        # Optional: extract velocity if your env returns it in obs or info
    linear_vel = obs[0]   # assuming obs[0] = linear velocity
    angular_vel = obs[1]  # assuming obs[1] = angular velocity
    distance_to_goal = obs[3]  # as per your reward function

        
    step_count += 1
    print(step_count)

        # Check if the robot reached the goal
    if terminated or truncated:
        done = True

env.close()


 
# # print("Navigation finished, robot stopped.")

# from stable_baselines3 import PPO
# from .myown_env import TurtleBotEnv
# import math
# import numpy as np

# env = TurtleBotEnv()



# obs, _ = env.reset()


# total = 0
# for i in range(200):
#     action = env.action_space.sample()
#     obs, reward, done, truncated, _ = env.step(action)
#     print(f"step {i:3d}: reward={reward:8.4f}  done={done}")
#     total += reward
#     if done or truncated:
#         break

# print(f"\ntotal reward: {total:.2f}")
# print(f"steps taken: {i+1}")