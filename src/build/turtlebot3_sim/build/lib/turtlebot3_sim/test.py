# # import rclpy
# # from stable_baselines3 import PPO
# # from .myown_env import TurtleBotEnv
# # import numpy as np
# # import math

# # env = TurtleBotEnv()
# # model = PPO.load("ppo_turtlebot3_24lidar.zip.zip", env=env)

# # obs, _ = env.reset()

# # done = False
# # step_count = 0

# # while not done and rclpy.ok():
# #         # Get action from the model
# #     action, _ = model.predict(obs, deterministic=True)

# #         # Take a step in the environment
# #     obs, reward, terminated, truncated, info = env.step(action)

# #         # Optional: extract velocity if your env returns it in obs or info
# #     linear_vel = obs[0]   # assuming obs[0] = linear velocity
# #     angular_vel = obs[1]  # assuming obs[1] = angular velocity
# #     distance_to_goal = obs[3]  # as per your reward function

        
# #     step_count += 1
# #     print(step_count)

# #         # Check if the robot reached the goal
# #     if terminated or truncated:
# #         done = True

# # env.close()


 
# # # # print("Navigation finished, robot stopped.")

# # # from stable_baselines3 import PPO
# # # from .myown_env import TurtleBotEnv
# # # import math
# # # import numpy as np

# # # env = TurtleBotEnv()



# # # obs, _ = env.reset()


# # # total = 0
# # # for i in range(200):
# # #     action = env.action_space.sample()
# # #     obs, reward, done, truncated, _ = env.step(action)
# # #     print(f"step {i:3d}: reward={reward:8.4f}  done={done}")
# # #     total += reward
# # #     if done or truncated:
# # #         break

# # # print(f"\ntotal reward: {total:.2f}")
# # # print(f"steps taken: {i+1}")
# import rclpy
# from stable_baselines3 import PPO
# from turtlebot_env import TurtleBotEnv

# env = TurtleBotEnv()
# model = PPO.load("/home/sostika/my_py/src/turtlebot3_sim/turtlebot3_sim/models_enhanced_v2/ppo_enhanced_v2.zip", env=env)

# obs, _ = env.reset()

# done = False
# step_count = 0
# while not done and rclpy.ok():
#     # Get action from the model
#     action, _ = model.predict(obs, deterministic=True)

#     # Take a step in the environment
#     obs, reward, terminated, truncated, info = env.step(action)

#     # Optional: extract velocity if your env returns it in obs or info
#     linear_vel = obs[0]   # assuming obs[0] = linear velocity
#     angular_vel = obs[1]  # assuming obs[1] = angular velocity
#     distance_to_goal = obs[3]  # as per your reward function

#     # Print info
#     print(f"Step {step_count}:")
#     print(f"  Action taken: {action}")
#     print(f"  Reward: {reward}")
#     print(f"  Linear velocity: {linear_vel}, Angular velocity: {angular_vel}")
#     print(f"  Distance to goal: {distance_to_goal}")
#     print(f"  Terminated: {terminated}, Truncated: {truncated}")
#     print("-" * 30)

#     step_count += 1

#     # Check if the robot reached the goal
#     if terminated or truncated:
#         done = True

# # Stop and close
# env.close()
# print("Navigation finished, robot stopped.")


import rclpy
from stable_baselines3 import PPO
from turtlebot_env import TurtleBotEnv

# Initialize environment and load model
env = TurtleBotEnv()
model_path = "/home/sostika/my_py/turtlebot_final_1.zip"
# model_path = "/home/sostika/my_py/turtlebot_final.zip"
model = PPO.load(model_path, env=env)

num_episodes = 10  # Set how many episodes you want to test

for episode in range(1, num_episodes + 1):
    obs, _ = env.reset()
    done = False
    step_count = 0
    episode_reward = 0
    
    print(f"\n--- Starting Episode {episode} ---")

    while not done and rclpy.ok():
        # Get action (deterministic=True is best for testing/evaluation)
        action, _ = model.predict(obs, deterministic=True)


        # Take a step
        obs, reward, terminated, truncated, info = env.step(action)
        
        episode_reward += reward
        step_count += 1
        print(action)

        # Optional: Print every 100 steps to avoid flooding the terminal
        if step_count % 100 == 0:
            print(f"Episode {episode} | Step {step_count} | Reward so far: {episode_reward:.2f}")

        if terminated or truncated:
            # print(f"--- Episode {episode} Finished ---")
            # print(f"Total Steps: {step_count}")
            # print(f"Total Reward: {episode_reward:.2f}")
            # print(f"Reason: {'Goal Reached/Collision' if terminated else 'Timeout'}")
            done = True

# Clean up
env.close()
print("\nAll test episodes finished.")