import rclpy
from stable_baselines3 import PPO
from .turtlebot_env import TurtleBotEnv

env = TurtleBotEnv()
model = PPO.load("ppo_turtlebot6", env=env)

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

    # Print info
    print(f"Step {step_count}:")
    print(f"  Action taken: {action}")
    print(f"  Reward: {reward}")
    print(f"  Linear velocity: {linear_vel}, Angular velocity: {angular_vel}")
    print(f"  Distance to goal: {distance_to_goal}")
    print(f"  Terminated: {terminated}, Truncated: {truncated}")
    print("-" * 30)

    step_count += 1

    # Check if the robot reached the goal
    if terminated or truncated:
        done = True

# Stop and close
env.close()
print("Navigation finished, robot stopped.")