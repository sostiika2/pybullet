import rclpy
from stable_baselines3 import PPO
from .turtlebot_env import TurtleBotEnv


env = TurtleBotEnv()
model = PPO.load("ppo_turtlebot5", env=env)

obs, _ = env.reset()

done = False
while not done and rclpy.ok():
    action, _ = model.predict(obs, deterministic=True)
    obs, reward, terminated, truncated, info = env.step(action)

    # Check if the robot reached the goal
    if terminated or truncated:
        done = True

# Once done, stop the robot and close ROS2 properly
env.close()
print("Navigation finished, robot stopped.")