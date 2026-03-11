import gymnasium as gym
from stable_baselines3 import PPO
from stable_baselines3.common.env_checker import check_env
from .turtlebot_env import TurtleBotEnv  # your environment file

# -------------------------------
# 1. Create environment
# -------------------------------
env = TurtleBotEnv()

# Optional: check if environment follows Gym API
check_env(env, warn=True)

# -------------------------------
# 2. Create PPO model
# -------------------------------
model = PPO(
    "MlpPolicy",       # Fully connected network
    env,
    verbose=1,         # Show training info
    learning_rate=3e-4,
    n_steps=2048,
    batch_size=64,
    ent_coef=0.01,
    gamma=0.99,
    clip_range=0.2
)

# -------------------------------
# 3. Train the model
# -------------------------------
# You can adjust timesteps depending on how long you want
model.learn(total_timesteps=200_000)

# -------------------------------
# 4. Save the model
# -------------------------------
model.save("ppo_turtlebot5")

# -------------------------------
# 5. Test trained model
# -------------------------------
obs, _ = env.reset()
for _ in range(500):
    action, _states = model.predict(obs, deterministic=True)
    obs, reward, done, truncated, info = env.step(action)
    if done:
        obs, _ = env.reset()

env.close()

# import gymnasium as gym
# from stable_baselines3 import PPO
# from .turtlebot_env import TurtleBotEnv  # Your custom TurtleBot environment

# # -------------------------------
# # 1. Create the environment
# # -------------------------------
# env = TurtleBotEnv()

# # -------------------------------
# # 2. Load the trained PPO model
# # -------------------------------
# model = PPO.load("ppo_turtlebot", env=env)  # Important: pass env here for evaluation

# # -------------------------------
# # 3. Run navigation using the trained model
# # -------------------------------
# obs, _ = env.reset()
# while True:
#     action, _states = model.predict(obs, deterministic=True)
#     obs, reward, done, truncated, info = env.step(action)
    
#     # Optional: stop if goal reached
#     if done:
#         print("Reached the goal or episode terminated!")
#         obs, _ = env.reset()