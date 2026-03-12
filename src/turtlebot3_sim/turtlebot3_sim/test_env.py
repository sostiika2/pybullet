import gymnasium as gym
from stable_baselines3 import PPO
from stable_baselines3.common.env_checker import check_env
from .turtlebot_env import TurtleBotEnv

# -------------------------------
# 1. Create environment
# -------------------------------
env = TurtleBotEnv()

check_env(env, warn=True)

# -------------------------------
# 2. Create PPO model
# -------------------------------
model = PPO(
    "MlpPolicy",
    env,
    verbose=1,
    learning_rate=1e-4,
    n_steps=2048,
    batch_size=64,
    ent_coef=0.005,
    gamma=0.99,
    clip_range=0.2,
    tensorboard_log="./ppo_turtlebot_tensorboard/"
)

# -------------------------------
# 3. Train
# -------------------------------
model.learn(
    total_timesteps=700_000,
    tb_log_name="turtlebot_run"
)

# -------------------------------
# 4. Save model
# -------------------------------
model.save("ppo_turtlebot6")

env.close()