# test_trained.py
import time
from env import TurtleBotEnv
from stable_baselines3 import PPO

POLICY_HZ  = 100
N_EPISODES = 40

# Load the best model saved by EvalCallback
# or replace with any checkpoint e.g. "checkpoints/turtlebot_ppo_340000_steps.zip"
model = PPO.load("/home/sostika/my_py/check/turtlebot_final_1.zip")

env = TurtleBotEnv(render=True)

try:
    for ep in range(N_EPISODES):
        obs, _ = env.reset()
        ep_reward = 0.0
        steps     = 0
        done      = False

        while not done:
            action, _ = model.predict(obs, deterministic=True)
            obs, reward, terminated, truncated, _ = env.step(action)
            ep_reward += reward
            steps     += 1
            done       = terminated or truncated

            time.sleep(1.0 / POLICY_HZ)   # real-time playback

        result = "GOAL" if terminated and reward > 0 else "COLLISION" if terminated else "TIMEOUT"
        print(f"Episode {ep+1:02d}: {result:10s}  steps={steps:5d}  reward={ep_reward:.2f}")

finally:
    env.close()