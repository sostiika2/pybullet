# train.py
from stable_baselines3 import PPO
from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.callbacks import CheckpointCallback
from .turtlebot_env import TurtleBotEnv

env = TurtleBotEnv()
env = Monitor(env)

checkpoint_callback = CheckpointCallback(
    save_freq=10000,
    save_path="./checkpoints4",
    name_prefix="turtlebot_nav",
    verbose=1,
)

model = PPO(
        "MlpPolicy",
        env,
        verbose=1,
        learning_rate=3e-4,
        n_steps=4096,
        batch_size=64,
        n_epochs=10,
        gamma=0.99,
        gae_lambda=0.95,
        clip_range=0.2,
        ent_coef=0.001,
        tensorboard_log="./new_appraoch4"
    )

try:
    model.learn(
        total_timesteps=2_000_000,
        reset_num_timesteps=True,
        callback=checkpoint_callback,
    )
except KeyboardInterrupt:
    print("Training interrupted — saving...")
finally:
    steps = model.num_timesteps
    model.save(f"turtlebot_nav_{steps}steps_new")
    print(f"Saved turtlebot_nav_{steps}steps_new.zip")