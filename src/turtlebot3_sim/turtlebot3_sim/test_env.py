from stable_baselines3 import PPO
from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.callbacks import CheckpointCallback
from .myown_env import TurtleBotEnv

env = TurtleBotEnv()
env = Monitor(env)

checkpoint_callback = CheckpointCallback(
    save_freq=10000,
    save_path="./checkpoints_navigation_project/",
    name_prefix="turtlebot_nav",
    verbose=1,
)

model = PPO(
    policy       = "MlpPolicy",
    env          = env,
    learning_rate= 3e-4,
    n_steps      = 4096,        # steps per rollout (larger = more stable)
    batch_size   = 256,         # minibatch size
    n_epochs     = 10,          # gradient passes per rollout
    gamma        = 0.99,        # discount
    gae_lambda   = 0.95,
    clip_range   = 0.2,
    clip_range_vf= None,        # no VF clipping – prevents value collapse
    ent_coef     = 0.01,        # entropy bonus – keeps policy exploring
    vf_coef      = 0.5,
    max_grad_norm= 0.5,
    verbose      = 1,
    tensorboard_log = "Autonomousnavigations",
    policy_kwargs = dict(
        net_arch = [dict(pi=[256, 256], vf=[256, 256])],  # separate actor/critic nets
    ),
)
try:
    model.learn(
        total_timesteps=2_000_000,
        callback=checkpoint_callback,
    )
except KeyboardInterrupt:
    print("Training interrupted — saving current model...")
finally:
    steps = model.num_timesteps
    model.save(f"robot_navigation_{steps}steps")
    print(f"Model saved to sostika_{steps}steps.zip")
