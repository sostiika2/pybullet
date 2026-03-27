"""
train.py  -  Vectorized PPO training for TurtleBot3 navigation.

Usage:
    python train.py                  # train with 8 headless envs
    python train.py --envs 4         # fewer envs on a weaker machine
    python train.py --render         # show one GUI env while training
    python train.py --eval           # run a visual eval of best_model
    python train.py --eval --model checkpoints/turtlebot_ppo_340000_steps
"""

import argparse
import os
import glob
import time

from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import SubprocVecEnv, DummyVecEnv, VecMonitor
from stable_baselines3.common.callbacks import CheckpointCallback, EvalCallback

from env import TurtleBotEnv


# ── Env factory ───────────────────────────────────────────────────────────────

def make_env(rank: int, render: bool = False):
    """Return a callable that creates one TurtleBotEnv."""
    def _init():
        return TurtleBotEnv(render=render)
    return _init


# ── Checkpoint helpers ────────────────────────────────────────────────────────

def get_latest_checkpoint(path: str = "coef0/checkpoints/"):
    """
    Scan checkpoints/ for turtlebot_ppo_*_steps.zip files and
    return the path of the one with the highest step count.
    Returns None if no checkpoints exist yet.
    """
    pattern = os.path.join(path, "turtlebot_ppo_*_steps.zip")
    checkpoints = glob.glob(pattern)
    if not checkpoints:
        return None
    # Sort by the step number embedded in the filename
    checkpoints.sort(key=lambda f: int(f.split("_steps")[0].split("_")[-1]))
    return checkpoints[-1]


# ── Training ──────────────────────────────────────────────────────────────────

def train(n_envs: int = 8, render_one: bool = False, total_steps: int = 100_000):

    # Create log / checkpoint directories
    os.makedirs("coef0/logs/monitor",     exist_ok=True)
    os.makedirs("coef0/logs/eval",        exist_ok=True)
    os.makedirs("coef0/logs/tensorboard", exist_ok=True)
    os.makedirs("coef0/checkpoints/best", exist_ok=True)

    # ── Environments ──────────────────────────────────────────────────────
    env_fns = [
        make_env(rank=i, render=(render_one and i == 0))
        for i in range(n_envs)
    ]
    train_env = SubprocVecEnv(env_fns)
    train_env = VecMonitor(train_env, filename="coef0/logs/monitor/train")

    eval_env = DummyVecEnv([make_env(rank=0, render=False)])
    eval_env = VecMonitor(eval_env)

    # ── Callbacks ─────────────────────────────────────────────────────────
    checkpoint_cb = CheckpointCallback(
        save_freq=10_000,              # save every 5k steps
        save_path="coef0/checkpoints/",
        name_prefix="turtlebot_ppo",
        verbose=1,
    )
    eval_cb = EvalCallback(
        eval_env,
        best_model_save_path="coef0/checkpoints/best/",
        log_path="coef0/logs/eval/",
        eval_freq=20_000,
        n_eval_episodes=5,
        deterministic=True,
        verbose=1,
    )

    # ── Auto-resume logic ─────────────────────────────────────────────────
    latest = get_latest_checkpoint()

    if latest:
        print(f"\nResuming from checkpoint: {latest}\n")
        model = PPO.load(
            latest,
            env=train_env,
            device="auto",
            # Pass these so SB3 doesn't complain about mismatched hyperparams
            custom_objects={
                "learning_rate": 3e-4,
                "clip_range":    0.2,
            },
        )
        model.set_env(train_env)
        reset_num_timesteps = False    # keep step count from checkpoint

    else:
        print("\nNo checkpoint found — starting fresh\n")
        model = PPO(
            "MlpPolicy",
            train_env,
            n_steps=2048,
            batch_size=512,
            n_epochs=10,
            learning_rate=3e-4,
            gamma=0.99,
            gae_lambda=0.95,
            clip_range=0.2,
            ent_coef=0.001,
            verbose=1,
            tensorboard_log="coef0/logs/tensorboard/",
            device="auto",
        )
        reset_num_timesteps = True     # start from step 0

    # ── Learn ─────────────────────────────────────────────────────────────
    print(f"Training: {n_envs} envs  |  target: {total_steps:,} steps\n")

    model.learn(
        total_timesteps=total_steps,
        callback=[checkpoint_cb, eval_cb],
        reset_num_timesteps=reset_num_timesteps,
        progress_bar=True,
    )

    model.save("coef0/turtlebot_final")
    print("\nSaved: coef0/turtlebot_final.zip")

    train_env.close()
    eval_env.close()


# ── Visual evaluation ─────────────────────────────────────────────────────────

def evaluate(model_path: str = "coef0/checkpoints/best/best_model", n_episodes: int = 10):

    POLICY_HZ = 100

    env   = TurtleBotEnv(render=True)
    model = PPO.load(model_path, env=env)

    print(f"\nEvaluating: {model_path}  |  {n_episodes} episodes\n")

    try:
        for ep in range(n_episodes):
            obs, _    = env.reset()
            ep_reward = 0.0
            steps     = 0
            done      = False

            while not done:
                action, _ = model.predict(obs, deterministic=True)
                obs, reward, terminated, truncated, _ = env.step(action)
                ep_reward += reward
                steps     += 1
                done       = terminated or truncated
                time.sleep(1.0 / POLICY_HZ)

            if terminated and ep_reward > 0:
                result = "GOAL"
            elif terminated:
                result = "COLLISION"
            else:
                result = "TIMEOUT"

            print(f"Episode {ep+1:02d}: {result:10s}  steps={steps:5d}  reward={ep_reward:.2f}")

    finally:
        env.close()


# ── CLI ───────────────────────────────────────────────────────────────────────

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="TurtleBot3 PPO trainer")

    parser.add_argument("--envs",   type=int,  default=8,
                        help="Number of parallel training envs (default: 8)")
    parser.add_argument("--steps",  type=int,  default=300_000,
                        help="Total training timesteps (default: 300_000)")
    parser.add_argument("--render", action="store_true",
                        help="Open GUI for env 0 during training")
    parser.add_argument("--eval",   action="store_true",
                        help="Run visual evaluation instead of training")
    parser.add_argument("--model",  type=str,  default="coef0/checkpoints/best/best_model",
                        help="Model path for --eval (default: coef0/checkpoints/best/best_model)")

    args = parser.parse_args()

    if args.eval:
        evaluate(model_path=args.model)
    else:
        train(n_envs=args.envs, render_one=args.render, total_steps=args.steps)