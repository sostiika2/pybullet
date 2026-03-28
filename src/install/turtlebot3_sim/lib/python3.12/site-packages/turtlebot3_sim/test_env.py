import os
import shutil
import datetime
from stable_baselines3 import PPO
from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.callbacks import CheckpointCallback
from turtlebot_env import TurtleBotEnv 


def train():
    # --- Configuration ---
    MODEL_NAME = "ppo_enhanced_v2"
    BASE_DIR = os.path.dirname(__file__)
    
    MODELS_DIR = os.path.join(BASE_DIR, "models_enhanced_v2")
    CHECKPOINT_DIR = os.path.join(MODELS_DIR, "checkpoints_enhanced_v2")
    LOG_PATH = os.path.join(BASE_DIR, "tensorboard_logs_enhanced_v2")
    SAVE_PATH = os.path.join(MODELS_DIR, MODEL_NAME)

    os.makedirs(MODELS_DIR, exist_ok=True)
    os.makedirs(CHECKPOINT_DIR, exist_ok=True)
    os.makedirs(LOG_PATH, exist_ok=True)

    # --- Environment ---
    env = Monitor(TurtleBotEnv())

    # --- Checkpoint Callback ---
    checkpoint_callback = CheckpointCallback(
        save_freq=10000,  # 🔥 safer (less loss if crash)
        save_path=CHECKPOINT_DIR,
        name_prefix=f"{MODEL_NAME}_ckpt",
        verbose=1
    )

    # --- Model Load or Create ---
    if os.path.exists(SAVE_PATH + ".zip"):
        print(f">>> Loading existing model: {SAVE_PATH}")

        model = PPO.load(
            SAVE_PATH + ".zip",
            env=env,
            tensorboard_log=LOG_PATH,
            custom_objects={"learning_rate": 3e-4},
            ent_coef=0.0000
            
        )

        reset_timesteps = False  # ✅ continue training
    else:
        print(">>> No existing model found. Training from scratch...")

        model = PPO(
            policy="MlpPolicy",
            env=env,
            learning_rate=3e-4,
            n_steps=2048,
            batch_size=64,
            n_epochs=10,
            gamma=0.99,
            gae_lambda=0.95,
            ent_coef=0.00,
            verbose=1,
            tensorboard_log=LOG_PATH   # ✅ ENABLE LOGGING
        )

        reset_timesteps = True  # ✅ new training

    # --- Training ---
    print(f"Starting training for {MODEL_NAME}...")

    try:
        model.learn(
            total_timesteps=750000,
            reset_num_timesteps=reset_timesteps,
            callback=checkpoint_callback,
            tb_log_name=MODEL_NAME   # ✅ SAME NAME → same TB graph continues
        )

    except KeyboardInterrupt:
        print("\n>>> Training interrupted. Saving...")

    except Exception as e:
        print(f"\n>>> ERROR: {e}")

    finally:
        # --- Save model ---
        model.save(SAVE_PATH)

        # --- Backup ---
        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M")
        backup_path = f"{SAVE_PATH}_backup_{timestamp}.zip"
        shutil.copy(SAVE_PATH + ".zip", backup_path)

        print(f"Final model saved at: {SAVE_PATH}")
        print(f"Backup created at: {backup_path}")

        env.close()
        print("Done!")


if __name__ == "__main__":
    train()