import os
from stable_baselines3 import PPO
from stable_baselines3.common.callbacks import CheckpointCallback

from rocket_mujoco_ros.rocket_env import FreeJointRocketTerrainLandingEnv
from rocket_mujoco_ros.rl.sensor_obs_wrapper import SensorObservationWrapper

def main():
    env = FreeJointRocketTerrainLandingEnv(render_mode=None)
    env = SensorObservationWrapper(env)

    model_dir = os.path.expanduser("~/rocket_logs/models")
    os.makedirs(model_dir, exist_ok=True)

    # Eğitimi hızlandırmak için PPO hiperparametreleri
    model = PPO(
        "MlpPolicy",
        env,
        learning_rate=3e-4,
        n_steps=2048,
        batch_size=64,
        n_epochs=10,
        gamma=0.99,
        gae_lambda=0.95,
        clip_range=0.2,
        ent_coef=0.0,
        verbose=1,
        tensorboard_log=os.path.join(model_dir, "tb_logs")
    )

    checkpoint_callback = CheckpointCallback(
        save_freq=10000,
        save_path=model_dir,
        name_prefix="rocket_model"
    )

    print("Eğitim başlıyor... Bu işlem sistem performansına göre vakit alabilir.")
    model.learn(total_timesteps=100000, callback=checkpoint_callback)

    model_path = os.path.join(model_dir, "rocket_model_final")
    model.save(model_path)
    print(f"Eğitim tamamlandı! Model şuraya kaydedildi: {model_path}.zip")

if __name__ == "__main__":
    main()
