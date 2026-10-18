"""PPO egitimi + sonuc raporu.

- Ortami Monitor ile sarar (episode odul/uzunluk/success loglanir)
- PPO'yu rocket_rl_tilepad.py ile ayni hiperparametrelerle egitir
- Modeli MODEL_NAME olarak kaydeder
- Egitim sonrasi deterministic degerlendirme yapar
- results/ altina ogrenme egrileri ve ozet yazar
"""

import argparse
import json
from pathlib import Path

import numpy as np
import pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from stable_baselines3 import PPO
from stable_baselines3.common.env_checker import check_env
from stable_baselines3.common.logger import configure
from stable_baselines3.common.monitor import Monitor

from rocket_env_tilepad import FreeJointRocketTerrainLandingEnv, MODEL_NAME

RESULTS_DIR = Path("results")


def make_env(monitor_path=None):
    env = FreeJointRocketTerrainLandingEnv()
    if monitor_path is not None:
        env = Monitor(env, filename=str(monitor_path), info_keywords=("success",))
    return env


def train(total_timesteps, run_name="train_monitor"):
    RESULTS_DIR.mkdir(exist_ok=True)

    raw_env = FreeJointRocketTerrainLandingEnv()
    check_env(raw_env, warn=True)
    raw_env.close()

    env = make_env(RESULTS_DIR / run_name)

    if Path(f"{MODEL_NAME}.zip").exists():
        print(f"Kayitli model bulundu, ustunden devam ediliyor: {MODEL_NAME}.zip")
        model = PPO.load(MODEL_NAME, env=env, device="cpu")
    else:
        model = PPO(
            policy="MlpPolicy",
            env=env,
            verbose=1,
            learning_rate=3e-4,
            n_steps=1024,
            batch_size=64,
            gamma=0.99,
            device="cpu",
        )

    logger = configure(str(RESULTS_DIR / f"sb3_logs_{run_name}"), ["stdout", "csv"])
    model.set_logger(logger)

    model.learn(total_timesteps=total_timesteps, reset_num_timesteps=False)
    model.save(MODEL_NAME)
    print(f"Kaydedildi: {MODEL_NAME}.zip")
    env.close()


def evaluate(n_episodes=50):
    env = make_env()
    model = PPO.load(MODEL_NAME, device="cpu")

    rewards, lengths, successes = [], [], []
    for ep in range(n_episodes):
        obs, info = env.reset()
        done = False
        total_r, steps = 0.0, 0
        while not done:
            action, _ = model.predict(obs, deterministic=True)
            obs, r, terminated, truncated, info = env.step(action)
            total_r += r
            steps += 1
            done = terminated or truncated
        rewards.append(total_r)
        lengths.append(steps)
        successes.append(bool(info.get("success", False)))
        print(f"eval ep {ep + 1}/{n_episodes}: reward={total_r:.1f} steps={steps} success={successes[-1]}")

    env.close()

    summary = {
        "n_episodes": n_episodes,
        "success_rate": float(np.mean(successes)),
        "mean_reward": float(np.mean(rewards)),
        "std_reward": float(np.std(rewards)),
        "min_reward": float(np.min(rewards)),
        "max_reward": float(np.max(rewards)),
        "mean_episode_length": float(np.mean(lengths)),
    }
    with open(RESULTS_DIR / "eval_summary.json", "w") as f:
        json.dump(summary, f, indent=2)
    print("Eval ozeti:", json.dumps(summary, indent=2))


def plot():
    # Tum kosulari sirayla birlestir (ilk egitim + devam kosulari).
    mon_paths = [RESULTS_DIR / "train_monitor.monitor.csv"]
    mon_paths += sorted(p for p in RESULTS_DIR.glob("train_monitor_*.monitor.csv"))
    frames = [pd.read_csv(p, skiprows=1) for p in mon_paths if p.exists()]
    df = pd.concat(frames, ignore_index=True)
    df["timesteps"] = df["l"].cumsum()
    df["success"] = df["success"].astype(str).isin(["True", "true", "1", "1.0"]).astype(float)

    window = max(5, len(df) // 50)

    ts = df["timesteps"].to_numpy()
    rew = df["r"].to_numpy()
    rew_roll = df["r"].rolling(window).mean().to_numpy()
    suc_roll = df["success"].rolling(window).mean().to_numpy()
    length = df["l"].to_numpy()
    len_roll = df["l"].rolling(window).mean().to_numpy()

    fig, axes = plt.subplots(1, 3, figsize=(16, 4.5))

    axes[0].plot(ts, rew, alpha=0.25, color="tab:blue")
    axes[0].plot(ts, rew_roll, color="tab:blue")
    axes[0].set_title("Episode Reward")
    axes[0].set_xlabel("Timesteps")
    axes[0].set_ylabel("Reward")

    axes[1].plot(ts, suc_roll, color="tab:green")
    axes[1].set_title(f"Success Rate (rolling {window} ep)")
    axes[1].set_xlabel("Timesteps")
    axes[1].set_ylabel("Success rate")
    axes[1].set_ylim(-0.05, 1.05)

    axes[2].plot(ts, length, alpha=0.25, color="tab:orange")
    axes[2].plot(ts, len_roll, color="tab:orange")
    axes[2].set_title("Episode Length")
    axes[2].set_xlabel("Timesteps")
    axes[2].set_ylabel("Steps")

    fig.tight_layout()
    fig.savefig(RESULTS_DIR / "training_curves.png", dpi=150)
    print(f"Grafik kaydedildi: {RESULTS_DIR / 'training_curves.png'}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--timesteps", type=int, default=300_000)
    parser.add_argument("--eval-episodes", type=int, default=50)
    parser.add_argument("--skip-train", action="store_true")
    parser.add_argument("--run-name", type=str, default="train_monitor")
    args = parser.parse_args()

    if not args.skip_train:
        train(args.timesteps, run_name=args.run_name)
    plot()
    evaluate(args.eval_episodes)
