from stable_baselines3 import PPO
from maze_rl_pkg.maze_env import MazeEnv

def main():
    env = MazeEnv()
    model = PPO.load("maze_ppo")

    obs, _ = env.reset()

    while True:
        action, _ = model.predict(obs, deterministic=True)
        obs, reward, done, truncated, _ = env.step(action)

        if done or truncated:
            obs, _ = env.reset()

if __name__ == "__main__":
    main()
