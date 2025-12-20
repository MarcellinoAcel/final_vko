# test_env.py
from maze_rl_pkg.maze_env import MazeEnv
from maze_rl_pkg.maze_env import FastTrainingWrapper
import numpy as np

def test_environment():
    """Test if the environment returns valid observations and rewards"""
    env = MazeEnv()
    env = FastTrainingWrapper(env)
    
    print("Testing environment reset...")
    obs, info = env.reset()
    print(f"Observation shape: {obs.shape}")
    print(f"Observation range: [{obs.min():.3f}, {obs.max():.3f}]")
    print(f"Contains NaN: {np.any(np.isnan(obs))}")
    print(f"Contains Inf: {np.any(np.isinf(obs))}")
    
    print("\nTesting 10 random steps...")
    for i in range(10):
        action = env.action_space.sample()
        obs, reward, terminated, truncated, info = env.step(action)
        
        print(f"Step {i}: Reward={reward:.3f}, "
              f"Terminated={terminated}, Truncated={truncated}")
        
        if np.any(np.isnan(obs)):
            print(f"  WARNING: Observation contains NaN at step {i}!")
            break
        if np.any(np.isinf(obs)):
            print(f"  WARNING: Observation contains Inf at step {i}!")
            break
            
        if terminated or truncated:
            obs, info = env.reset()
            print(f"  Environment reset")
    
    env.close()
    print("\nEnvironment test completed!")

if __name__ == "__main__":
    test_environment()