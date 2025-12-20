import stable_baselines3 as sb3
from maze_rl_pkg.maze_env import MazeEnv
from maze_rl_pkg.maze_env import FastTrainingWrapper
import torch
import os

def main():
    # Create environment
    env = MazeEnv()

    # Wrap if needed for additional functionality
    env = FastTrainingWrapper(env)

    # Create PPO agent with STABLE hyperparameters
    model = sb3.PPO(
        "MlpPolicy", 
        env,
        # Critical: Batch size MUST be > 1 for advantage normalization
        batch_size=64,
        # Set value clipping to match policy clipping
        clip_range_vf=0.2,
        # Default gradient clipping should be fine
        max_grad_norm=0.5,
        # Add policy kwargs for stable network
        policy_kwargs=dict(
            net_arch=[dict(pi=[64, 64], vf=[64, 64])],
            activation_fn=torch.nn.ReLU,
            log_std_init=-0.5  # Smaller initial exploration
        ),
        # Other stable hyperparameters
        learning_rate=3e-4,
        n_steps=2048,
        n_epochs=10,
        gamma=0.99,
        gae_lambda=0.95,
        clip_range=0.2,
        ent_coef=0.01,
        vf_coef=0.5,
        verbose=1,
        # Add tensorboard logging for debugging
        tensorboard_log="./ppo_maze_tensorboard/"
    )

    # Create directory for tensorboard logs
    os.makedirs("./ppo_maze_tensorboard/", exist_ok=True)
    
    print("Starting training with stable PPO parameters...")
    print(f"Batch size: {model.batch_size}")
    print(f"n_steps: {model.n_steps}")
    
    # Train with try-catch for better error handling
    try:
        model.learn(total_timesteps=10000)
        
        # Save
        model.save("maze_ppo")
        print("Training completed successfully!")
        
    except Exception as e:
        print(f"Training failed with error: {e}")
        print("Consider trying SAC algorithm instead...")
        
    finally:
        # Close environment
        env.close()
    
if __name__ == "__main__":
    main()