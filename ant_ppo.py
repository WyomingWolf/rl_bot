import gym
from stable_baselines3 import PPO
from stable_baselines3.common.evaluation import evaluate_policy
from bots import ant

# Create environment
env = ant()

# Instantiate the agent
model = PPO('MlpPolicy', env, verbose=2)
# Train the agent
model.learn(total_timesteps=int(1000))
# Save the agent
model.save("ppo_ant")

env.stop()
print("Training complete")
