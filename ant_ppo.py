import gym
from stable_baselines3 import PPO
from stable_baselines3.common.evaluation import evaluate_policy
from bots import ant

steps = 64
episodes = 10

# Create environment
env = ant()

# Instantiate the agent
model = PPO('MlpPolicy', env, n_steps=steps, verbose=1)
# Train the agent
'''
for i in range (episodes):
    print("Episode: ", i+1)
    model.learn(total_timesteps=int(steps))  
    #env.reset()
'''
model.learn(total_timesteps=int(steps*episodes))
# Save the agent
model.save("ant_walk")

env.reset()
env.stop()
print("Training complete")
