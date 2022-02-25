# *******************************************************************************
#  train.py 
#
#  Author: James Mock
#  Email: james.w.mock@protonmail.com
#  Date: 08/21/2021
# *******************************************************************************

import os
import gym
import argparse
from stable_baselines3 import PPO
from stable_baselines3 import SAC
from stable_baselines3.common.monitor import Monitor
from utils import SaveOnBestTrainingRewardCallback
from gym.wrappers.time_limit import TimeLimit
from utils import plot_results
from envs.walkers.ant import AntEnv

# construct the argument parser and parse the arguments
ap = argparse.ArgumentParser()
ap.add_argument("-a", "--algorithm", default="PPO",
	help="PPO or SAC")
ap.add_argument("-m", "--model", required=False,
	help="Load trained model")
ap.add_argument("-s", "--save_as", default="trained_model",
	help="Name saved model")
ap.add_argument("-t", "--timesteps", type=int, default=100000,
	help="Total training steps")
args = vars(ap.parse_args())

n_steps = 1000
total_timesteps = args["timesteps"]

cwd = os.path.abspath(os.getcwd())
log_dir = os.path.join(cwd, "logs")

# Create environment
env = AntEnv()
env = TimeLimit(env, max_episode_steps=n_steps)
env = Monitor(env, log_dir)
#env.spec.max_episode_steps = n_steps
#env = TimeLimit(env, max_episode_steps=n_steps)

# Initialize agent
if args["algorithm"] == "PPO":
    if args["model"] is not None:
        try:
            model = PPO.load(args["model"], env, print_system_info=True)
        except Exception as e:
            print(e)
    else:
        model = PPO('MlpPolicy', env, verbose=1)
else:
    if args["model"] is not None:
        try:
            model = SAC.load(args["model"], env, print_system_info=True)
        except Exception as e:
            print(e)

        print("Model S")
    else:
        model = SAC('MlpPolicy', env, train_freq=(1, "episode"), gradient_steps=-1, verbose=1)

callback = SaveOnBestTrainingRewardCallback(check_freq=5000, log_dir=log_dir)
# Train the agent
model.learn(total_timesteps=int(total_timesteps), callback=callback)

# Save the agent
model.save(args["save_as"])

env.reset()
env.close()
print("Training complete")


