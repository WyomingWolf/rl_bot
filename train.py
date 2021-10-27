#!/usr/bin/env python
# -*- coding: utf-8 -*-

# ***********************     SyncRead and SyncWrite Example      ***********************
#  Required Environment to run this example :
#    - 
#    - 
#  How to use the example :
#    -  
#    - 
#    - 
#    - 
#  Author: James Mock
#  Email: james.w.mock@protonmail.com
#  Date: 08/21/2021
# *******************************************************************************

import gym
import argparse
from stable_baselines3 import PPO
from stable_baselines3 import SAC
from stable_baselines3.common.evaluation import evaluate_policy
from envs.walkers.ant import AntEnv
from gym.wrappers.time_limit import TimeLimit


# construct the argument parser and parse the arguments
ap = argparse.ArgumentParser()
ap.add_argument("-a", "--algorithm", default="PPO",
	help="PPO or SAC")
ap.add_argument("-m", "--model", required=False,
	help="Load trained model")
ap.add_argument("-s", "--save_as", default="trained_model",
	help="Name saved model")
ap.add_argument("-t", "--timesteps", type=int, default=20000,
	help="Total training steps")
args = vars(ap.parse_args())

n_steps = 256
total_timesteps = args["timesteps"]


# Create environment
env = AntEnv()
#env.spec.max_episode_steps = steps
env = TimeLimit(env, max_episode_steps=n_steps)

# Initialize agent
if args["algorithm"] == "PPO":
    if args["model"] is not None:
        model = PPO.load(args["model"], env, print_system_info=True)
    else:
        model = PPO('MlpPolicy', env, verbose=1)
else:
    if args["model"] is not None:
        model = SAC.load(args["model"], env, print_system_info=True)
    else:
        model = SAC('MlpPolicy', env, train_freq=(n_steps, "step"), gradient_steps=-1, verbose=1)

# Train the agent
model.learn(total_timesteps=int(total_timesteps))

# Save the agent
model.save(args["save_as"])

env.reset()
env.stop()
print("Training complete")


