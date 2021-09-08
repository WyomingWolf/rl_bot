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

import os
import time
import numpy as np
from bots import ant
from stable_baselines import PPO2

env = ant()
# Optional: PPO2 requires a vectorized environment to run
# the env is now wrapped automatically when passing it to the constructor
# env = DummyVecEnv([lambda: env])

model = PPO2('MlpPolicy', env, verbose=1)
model.learn(total_timesteps=10000)

obs = env.reset()
for i in range(1000):
    action, _states = model.predict(obs)
    obs, rewards, dones, info = env.step(action)
    if done:
      obs = env.reset()

env.reset()
time.sleep(2.0)
env.stop()
plt.plot(reward_arr)


