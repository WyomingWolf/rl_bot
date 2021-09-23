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

# hyperparameters
n_steps          = 100
n_episodes       = 5

# initalize robot
bot = ant()
act_space = bot.act_space
obs_space = bot.obs_space
 
reward_arr = np.zeros(n_episodes)
for i in range(n_episodes):

     total_reward = 0

     print("Episode: ", i+1)
     state = bot.reset()
     time.sleep(1.0)
     start = time.monotonic()
     step = 1
     for j in range(n_steps):

         action = np.random.rand(act_space)
         next_state, reward, done = bot.step(action)
         total_reward += reward
         step += 1
         if done:
             break
    
     stop = time.monotonic()
     freq = step/(stop-start)
     reward_arr[i] = total_reward
     print("Total Reward: {:.3f}".format(total_reward), "\tForward Pass Per Second: {:.3f}".format(freq))

bot.stop()



