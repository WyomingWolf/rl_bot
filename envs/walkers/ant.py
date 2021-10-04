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

import time
import numpy as np
from envs.walkers.bot_env import BotEnv


class AntEnv(BotEnv):
    def __init__(self, 
                 forward_weight=1.0, 
                 ctrl_cost_weight=0.5,
    ):
        self.forward_weight = forward_weight
        self.ctrl_cost_weight = ctrl_cost_weight
        # SERVO_ID DEFAULT_POS MIN_POS MAX_POS
        self.servos = np.array([[0, 2048, 1024, 3072], # front left hip
                                [1, 1366, 1309, 2048], # front left knee
                                [2, 2048, 1024, 3072], # front right hip
                                [3, 2730, 2048, 2787], # front right knee
                                [4, 2048, 1024, 3072], # rear left hip
                                [5, 1366, 1309, 2048], # rear left knee
                                [6, 2048, 1024, 3072], # rear right hip
                                [7, 2730, 2048, 2787]])# rear right knee
        num_sensors = 4
        num_cameras = 1
        super().__init__(self.servos, num_sensors, num_cameras)

    def control_cost(self, action):
        #control_cost = self.ctrl_cost_weight * np.sum(np.square(torque))
        #print(control_cost)
        control_cost = self.ctrl_cost_weight * np.sum(np.square(action - self.lastAction))
        self.lastAction = action
        return control_cost

    def getReward(self, state):
        reward = self.forward_weight * (state[0] - self.lastPosition[0]) / self.dt
        self.lastPosition = np.copy(state)
        return reward

    def step(self, action):
         #print(action)	
         actionStart = time.monotonic()
         act = np.where(action<0, 0, action) 
         act = np.where(act>1, 1, act)
         self.servos.writeAction(action)
         actionStop = time.monotonic()

         obsStart = time.monotonic()
         state, err = self.observeState()
         obsStop= time.monotonic()
         time.sleep(self.dt)
         actionTime = actionStop-actionStart
         obsTime = obsStop-obsStart
         #print("Action Time: ", actionTime, "\tObs Time:, ", obsTime)
         rewards = self.getReward(state[-6:])
         #torque = state[::3]
         #print(torque[:self.act_size])
         costs = self.control_cost(action)
         reward = rewards - costs
         done = False
         if np.sum(err) > 0:
             done = True

         info = {"forward_reward": rewards,
                 "control_cost": costs,

                 "action_time": actionTime,
                 "observation_time": obsTime,
                 "Translation": state[-6:-3],
                 "Rotation": state[-3:]}

         return state[:self.obs_size], reward, done, info
   


