# *******************************************************************************
#  ant.py 
#
#  Author: James Mock
#  Email: james.w.mock@protonmail.com
#  Date: 08/21/2021
# *******************************************************************************

import time
import numpy as np
from envs.walkers.bot_env import BotEnv

THERMAL_LIMIT = 70 # celsius
COOLDOWN_TIME = 20 # minutes


class AntEnv(BotEnv):
    def __init__(self, 
                 forward_weight=2.0, 
                 ctrl_cost_weight=0.5,
                 contact_cost_weight=0.05,
                ):

        self.forward_weight = forward_weight
        self.ctrl_cost_weight = ctrl_cost_weight
        self.contact_cost_weight = contact_cost_weight
        # SERVO_ID, MIN_POS, MAX_POS, RESET_STEPS
        self.servo_params = np.array([[0, 1536, 2560, 2048, 2048], # front left hip
                                      [1, 1266, 1707, 2048, 1366], # front left knee
                                      [2, 1536, 2560, 2048, 2048], # front right hip
                                      [3, 2389, 2830, 2048, 2730], # front right knee
                                      [4, 1536, 2560, 2048, 2048], # rear left hip
                                      [5, 1266, 1707, 2048, 1366], # rear left knee
                                      [6, 1536, 2560, 2048, 2048], # rear right hip
                                      [7, 2389, 2830, 2048, 2730]])# rear right knee
        num_sensors = 4
        num_cameras = 1
        super().__init__(self.servo_params, num_sensors, num_cameras)

    def control_cost(self, action):
        control_cost = self.ctrl_cost_weight * np.sum(np.square(action - self.lastAction))
        return control_cost

    def contact_cost(self, contact):
        contact_cost = self.contact_cost_weight * np.sum(np.square(contact))
        return contact_cost

    def getReward(self, state):
        reward = self.forward_weight * (state[2] - self.lastPosition[2]) / self.dt
        self.lastPosition = np.copy(state)
        return reward

    def step(self, action):

        #print(action)	
        actionStart = time.monotonic()
        #act = np.where(action<0, 0, action) 
        #act = np.where(act>1, 1, act)
        self.servoLock.acquire()
        self.action[:] = action
        self.servoLock.release()
        self.newAction.set()
        actionStop = time.monotonic()

        time.sleep(self.dt)

        obsStart = time.monotonic()
        state = self.observeState()
        obsStop = time.monotonic()

        actionTime = actionStop-actionStart
        obsTime = obsStop-obsStart
        #print("Action Time: ", actionTime, "\tObs Time:, ", obsTime)
        rewards = self.getReward(state[:3])
        costs = self.control_cost(action) + self.contact_cost(state[-4:])
        reward = 1.0 + rewards - costs

        # check errors
        done = False
        if self.error.is_set() or not self.servoActive.is_set():
            done = True
            reward = reward - 10
        #print(temp)
        if self.overheat.is_set() or not self.camActive.is_set():
            done = True
            
             

        self.lastPosition = np.copy(state[:3])
        self.lastAction = action
        quat = state[3:7]
        #print("X: %.3f Y: %.3f Z: %.3f" % (xyz[0], xyz[1], xyz[2]))
        #print("Reward: %.3f" % reward)

        info = {"forward_reward": rewards,
                "control_cost": costs,

                "action_time": actionTime,
                "observation_time": obsTime,
                "Translation": self.lastPosition,
                "Orientation": quat}

        return state[3:], reward, done, info
   


