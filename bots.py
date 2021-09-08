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
import multiprocessing as mp
from ServoController import ServoController as sc
from adc import GetContactData
from PositionTracker import GetZEDPosition

class bot:
    def __init__(self, servos, num_sensors=0, num_cameras=1):
        self.act_space = np.shape(servos)[0]
        self.num_sensors = num_sensors
        self.num_cameras = num_cameras 
        self.obs_space = self.act_space*3 + self.num_sensors
        self.lastPos = np.zeros(6)
        self.botActive = mp.Event()
        self.botActive.set()
             
        # initialize servos
        print("Initializing servos")
        self.servos = sc(servos)

        # initialize foot sensors
        self.adcLock = mp.Lock()
        self.adcData = mp.Array('d', 4)#, lock=self.frsLock)
        self.adcActive = mp.Event()
        if self.num_sensors > 0 and self.num_sensors <= 4:
            try:
                print("Initializing contact sensors")
                self.contact = mp.Process(target=GetContactData, 
                                          args=(self.botActive,
                                                self.adcData,
                                                self.adcLock,
                                                self.adcActive))
                self.contact.start()
                while not self.adcActive.is_set():
                    time.sleep(.01)
            except Exception as e:
                print(e)
                print("Failed to inilize contact sensors")
                quit()

        print("Initializing camera(s)")
        # start ZED camera process
               
        self.camLock = mp.Lock()
        self.posEst = mp.Array('d', 6)#, lock=self.camLock)
        self.camActive = mp.Event()
        try:
            self.tracker = mp.Process(target=GetZEDPosition, 
                                      args=(self.botActive,
                                            self.posEst,
                                            self.camLock,
                                            self.camActive))
            self.tracker.start()
            while not self.camActive.is_set():
                time.sleep(.01)

        except Exception as e:
            print(e)
            print("Failed to inilize camera(s)")
            quit()

        print("All systems nonimal")       
            
    def observeState(self):
        # get servo data
        obsStart0 = time.monotonic()
        obs, err = self.servos.readState()
        servoState = np.ndarray.flatten(obs)
        obsStop0 = time.monotonic()
        obsTime0 = obsStop0-obsStart0
        
        # get force sensor data
        #obsStart1 = time.monotonic()
        self.adcLock.acquire()
        contact = self.adcData[:]
        self.adcLock.release()
        #obsStop1 = time.monotonic()
        #obsTime1 = obsStop1-obsStart1

        # get camera position
        #obsStart2 = time.monotonic()
        self.camLock.acquire()
        position = self.posEst[:]
        self.camLock.release()
        #obsStop2 = time.monotonic()
        #obsTime2 = obsStop2-obsStart2

        print("Servo Time:, ", obsTime0) #, "\tSensor Time: ", obsTime1, "\tCamera Time: ", obsTime2)
        return np.concatenate((servoState, contact, position), -1), err

    def getReward(self, state):
        reward = state[2] - self.lastPos[2]
        self.lastPos = np.copy(state)
        return reward

    def step(self, action):
         actionStart = time.monotonic()
         #print(np.size(action))
         act = np.where(action<0, 0, action) 
         act = np.where(act>1, 1, act)
         self.servos.writeAction(action)
         actionStop = time.monotonic()

         obsStart = time.monotonic()
         state, err = self.observeState()
         obsStop= time.monotonic()
         time.sleep(.02)
         actionTime = actionStop-actionStart
         obsTime = obsStop-obsStart
         #print("Action Time: ", actionTime, "\tObs Time:, ", obsTime)
         reward = self.getReward(state[-6:])
         done = False
         if np.sum(err) > 0:
             done = True
         return state[:self.obs_space], reward, done 

    def reset(self):
         obs, err = self.observeState()
         if np.sum(err) > 0:
             self.servos.reboot(err)
         self.servos.moveToStart()
         time.sleep(1.0)
         obs, err = self.observeState()
         if np.sum(err) > 0:
             print("Error: Check servos")
             quit()
         return obs[:self.obs_space]

    def stop(self):
        self.botActive.clear()
        self.tracker.join()
        self.tracker.terminate()
        self.servos.close()


class ant(bot):
    def __init__(self):     
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


