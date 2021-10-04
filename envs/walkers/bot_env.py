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
import gym
from gym import spaces
from envs.walkers.hardware.dynamixel_xseries import ServoController as sc
from envs.walkers.hardware.ads1115 import GetContactData
from envs.walkers.hardware.zed_camera import GetZEDPosition

TIMEOUT = 10.0

class BotEnv(gym.Env):
    def __init__(self, servos, num_sensors=0, num_cameras=1):
        self.dt = 0.1 # 100ms
        #self.spec.max_episode_steps = 256
        self.act_size = np.shape(servos)[0]
        self.num_sensors = num_sensors
        self.num_cameras = num_cameras 
        self.obs_size = self.act_size*3 + self.num_sensors
        self.lastPosition = np.zeros(6)
        self.lastAction = np.zeros(self.act_size)
        self.botActive = mp.Event()
        self.botActive.set()
        act = np.ones((self.act_size, 2)) * [-1, 1]
        self.action_space = spaces.Box(low=act[:,0], high=act[:,1], dtype=np.float16) 
        obs = np.ones((self.obs_size, 2)) * [-1, 1] 
        self.observation_space = spaces.Box(low=obs[:,0], high=obs[:,1], dtype=np.float16)

             
        # initialize servos
        print("Initializing servos")
        self.servos = sc(servos)

        # initialize foot sensors
        self.adcLock = mp.Lock()
        self.adcData = mp.Array('d', 4)
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
                sleep_time = 0
                while not self.adcActive.is_set():
                    if sleep_time == TIMEOUT:
                        print("Failed to initalize contact sensors")
                        self.stop()
                    else: 
                        time.sleep(.01)
                        sleep_time += .01
            except Exception as e:
                print(e)
                print("Failed to initalize contact sensors")
                self.stop()

        print("Initializing stereo camera")
        # start ZED camera process
               
        self.camLock = mp.Lock()
        self.posEst = mp.Array('d', 6)
        self.camActive = mp.Event()
        self.resetPos = mp.Event()
        try:
            self.tracker = mp.Process(target=GetZEDPosition, 
                                      args=(self.botActive,
                                            self.posEst,
                                            self.camLock,
                                            self.camActive,
                                            self.resetPos))
            self.tracker.start()
            sleep_time = 0
            while not self.camActive.is_set():
                if sleep_time == TIMEOUT:
                    print("Failed to initalize stereo camera")
                    self.stop()
                else: 
                    time.sleep(.01)
                    sleep_time += .01
        except Exception as e:
            print(e)
            print("Failed to initalize stereo camera")
            self.stop()

        print("All systems nonimal")       
            
    def observeState(self):
        # get servo data
        #obsStart0 = time.monotonic()
        obs, err = self.servos.readState()
        servoState = np.ndarray.flatten(obs)
        #obsStop0 = time.monotonic()
        #obsTime0 = obsStop0-obsStart0
        
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

        #print("Servo Time: ", obsTime0) #, "\tSensor Time: ", obsTime1, "\tCamera Time: ", obsTime2)
        return np.concatenate((servoState, contact, position), -1), err 

    def reset(self):
         print("Reset called")
         obs, err = self.observeState()
         if np.sum(err) > 0:
             self.servos.reboot(err)
         self.servos.moveToStart()
         time.sleep(0.5)
         self.resetPos.set()
         time.sleep(0.5)
         obs, err = self.observeState()
         if np.sum(err) > 0:
             print("Error: Check servos")
             self.stop()
         #print(obs[:self.obs_size])
         return obs[:self.obs_size]

    def stop(self):
        self.botActive.clear()
        try:
            self.contact.join()
            self.contact.terminate()
        except Exception as e:
            print(e)
        try:
            self.tracker.join()
            self.tracker.terminate()
        except Exception as e:
            print(e)
        self.servos.close()



   


