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
from pynput import keyboard

TIMEOUT = 10.0

class BotEnv(gym.Env):
    def __init__(self, servos, num_sensors=0, num_cameras=1):
        self.dt = 0.1 # 100ms
        #self.spec.max_episode_steps = 256
        self.act_size = np.shape(servos)[0]
        self.servo_ids = servos[:,0]
        self.num_sensors = num_sensors
        self.num_cameras = num_cameras 
        self.obs_size = self.act_size*3 + self.num_sensors
        self.lastPosition = np.zeros(6)
        self.lastAction = np.zeros(self.act_size)
        
        act = np.ones((self.act_size, 2)) * [-1, 1]
        self.action_space = spaces.Box(low=act[:,0], high=act[:,1], dtype=np.float16) 
        obs = np.ones((self.obs_size, 2)) * [-1, 1] 
        self.observation_space = spaces.Box(low=obs[:,0], high=obs[:,1], dtype=np.float16)
        #self.spec.max_episode_steps = 128

        self.botActive = mp.Event()
        self.botActive.set()

        self.botPause = False
                   
        # initialize servos
        print("\nInitializing servos")
        self.servos = sc(servos)

        # initialize foot sensors
        self.adcLock = mp.Lock()
        self.adcData = mp.Array('d', 4)
        self.adcActive = mp.Event()
        if self.num_sensors > 0 and self.num_sensors <= 4:
            try:
                print("\nInitializing contact sensors")
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

        print("\nInitializing stereo camera")
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

        # Collect events until released
        listener = keyboard.Listener(on_press=self.on_press)
        listener.start()
        #with keyboard.Listener(on_press=self.on_press) as listener: listener.join()

        print("\nAll systems nonimal\n") 


    def on_press(self, key):
        if key == keyboard.Key.space:
            self.botPause = True     

    def observeState(self):
        # get servo data
        #obsStart0 = time.monotonic()
        obs, err, temp = self.servos.readState()
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
        return np.concatenate((servoState, contact, position), -1), err, temp 

    def reset(self):
         print("Reset called")
         obs, err, temp = self.observeState()
         if np.sum(err) > 0:
             print("Error: Hardware error detected, rebooting servos.")
             self.rebootServos(err)
             time.sleep(0.25)
         self.servos.moveToStart()
         time.sleep(0.5)

         if np.sum(err) > 0:
             print("Error: Check servos")
             self.stop()
         timer = 0
         while not self.camActive.is_set():
             print("Waiting for camera")
             time.sleep(1.0)
             if timer == 15:
                 try:
                     print("Error: Unable to close camera, relaunching process.")
                     self.tracker.join()
                     self.tracker.terminate()
                      
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
                     self.stop()
                        
         if self.botPause:
             self.botPause = False
             print("Robot paused. Press 'Enter' to continue.")
             x = input()
 
         self.resetPos.set()
         time.sleep(0.5)
         obs, err, temp = self.observeState()

         #print(obs[:self.obs_size])
         return obs[:self.obs_size]

    def enableServos(self):
        for i in range(self.act_size):
            servo = self.servo_ids[i]
            self.servos.setTorque(servo, 1)

    def disableServos(self):
        for i in range(self.act_size):
            servo = self.servo_ids[i]
            self.servos.setTorque(servo, 0)

    def rebootServos(self, err):
        for i in range(len(err)):
           if err[i] != 0:
              self.servos.reboot(self.servos.servos[i,0])

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



   


