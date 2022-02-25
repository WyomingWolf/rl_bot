# *******************************************************************************
#  bot_env.py 
#
#  Author: James Mock
#  Email: james.w.mock@protonmail.com
#  Date: 08/21/2021
# *******************************************************************************

import time
import numpy as np
import multiprocessing as mp
import gym
from gym import spaces
from envs.walkers.hardware.servo_controller import ServoController
from envs.walkers.hardware.ads1115 import GetContactData
from envs.walkers.hardware.zed_camera import GetZEDPosition

TIMEOUT = 20.0

class BotEnv(gym.Env):
    def __init__(self, servos, num_sensors=0, num_cameras=1):
        self.dt = 0.05 # 50ms
        self.num_servos = np.shape(servos)[0]
        self.servo_ids = servos[:,0]
        self.num_sensors = num_sensors
        self.num_cameras = num_cameras 
        self.obs_size = 10 + self.num_servos*3 + self.num_sensors
        self.lastPosition = np.zeros(7)
        self.lastAction = np.zeros(self.num_servos)
        
        act = np.ones((self.num_servos, 2)) * [-1, 1]
        self.action_space = spaces.Box(low=act[:,0], high=act[:,1], dtype=np.float32) 
        obs = np.ones((self.obs_size, 2)) * [-float("inf"), float("inf")] 
        self.observation_space = spaces.Box(low=obs[:,0], high=obs[:,1], dtype=np.float32)

        self.botActive = mp.Event()
        self.botActive.set()
                   
        # initialize servos
        self.servoLock = mp.Lock()
        self.servoData = mp.Array('d', self.num_servos*3)
        self.action = mp.Array('d', self.num_servos)
        self.newAction = mp.Event()
        self.error = mp.Event()
        self.overheat = mp.Event()
        self.servoActive = mp.Event()
        self.servoReset = mp.Event()
        try:
            print("\nInitializing servos...")
            self.servos = mp.Process(target=ServoController,
                                     args=(self.botActive,
                                           servos,
                                           self.servoData,
                                           self.newAction,
                                           self.action,
                                           self.servoLock,
                                           self.error,
                                           self.overheat,
                                           self.servoActive,
                                           self.servoReset))
            self.servos.start()

            sleep_time = 0
            while not self.servoActive.is_set():
                if sleep_time >= TIMEOUT*2:
                    print("Failed to initalize servos.")
                    self.close()
                else: 
                    time.sleep(0.1)
                    sleep_time += 0.1
        except Exception as e:
            print(e)
            print("Failed to initalize servos.")
            self.close()

        # initialize foot sensors
        self.adcLock = mp.Lock()
        self.adcData = mp.Array('d', 4)
        self.adcActive = mp.Event()
        if self.num_sensors > 0 and self.num_sensors <= 4:
            try:
                print("\nInitializing contact sensors...")
                self.contact = mp.Process(target=GetContactData, 
                                          args=(self.botActive,
                                                self.adcData,
                                                self.adcLock,
                                                self.adcActive))
                self.contact.start()
                sleep_time = 0
                while not self.adcActive.is_set():
                    if sleep_time >= TIMEOUT:
                        print("Failed to initalize contact sensors.")
                        self.close()
                    else: 
                        time.sleep(0.1)
                        sleep_time += 0.1
            except Exception as e:
                print(e)
                print("Failed to initalize contact sensors.")
                self.close()

        print("\nInitializing stereo camera...")
        # start ZED camera process
               
        self.camLock = mp.Lock()
        self.posEst = mp.Array('d', 13)
        self.camActive = mp.Event()
        self.failedStart = mp.Event()
        self.resetPos = mp.Event()
        try:
            self.tracker = mp.Process(target=GetZEDPosition, 
                                      args=(self.botActive,
                                            self.posEst,
                                            self.camLock,
                                            self.camActive,
                                            self.failedStart,
                                            self.resetPos))
            self.tracker.start()
            sleep_time = 0
            while not self.camActive.is_set():
                if sleep_time >= TIMEOUT:
                    print("Failed to initalize stereo camera.")
                    self.close()
                else: 
                    time.sleep(0.1)
                    sleep_time += 0.1
        except Exception as e:
            print(e)
            print("Failed to initalize stereo camera.")
            self.close()

        print("\nAll systems nonimal\n")    

    def observeState(self):
        # get camera position
        #obsStart2 = time.monotonic()
        self.camLock.acquire()
        cameraState = self.posEst[:]
        self.camLock.release()
        #obsStop2 = time.monotonic()
        #obsTime2 = obsStop2-obsStart2

        # get servo data
        #obsStart0 = time.monotonic()
        self.servoLock.acquire()
        servoState = self.servoData[:]
        self.servoLock.release()
        #obsStop0 = time.monotonic()
        #obsTime0 = obsStop0-obsStart0
        
        # get force sensor data
        #obsStart1 = time.monotonic()
        self.adcLock.acquire()
        contact = self.adcData[:]
        self.adcLock.release()
        #obsStop1 = time.monotonic()
        #obsTime1 = obsStop1-obsStart1

        #print("Servo Time: ", obsTime0) #, "\tSensor Time: ", obsTime1, "\tCamera Time: ", obsTime2)
        return np.concatenate((cameraState, servoState, contact))

    def reset(self):
        #print("\nReset called\n")

        while self.overheat.is_set():
                time.sleep(1.0)

        timer = 0
        if self.error.is_set() or not self.servoActive.is_set():
            print("Waiting for servos...")
            while self.error.is_set():            
                time.sleep(0.1)
                timer += 0.1
                if timer >= TIMEOUT:
                    print("Failed to reboot servos.")
                    self.close()

        self.servoReset.set()
        time.sleep(0.5)
        
        timer = 0
        if not self.camActive.is_set():
            print("Waiting for camera...")
            while not self.camActive.is_set(): 
                time.sleep(0.1)
                timer += 0.1
                if timer >= TIMEOUT:
                    print("Failed to reboot camera. Relaunching process...")
                    self.tracker.terminate()
                    time.sleep(0.5)
                    try:
                        self.tracker = mp.Process(target=GetZEDPosition, 
                                                  args=(self.botActive,
                                                        self.posEst,
                                                        self.camLock,
                                                        self.camActive,
                                                        self.failedStart,
                                                        self.resetPos))
                        self.tracker.start()
                        sleep_time = 0
                        while not self.camActive.is_set():
                            if self.failedStart.is_set():
                                self.failedStart.clear()
                                print("Failed to initalize stereo camera. Relaunching process...")
                                self.tracker.terminate()
                                time.sleep(0.5)
                                try:
                                    self.tracker = mp.Process(target=GetZEDPosition, 
                                                              args=(self.botActive,
                                                                    self.posEst,
                                                                    self.camLock,
                                                                    self.camActive,
                                                                    self.failedStart,
                                                                    self.resetPos))
                                    self.tracker.start()
                                    while not self.camActive.is_set():
                                        if self.failedStart.is_set():
                                            print("Failed to initalize stereo camera.")
                                            self.close()
                                        else: 
                                            time.sleep(0.1)
                                            sleep_time += 0.1
                                except Exception as e:
                                    print(e)
                            else: 
                                time.sleep(0.1)
                                sleep_time += 0.1
                    except Exception as e:
                        print(e)
                        print("Failed to initalize stereo camera.")
                        self.close()
                    
        print("Robot paused. Press 'Enter' to continue.")
        x = input()

        self.resetPos.set()
        time.sleep(0.5)
        obs = self.observeState()

        #print(obs[:self.obs_size])
        return obs[:self.obs_size]

    def close(self):
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
        try:
            self.servos.join()
            self.servos.terminate()
        except Exception as e:
            print(e)
        quit()




   


