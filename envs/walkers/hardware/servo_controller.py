import time
import numpy as np
from envs.walkers.hardware.dynamixel_xseries import XSeries

THERMAL_LIMIT = 75 # celsius
COOLDOWN_TIME = 20 # minutes

def enableServos(servos):
    for i in range(servos.num_servos):
        id= servos.ids[i]
        servos.setTorque(id, 1)

def disableServos(servos):
    for i in range(servos.num_servos):
        id = servos.ids[i]
        servos.setTorque(id, 0)

def rebootServos(servos, error):
    for i in range(servos.num_servos):
        if error[i] != 0:
            servos.reboot(servos.ids[i])
            time.sleep(0.2)

def ServoController(botActive, params, state, newAction, action, memLock, error, overheat, active, reset):

    servos = XSeries(params)
    active.set()
    error_count = 0

    while(botActive.is_set()): 

        if newAction.is_set():
            memLock.acquire()
            target = action[:]
            memLock.release()
            servos.writeAction(target)
            newAction.clear()
        
        #start = time.monotonic()
        try:
            obs, err, temp, fail = servos.readState()
        except Exception as e:
            print(e)
        #stop = time.monotonic()

        #obsTime = stop-start
        #print(obsTime)

        if not fail:
            memLock.acquire()
            state[:] = np.ndarray.flatten(obs.T)
            memLock.release()
            error_count = 0
        else:
            error_count += 1
            if error_count >= 10:
                active.clear()
                servos.close()
                print("Rebooting servo controller...")
                time.sleep(0.5)
                servos = XSeries(params)
                active.set()

        if any(err) != 0:
            print("\nError: Hardware error detected, rebooting servos.\n")
            error.set()
            rebootServos(servos, err)
            time.sleep(0.5)
            error.clear()
            print("Servos: Online")

        if any(temp > THERMAL_LIMIT):
            overheat.set()
            newAction.clear()
            print("\nError: Servos too hot, beginning cooldown cycle.\n")
            disableServos(servos)
            for i in range(COOLDOWN_TIME):
                print(COOLDOWN_TIME-i, "minutes remaining.")
                time.sleep(60)
            print("Cooldown cycle complete")
            enableServos(servos)
            overheat.clear()

        if reset.is_set():
            servos.moveToStart()
            reset.clear()

    servos.close()