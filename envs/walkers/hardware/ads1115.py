import busio
import board
import time
import numpy as np
import adafruit_ads1x15.ads1115 as ADS
from adafruit_ads1x15.analog_in import AnalogIn

MAX_VOLTAGE = 3.3
PRECISION = 2**16

def GetContactData(botActive, data, memLock, active):

    # iniialize i2c port
    i2c = busio.I2C(board.SCL, board.SDA)
    # create adc object
    adc = ADS.ADS1115(i2c)
    adc.gain = 1
    print("ADC Gain: ", adc.gain)
    # create analog input channels
    A0 = AnalogIn(adc, ADS.P0)
    print("Force Sensor#%d: Online" % 0)
    A1 = AnalogIn(adc, ADS.P1)
    print("Force Sensor#%d: Online" % 1)
    A2 = AnalogIn(adc, ADS.P2)
    print("Force Sensor#%d: Online" % 2)
    A3 = AnalogIn(adc, ADS.P3)
    print("Force Sensor#%d: Online" % 3)
    active.set()
                 
    while(botActive.is_set()):
        #print(A0.value, A1.value, A2.value, A3.value)
        memLock.acquire()
        try:
            data[:] = np.array([A0.value, A1.value, A2.value, A3.value])/PRECISION
        except Exception as e:
            print(e)
        memLock.release()
        time.sleep(.01)




