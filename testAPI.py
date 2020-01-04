# -*- coding: utf-8 -*-
"""
Created on Sun Dec  1 18:57:59 2019

@author: Anton
"""

import airsim
import time
import numpy as np


client = airsim.CarClient()
client.confirmConnection()
client.enableApiControl(True)
car_controls = airsim.CarControls()

client.reset()
#time.sleep(0.001)
#=============================================================================
# go reverse
car_controls.brake = 100
client.setCarControls(car_controls)
time.sleep(5)
car_controls.brake = 0
car_controls.throttle = -1
car_controls.is_manual_gear = True;
car_controls.manual_gear = -1
car_controls.steering = -1
client.setCarControls(car_controls)
print("Go reverse, steer left")
time.sleep(5)   # let car drive a bit
car_controls.is_manual_gear = False; # change back gear to auto
car_controls.manual_gear = 0  

# apply breaks
car_controls.brake = 1
client.setCarControls(car_controls)
print("Apply break")
time.sleep(3)   # let car drive a bit
car_controls.brake = 0 #remove break
#=============================================================================
# turn left
car_controls.throttle = 1
car_controls.steering = 1
client.setCarControls(car_controls)
time.sleep(6)

# steering 0
car_controls.steering = 0
client.setCarControls(car_controls)
time.sleep(5)

# turn slightly right
car_controls.steering = 0.7
client.setCarControls(car_controls)
time.sleep(2)

# steering 0
car_controls.steering = 0
client.setCarControls(car_controls)
time.sleep(3)

x = np.arange(0,2*np.pi,0.01)
y = np.cos(x)

# ============================================================================

#=============================================================================

i = 0
j = 0
while True:
    if i == len(y)-1:
        i = 0
    i += 1
    if j > len(y)*1:
        car_controls.steering = 0
        car_controls.brake = 100
        client.setCarControls(car_controls)
        print("Car stopped")
        break
    j += 1
    car_controls.steering = y[i]
    client.setCarControls(car_controls)
    time.sleep(0.01)


car_controls.brake = 0
car_controls.steering = 1
car_controls.throttle = 1
client.setCarControls(car_controls)
time.sleep(3)
car_controls.steering = 0
client.setCarControls(car_controls)