# -*- coding: utf-8 -*-
"""
Created on Thu Apr 15 17:30:18 2021

@author: uetre
"""

import numpy as np
import matplotlib.pyplot as plt
from Sensor import IMU, ActuatorPositionSensor, WheelSpeedSensor



class Actuator:
    positionBounds = [0,0]
    maxPositionDot = 0
    position = 0
    setPosition = 0
    units = ''
    actuatorType = ''
    angleToLinearMap = 0
    
    def __init__(self, positionBounds, maxPositionDot=np.Inf, initialPosition=0, initialSetPosition=0, units='m', actuatorType='linear', angleToLinearMap=1):
        self.positionBounds = positionBounds
        self.maxPositionDot = maxPositionDot
        self.position = initialPosition
        self.setPosition = initialSetPosition
        self.units = units
        self.actuatorType = actuatorType
        if(actuatorType != 'linear' and actuatorType != 'rotational'):
            raise ValueError('Invalid actuator type')
        if(actuatorType == 'linear' and angleToLinearMap != 1):
            raise ValueError('Invalid angular to linear map for linear actuator')
        self.angleToLinearMap = angleToLinearMap
    
    def set_desired_position(self, value):
        if(value > self.positionBounds[1]):
            self.setPosition = self.positionBounds[1]
            raise ValueError('Set position too high')
        elif(value < self.positionBounds[0]):
            self.setPosition = self.positionBounds[0]
            raise ValueError('Set position too low')
        else:
            self.setPosition = value
    
    def update_position(self, dt):
        error = self.setPosition - self.position
        required_speed = error/dt
        set_speed = required_speed
        if(np.abs(required_speed) > self.maxPositionDot):
            set_speed = self.maxPositionDot
        self.position += set_speed*dt
    
    def get_true_position(self):
        return self.position*self.angleToLinearMap, self.units


def main():
    dt = 0.001
    time = np.arange(0,5,dt)
    
    actuatorPositionHist = np.zeros_like(time)
    actuatorPositionHistMeas = np.zeros_like(time)
    
    a = Actuator([0,2],0.7,0,0,'m','linear',1)
    a_sens = ActuatorPositionSensor(1024,[0,2],'floor',0.01,'m','linear')
    
    for t in np.arange(len(time)):
        actuatorPositionHist[t], units_true = a.get_true_position()
        actuatorPositionHistMeas[t], units_meas = a_sens.read_value(actuatorPositionHist[t])
        if(units_true != units_meas):
            raise ValueError('Unit mismatch (simulation vs. sensor)')
        
        if(time[t] == 1):
            a.set_desired_position(2)
        
        a.update_position(dt)
    
    plt.figure(0)
    plt.plot(time, actuatorPositionHist, label='True Position')
    plt.plot(time, actuatorPositionHistMeas, label='Measured Position')
    plt.legend()

if __name__ == "__main__":
    main()
    
    
    
    
    
    
    
    
    
    