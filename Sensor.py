# -*- coding: utf-8 -*-
"""
ECE183DB Sensor Classes

Created on Thu Apr 15 15:35:02 2021

@author: Ethan Uetrecht
"""

import numpy as np

# Basic sensor class, can be initialized with various properties, can 'read' values (apply noise to and discretize input truth values)
class Sensor:
    resolution = 0
    valueBounds = [0,0]
    valueSpace = 0
    rounding = ''
    fullscale = 0
    noiseVariance = 0
    units = ''
    
    def __init__(self, resolution, valueBounds, roundingType, noiseVariance, units):
        self.resolution = resolution
        self.valueBounds = valueBounds
        self.rounding = roundingType
        if(self.rounding not in ['floor', 'ceil', 'avg']):
            raise ValueError('Invalid rounding type')
        self.noiseVariance = noiseVariance
        self.units = units
        self.fullscale = self.valueBounds[1] - self.valueBounds[0]
        self.valueSpace = np.linspace(self.valueBounds[0], self.valueBounds[1], self.resolution)
    
    def apply_resolution(self, continuous_value):
        discrete_value = 0
        for i in np.arange(self.resolution):
            if(self.valueSpace[i] > continuous_value and i > 0):
                if(self.rounding == 'floor'):
                    discrete_value = self.valueSpace[i-1]
                    break
                elif(self.rounding == 'ceil'):
                    discrete_value = self.valueSpace[i]
                    break
                elif(self.rounding == 'avg'):
                    discrete_value = (self.valueSpace[i] + self.valueSpace[i-1])/2
                    break
        if(self.valueSpace[-1] < continuous_value):
            discrete_value = self.valueSpace[-1]
        elif(self.valueSpace[0] > continuous_value):
            discrete_value = self.valueSpace[0]
        return discrete_value
    
    def read_value(self, true_value):
        noisy_value = true_value + np.random.normal(loc=0.0, scale=self.noiseVariance)
        value = self.apply_resolution(noisy_value)
        #value = noisy_value
        return value, self.units
    
    def check_units(self):
        return self.units

# Unique: less default noise, value bounds of [-pi,pi]
class IMU(Sensor):
    def __init__(self, resolution=1024, valueBounds=[-np.pi,np.pi], roundingType='floor', noiseVariance=0.001, units='radians'):
        Sensor.__init__(self, resolution, valueBounds, roundingType, noiseVariance, units)
        
    def read_value(self, true_value):
        return Sensor.read_value(self,true_value)
    
    def check_units(self):
        return Sensor.check_units(self)

# Unique: actuator type selectable, some unit-actuator matching
class ActuatorPositionSensor(Sensor):
    actuatorType = ''
    
    def __init__(self, resolution=1024, valueBounds=[0,2*np.pi], roundingType='floor', noiseVariance=0.01, units='radians', actuatorType='rotational'):
        Sensor.__init__(self, resolution, valueBounds, roundingType, noiseVariance, units)
        if(actuatorType != 'linear' and actuatorType != 'rotational'):
            raise ValueError('Invalid actuator type')
        if((self.units == 'radians' or self.units == 'degrees') and actuatorType != 'rotational'):
            raise ValueError('Units do not match actuator type')
        
    def read_value(self, true_value):
        return Sensor.read_value(self,true_value)
    
    def check_units(self):
        return Sensor.check_units(self)

# Unique: angular position rollover accounted for, previous value used for speed calculation
class WheelSpeedSensor(Sensor):
    previousPosition = 'n/a'
    previousTime = 'n/a'
    previousSpeed = 'n/a'
    
    def __init__(self, resolution=1024, valueBounds=[0,2*np.pi], roundingType='floor', noiseVariance=0.01, units='radians/s'):
        Sensor.__init__(self, resolution, valueBounds, roundingType, noiseVariance, units)
        
    def rollover_value(self, value):
        while(value > self.valueSpace[-1]):
            value -= self.fullscale
        while(value < self.valueSpace[0]):
            value += self.fullscale
        return value
    
    def read_value(self, true_position, current_time):
        bounded_position = self.rollover_value(true_position)
        position, units = Sensor.read_value(self,bounded_position)
        if(self.previousPosition == 'n/a'):
            self.previousPosition = position
            self.previousTime = current_time
            speed = 0
            self.previousSpeed = speed
            return speed, units
        else:
            if(position >= self.previousPosition and self.previousSpeed < 0):
                speed = (position - self.fullscale - self.previousPosition)/(current_time - self.previousTime)
            elif(position < self.previousPosition and self.previousSpeed >= 0):
                speed = (position + self.fullscale - self.previousPosition)/(current_time - self.previousTime)
            else:
                speed = (position - self.previousPosition)/(current_time - self.previousTime)
        self.previousSpeed = speed
        return speed
    
    def check_units(self):
        return Sensor.check_units(self)




