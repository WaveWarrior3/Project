from controller import *
import sys

robot = Robot()

timestep = int(robot.getBasicTimeStep())

lidar = robot.getDevice('lidar')
lidar.enable(timestep)
lidar.enablePointCloud()

#val = lidar.getPointCloud()
#print(val)

while (robot.step(timestep) != -1):
    # Read the sensors, like:
    val = lidar.getPointCloud()
    print(val)