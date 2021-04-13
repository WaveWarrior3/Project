from math import sin, cos
import numpy as np

def readingTo3D(distance, angle_vert, angle_horiz):
    return (distance*sin(angle_vert)*cos(angle_horiz), distance*sin(angle_horiz), distance*cos(angle_vert)*cos(angle_horiz))


def pointCloud(LIDAR, parameters):
    '''
    LIDAR is a 2 dimensional array of sensor readings
    parameters is a tuple of (maxAngle, resolution)

    returns (res x res x 3) matrix where [y][z][:] returns (xdist, ydist, zdist) at the given y=horiz_angle, z=vert_angle
    '''


    maxAngle = parameters[0]
    res = parameters[1]

    distanceReadings = np.zeros((res, res, 3), dtype=float)

    for y in range(res):
        for z in range(res):
            horiz_angle = (-1*maxAngle)+((2*float(maxAngle)/float(res))*y)
            vert_angle = (-1*maxAngle)+((2*float(maxAngle)/float(res))*z)
            values = readingTo3D(LIDAR[y][z], vert_angle, horiz_angle)
            distanceReadings[y][z][0] = values[0]
            distanceReadings[y][z][1] = values[1]
            distanceReadings[y][z][2] = values[2]

    return distanceReadings