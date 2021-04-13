from math import sin, cos
import numpy as np

def readingTo3D(distance, angle_vert, angle_horiz):
    return (distance*sin(angle_vert)*cos(angle_horiz), distance*sin(angle_horiz), distance*cos(angle_vert)*cos(angle_horiz))


def PointCloud(LIDAR, parameters):
    '''
    LIDAR is a 2 dimensional array of sensor readings
    parameters is a tuple of (maxAngle, resolution, restAngle)

    restAngle is the angle above straight down where the center of the LIDAR sensing is
    returns (res x res x 3) matrix where [y][z][:] returns (xdist, ydist, zdist) at the given y=horiz_angle, z=vert_angle


    signs: -z is rotating toward ground, +z is rotating toward sky
           -y is rotating right, +y is rotating left
    '''


    maxAngle = parameters[0]
    res = parameters[1]
    restAngle = parameters[2]

    distanceReadings = np.zeros((res, res, 3), dtype=float)

    for y in range(res):
        for z in range(res):
            horiz_angle = (-1*maxAngle)+((2*float(maxAngle)/float(res))*y)
            vert_angle = (-1*maxAngle)+((2*float(maxAngle)/float(res))*z) + restAngle
            values = readingTo3D(LIDAR[y][z], vert_angle, horiz_angle)
            distanceReadings[y][z][0] = values[0]
            distanceReadings[y][z][1] = values[1]
            distanceReadings[y][z][2] = values[2]

    return distanceReadings