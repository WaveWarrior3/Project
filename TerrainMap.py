import numpy as np
from math import sin, cos, sqrt
import matplotlib.pyplot as plt


def read_file(filename):
    f = open(filename,'r')
    temp = f.readlines()
    data = []
    for i in range(len(temp)):
        temp[i].replace("\n", "")
        data.append(temp[i].split())
    
    for i in range(len(data)):
        for j in range(len(data[i])):
            data[i][j] = float(data[i][j])
    
    return data


def OrganizeTimestamp(lidar, gps, imu):
    data = []
    for time in range(len(gps)):
        curr_data = []
        curr_data.append(gps[time])
        curr_data.append(imu[time])
        for i in range(len(lidar)):
            if lidar[i][-1] == gps[time][-1]:
                curr_data.append(lidar[i])
        data.append(curr_data)
    return data

def ConvertLidarToTerrain(data, points):
    # takes in list of data at a single timestamp
    gps = data[0]
    imu = data[1]

    pos_x = gps[0]
    pos_y = gps[1]
    pos_z = gps[2]

    rot_x = imu[0]
    rot_y = imu[1]
    rot_z = imu[2]

    if abs(rot_y) > 0.01:
        return

    for i in range(2, len(data)):
        lidar = data[i]
        lidar_x1 = lidar[0]
        lidar_y1 = lidar[2] / sqrt(2)
        lidar_z1 = lidar[2] / sqrt(2)

        lidar_x2 = lidar[3]
        lidar_y2 = lidar[5] / sqrt(2)
        lidar_z2 = lidar[5] / sqrt(2)

        P1 = [lidar_x1*sin(rot_x) - lidar_z1*cos(rot_x) + pos_x + 0.051, lidar_y1 + pos_y, lidar_x1*cos(rot_x) + lidar_z1*sin(rot_x) + pos_z + 0.05]
        P2 = [lidar_x2*sin(rot_x) - lidar_z2*cos(rot_x) + pos_x + 0.051, lidar_y2 + pos_y, lidar_x2*cos(rot_x) + lidar_z2*sin(rot_x) + pos_z - 0.05]
        points.append(P1)
        points.append(P2)

    return


def PlotPoints(points):
    x = []
    y = []
    z = []

    for i in range(len(points)):
        x.append(points[i][0])
        y.append(points[i][1])
        z.append(points[i][2])

    fig = plt.figure(figsize = (10, 7))
    ax = plt.axes(projection = "3d")
    ax.scatter3D(x, z, y, color = "green") # change order of y and z so the axes match our model
    ax.set_xlabel("x")
    ax.set_ylabel("z")
    ax.set_zlabel("y")
    plt.show()


def main():
    lidar = read_file('Good Data/lidar_data.txt')
    gps = read_file('Good Data/gps_data.txt')
    imu = read_file('Good Data/imu_data.txt')

    data = OrganizeTimestamp(lidar, gps, imu)
    points = []
    for i in range(len(data)):
        ConvertLidarToTerrain(data[i], points)
    #print(data[0])
    PlotPoints(points)

if __name__ == "__main__":
    main()