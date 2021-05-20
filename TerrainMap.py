import numpy as np
from math import sin, cos, sqrt, atan2, acos, asin
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


    for i in range(2, len(data)):
        # read in x,y,z data
        lidar = data[i]
        lidar_x1 = lidar[0] + 0.05
        lidar_y1 = lidar[2] / sqrt(2)
        lidar_z1 = (lidar[2] / sqrt(2)) - 0.051
        

        lidar_x2 = lidar[3] - 0.05
        lidar_y2 = lidar[5] / sqrt(2)
        lidar_z2 = (lidar[5] / sqrt(2)) - 0.051


        # convert coordinates
        P1 = [-1*lidar_z1, -1*lidar_x1, lidar_y1]
        P2 = [-1*lidar_z2, -1*lidar_x2, lidar_y2]

        # apply pitch
        pitch1 = [cos(-1*rot_y)*P1[0] + sin(-1*rot_y)*P1[2], P1[1], -1*sin(-1*rot_y)*P1[0] + cos(-1*rot_y)*P1[2]]
        pitch2 = [cos(-1*rot_y)*P2[0] + sin(-1*rot_y)*P2[2], P2[1], -1*sin(-1*rot_y)*P2[0] + cos(-1*rot_y)*P2[2]]
        
        # apply roll
        roll1 = [cos(rot_x)*pitch1[0] - sin(rot_x)*pitch1[1], sin(rot_x)*pitch1[0] + cos(rot_x)*pitch1[1], pitch1[2]]
        roll2 = [cos(rot_x)*pitch2[0] - sin(rot_x)*pitch2[1], sin(rot_x)*pitch2[0] + cos(rot_x)*pitch2[1], pitch2[2]]

        # apply yaw
        yaw1 = [roll1[0], cos(rot_z)*roll1[1] -sin(rot_z)*roll1[2], sin(rot_z)*roll1[1] + cos(rot_z)*roll1[2]]
        yaw2 = [roll1[0], cos(rot_z)*roll2[1] -sin(rot_z)*roll2[2], sin(rot_z)*roll2[1] + cos(rot_z)*roll2[2]]
        

        # convert coordinates to global and add offsets
        # P1 = [pitch1[0] + pos_x, pitch1[2] + pos_y, -1*pitch1[1] + pos_z]
        # P2 = [pitch2[0] + pos_x, pitch2[2] + pos_y, -1*pitch2[1] + pos_z]
        P1 = [yaw1[0] + pos_x, yaw1[2] + pos_y, -1*yaw1[1] + pos_z]
        P2 = [yaw2[0] + pos_x, yaw2[2] + pos_y, -1*yaw2[1] + pos_z]
        
        points.append(P1)
        points.append(P2)        
    return


def PlotPoints(points):
    x = []
    y = []
    z = []

    x2 = []
    y2 = []
    z2 = []

    for i in range(len(points)):
        x.append(points[i][0])
        y.append(points[i][1])
        z.append(points[i][2])
        x2.append(str(points[i][0]) + ", ")
        y2.append(str(points[i][1]) + ", ")
        z2.append(str(points[i][2]) + ", ")
    
    fig = plt.figure(figsize = (10, 7))
    ax = plt.axes(projection = "3d")
    ax.scatter3D(x, z, y) # change order of y and z so the axes match our model
    ax.set_xlabel("x")
    ax.set_ylabel("z")
    ax.set_zlabel("y")
    plt.show()

    return x2, y2, z2
    


def write_points(data, name):
    f = open(name, 'w')
    f.writelines(data)
    f.close()



def main():
    lidar = read_file('Good Data/lidar_data.txt')
    gps = read_file('Good Data/gps_data.txt')
    imu = read_file('Good Data/imu_data.txt')

    data = OrganizeTimestamp(lidar, gps, imu)
    points = []
    for i in range(len(data)):
        ConvertLidarToTerrain(data[i], points)
    #print(data[0])
    x, y, z = PlotPoints(points)
    

if __name__ == "__main__":
    main()