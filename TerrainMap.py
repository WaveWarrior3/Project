import numpy as np


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



def main():
    lidar = read_file('lidar_data.txt')
    gps = read_file('gps_data.txt')
    imu = read_file('imu_data.txt')

    data = OrganizeTimestamp(lidar, gps, imu)
    print(data[0])

if __name__ == "__main__":
    main()