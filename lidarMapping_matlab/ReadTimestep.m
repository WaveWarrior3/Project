function [x, y, z] = ReadTimestep(lidar_data)
% take in all the data from a single timestep, and break it apart into x,
% y, and z points
% lidar data is a Nx7 matrix.  Each row is x1,y1,z1,x2,y2,z2,t.  N is the
% number of readings per timestep

x1 = lidar_data(:,1);
y1 = lidar_data(:,2);
z1 = lidar_data(:,3);
x2 = lidar_data(:,4);
y2 = lidar_data(:,5);
z2 = lidar_data(:,6);

x = [x1
    x2];
y = [y1
    y2];
z = [z1
    z2];


end