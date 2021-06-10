function [x, y, z] = lidarToXYZ(x,y,z,lidar_data)
% take in all the data from a single timestep, and break it apart into x,
% y, and z points
% lidar data is a Nx7 matrix.  Each row is x1,y1,z1,x2,y2,z2,t.  N is the
% number of readings per timestep

x_t = [lidar_data(:,1);lidar_data(:,4)];
y_t = [lidar_data(:,2);lidar_data(:,5)];
z_t = [lidar_data(:,3);lidar_data(:,6)];

numPts = length(lidar_data);

x(1:length(x)-numPts*2,:) = x(numPts*2+1:length(x),:);
x(length(x)-numPts*2+1:length(x),:) = x_t;

y(1:length(y)-numPts*2,:) = y(numPts*2+1:length(y),:);
y(length(y)-numPts*2+1:length(y),:) = y_t;

z(1:length(z)-numPts*2,:) = z(numPts*2+1:length(z),:);
z(length(z)-numPts*2+1:length(z),:) = z_t;
end