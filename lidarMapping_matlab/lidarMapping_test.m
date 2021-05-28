clear all
close all
clc

%% Test Lidar Mapping (INIT)
lidar_data = readmatrix('fullcar_data/lidar_data.txt');
imu_data = readmatrix('fullcar_data/imu_data.txt');
gps_data = readmatrix('fullcar_data/gps_data.txt');
numPts = 100;

lookaheadTime = 1;
data_timeStep = 0.002;

%% Run Loop
figure(1)
T = zeros(numPts*lookaheadTime/data_timeStep,7);
for ii=1:length(imu_data)
    T = lidarMapping(T, lidar_data((ii-1)*numPts+1:ii*numPts,:), imu_data(ii,:), gps_data(ii,:), numPts);
    scatter3(T(:,3),T(:,1),T(:,2),100,'.')
    hold on
    scatter3(T(:,6),T(:,4),T(:,5),100,'.')
    hold off
    %xlim([-1,22])
    %ylim([-6,6])
    %zlim([-2,1])
    xlabel('x')
    ylabel('y')
    zlabel('z')
    drawnow;
end
