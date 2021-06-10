clear all
close all
clc

%% Test Lidar Mapping (INIT)
numPts = 100;
lookaheadTime = 2.5;
control_timeStep = 0.001;
data_timeStep = 0.01;

lidar_data_ct = readmatrix('fullcar_data/lidar_data_truth.txt');
dataLength = data_timeStep/control_timeStep;
while dataLength < length(lidar_data_ct)/numPts
    dataLength = dataLength + data_timeStep/control_timeStep;
end
dataLength = dataLength - data_timeStep/control_timeStep;

lidar_data_ct = lidar_data_ct(1:dataLength*numPts,:);
imu_data_ct = readmatrix('fullcar_data/imu_data_truth.txt');
imu_data_ct = imu_data_ct(1:dataLength,:);
gps_data_ct = readmatrix('fullcar_data/gps_data_truth.txt');
gps_data_ct = gps_data_ct(1:dataLength,:);
wheel_pos_data_ct = readmatrix('fullcar_data/wheel_data.txt');
wheel_pos_data_ct = wheel_pos_data_ct(1:dataLength,:);

%% Downsample
lidar_data = zeros(length(lidar_data_ct)/(data_timeStep/control_timeStep),7);
imu_data = zeros(length(imu_data_ct)/(data_timeStep/control_timeStep),4);
gps_data = zeros(length(gps_data_ct)/(data_timeStep/control_timeStep),4);
wheel_pos_data = zeros(length(wheel_pos_data_ct)/(data_timeStep/control_timeStep),3);
wheel_speed_data = zeros(length(wheel_pos_data),3);

lidar_data(1:numPts,:) = lidar_data_ct(1:numPts,:);
imu_data(1,:) = imu_data_ct(1,:);
gps_data_ct(1,:) = gps_data_ct(1,:);
wheel_pos_data(1,:) = wheel_pos_data(1,:);
for ii=2:length(imu_data)
    lidar_data((ii-1)*numPts+1:ii*numPts,:) = lidar_data_ct((1:numPts)+(ii-1)*(data_timeStep/control_timeStep)*numPts,:);
    imu_data(ii,:) = imu_data_ct((ii-1)*(data_timeStep/control_timeStep)+1,:);
    gps_data(ii,:) = gps_data_ct((ii-1)*(data_timeStep/control_timeStep)+1,:);
    wheel_pos_data(ii,:) = wheel_pos_data_ct((ii-1)*(data_timeStep/control_timeStep)+1,:);
end

for ii=2:length(wheel_pos_data)
    wheel_speed_data(ii,1) = eulerDerivative(wheel_pos_data(ii,1),wheel_pos_data(ii-1,1),wheel_pos_data(ii,3)-wheel_pos_data(ii-1,3));
    wheel_speed_data(ii,2) = eulerDerivative(wheel_pos_data(ii,2),wheel_pos_data(ii-1,2),wheel_pos_data(ii,3)-wheel_pos_data(ii-1,3));
    wheel_speed_data(ii,3) = wheel_pos_data(ii,3);
end

%% Run Loop
figure(1)
T = zeros(numPts*lookaheadTime/data_timeStep,7);
lidar_x = zeros(numPts*2*lookaheadTime/data_timeStep,1);
lidar_y = zeros(numPts*2*lookaheadTime/data_timeStep,1);
lidar_z = zeros(numPts*2*lookaheadTime/data_timeStep,1);

lookahead_dataL = zeros(length(imu_data),1+1/control_timeStep);
lookahead_dataR = zeros(length(imu_data),1+1/control_timeStep);

for ii=2:length(imu_data)
    l_d_ii = lidar_data((ii-1)*numPts+1:ii*numPts,:);
    
    T = lidarMapping(T, l_d_ii, imu_data(ii,:), gps_data(ii,:),gps_data(ii-1,:), numPts);
    [lidar_x,lidar_y,lidar_z] = lidarToXYZ(lidar_x,lidar_y,lidar_z,T(length(T)-numPts+1:length(T),:));
    
    [Lz,Rz,Timez,Lxy,Rxy] = lookahead(lidar_z,lidar_x,lidar_y,wheel_speed_data(ii,1),wheel_speed_data(ii,2),0,0,gps_data(ii,:),imu_data(ii,:));
    
    lookahead_dataL(ii,:) = Lz;
    lookahead_dataR(ii,:) = Rz;
    
    scatter3(T(:,3),T(:,1),T(:,2),100,'.')
    hold on
    scatter3(T(:,6),T(:,4),T(:,5),100,'.')
    scatter3(Lxy(1,:),Lxy(2,:),Lz,1000,'.')
    scatter3(Rxy(1,:),Rxy(2,:),Rz,1000,'.')
    hold off
    %xlim([-1,22])
    %ylim([-6,6])
    zlim([-1.2,-0.6])
    xlabel('x')
    ylabel('y')
    zlabel('z')
    F(ii) = getframe(gcf);
    drawnow;
    
end

%% Write To Video
writerObj = VideoWriter('pipeline_noisy_step.avi');
writerObj.FrameRate = 100;
% set the seconds per image
% open the video writer
open(writerObj);
% write the frames to the video
for ii=2:length(F)
    % convert the image to a frame
    frame = F(ii);    
    writeVideo(writerObj, frame);
end
% close the writer object
close(writerObj);

%% Upsample and Save
lookahead_dataL_upsampled = zeros((data_timeStep/control_timeStep)*size(lookahead_dataL,1),1+1/control_timeStep);
lookahead_dataR_upsampled = zeros((data_timeStep/control_timeStep)*size(lookahead_dataR,1),1+1/control_timeStep);
for ii=1:size(lookahead_dataL,1)
    for dataLength=1:(data_timeStep/control_timeStep)
        lookahead_dataL_upsampled((ii-1)*(data_timeStep/control_timeStep)+dataLength) = lookahead_dataL(ii);
        lookahead_dataR_upsampled((ii-1)*(data_timeStep/control_timeStep)+dataLength) = lookahead_dataR(ii);
    end
end

save('lookahead_dataL.txt','lookahead_dataL','-ascii');
save('lookahead_dataR.txt','lookahead_dataR','-ascii');
save('lookahead_dataL_upsampled.txt','lookahead_dataL_upsampled','-ascii');
save('lookahead_dataR_upsampled.txt','lookahead_dataR_upsampled','-ascii');

save('pipeline_data.mat');
























