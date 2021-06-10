clear all
close all
clc

%% Load Lidar Data
lidar_data = readmatrix('fullcar_data/lidar_data_truth.txt');
numPts = 100;

%% Plot Raw Data
figure()
hold on
scatter3(lidar_data(:,1),lidar_data(:,2),lidar_data(:,3),1000,'.')
scatter3(0,0,0,1000,'.')
xlabel('x')
ylabel('y')
zlabel('z')
grid on

figure()
hold on
scatter3(lidar_data(:,4),lidar_data(:,5),lidar_data(:,6),1000,'.')
scatter3(0,0,0,1000,'.')
xlabel('x')
ylabel('y')
zlabel('z')
grid on

%% Transform 1
transform_data = lidar_data;

alpha = pi/4;

transform_data(:,2) = transform_data(:,3)*sin(alpha);
transform_data(:,5) = transform_data(:,6)*sin(alpha);

transform_data(:,3) = -transform_data(:,3)*cos(alpha);
transform_data(:,6) = -transform_data(:,6)*cos(alpha);

%% Transform 2
transform_data(:,1) = transform_data(:,1) + 0.9;
transform_data(:,4) = transform_data(:,4) - 0.9;

%% Transform 3
rpyt = readmatrix('fullcar_data/imu_data_truth.txt');

for ii=1:length(rpyt)
    r = rpyt(ii,1);
    p = rpyt(ii,2);
    y = rpyt(ii,3);
    
    B = [1 0 0; 0 cos(p) sin(p); 0 -sin(p) cos(p)]; % rotation about x = pitch
    C = [cos(y) 0 -sin(y); 0 1 0; sin(y) 0 cos(y)]; % rotation about y = yaw
    D = [cos(r) sin(r) 0; -sin(r) cos(r) 0; 0 0 1]; % rotation about z = roll
    
    R = B*C*D;
    
    %R = [cos(p)*cos(y), cos(p)*sin(y), -sin(p)
    %    sin(r)*sin(p)*cos(y)-cos(r)*sin(y), sin(r)*sin(p)*sin(y)+cos(r)*cos(y), cos(p)*sin(r)
    %    cos(r)*sin(p)*cos(y)+sin(r)*sin(y), cos(r)*sin(p)*sin(y)-sin(r)*cos(y), cos(p)*cos(r)];
    Rinv = inv(R);
    
    transform_data((ii-1)*numPts+1:ii*numPts,1:3) = (Rinv*transform_data((ii-1)*numPts+1:ii*numPts,1:3)')';
    transform_data((ii-1)*numPts+1:ii*numPts,4:6) = (Rinv*transform_data((ii-1)*numPts+1:ii*numPts,4:6)')';
end

%% Transform 4
xyzt = readmatrix('fullcar_data/gps_data_truth.txt');

for ii=1:length(xyzt)
    xyz = xyzt(ii,1:3);
    
    for jj=(ii-1)*numPts+1:ii*numPts
        transform_data(jj,1:3) = transform_data(jj,1:3) + xyz;
        transform_data(jj,4:6) = transform_data(jj,4:6) + xyz;
    end
end

z_offset = (2.22-0.76);
transform_data(:,3) = transform_data(:,3) + z_offset;
transform_data(:,6) = transform_data(:,6) + z_offset;

%% Plot Transformed Data
figure()
hold on
%C1 = [transform_data(:,2)]/max(transform_data(:,2));
%C2 = [transform_data(:,5) transform_data(:,5) transform_data(:,5)];
scatter3(transform_data(:,3),transform_data(:,1),transform_data(:,2),100,'.')
scatter3(transform_data(:,6),transform_data(:,4),transform_data(:,5),100,'.')
scatter3(xyzt(:,3),xyzt(:,1),xyzt(:,2))
xlabel('x')
ylabel('y')
zlabel('z')
grid on
zlim([-2,1])
legend('Left Lidar','Right Lidar','GPS Position')

%% Write Data to File
lidarXFileID = fopen('lidar_x.txt','w');
lidarYFileID = fopen('lidar_y.txt','w');
lidarZFileID = fopen('lidar_z.txt','w');
lidarTFileID = fopen('lidar_t.txt','w');
lidarFormatSpec = '%5.4f\n';

lidar_x = zeros(numPts*2,1);
lidar_y = zeros(numPts*2,1);
lidar_z = zeros(numPts*2,1);
lidar_t = zeros(numPts*2,1);
for ii=1:length(rpyt)
    lidar_x(1:numPts,1) = transform_data((ii-1)*numPts+1:ii*numPts,1);
    lidar_x(numPts+1:2*numPts,1) = transform_data((ii-1)*numPts+1:ii*numPts,4);
    
    lidar_y(1:numPts,1) = transform_data((ii-1)*numPts+1:ii*numPts,2);
    lidar_y(numPts+1:2*numPts,1) = transform_data((ii-1)*numPts+1:ii*numPts,5);
    
    lidar_z(1:numPts,1) = transform_data((ii-1)*numPts+1:ii*numPts,3);
    lidar_z(numPts+1:2*numPts,1) = transform_data((ii-1)*numPts+1:ii*numPts,6);
    
    lidar_t(1:numPts,1) = transform_data((ii-1)*numPts+1:ii*numPts,7);
    lidar_t(numPts+1:2*numPts,1) = transform_data((ii-1)*numPts+1:ii*numPts,7);
    
    for jj=1:numPts*2
        fprintf(lidarXFileID,lidarFormatSpec,lidar_x(jj));
        fprintf(lidarYFileID,lidarFormatSpec,lidar_y(jj));
        fprintf(lidarZFileID,lidarFormatSpec,lidar_z(jj));
        fprintf(lidarTFileID,lidarFormatSpec,lidar_t(jj));
    end
end

fclose(lidarXFileID);
fclose(lidarYFileID);
fclose(lidarZFileID);
fclose(lidarTFileID);


















