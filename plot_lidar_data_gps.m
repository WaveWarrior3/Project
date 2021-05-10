clc; clear all;

dataL = readmatrix('lidar_data.txt');
dataL(:,4) = [];
dataL(:,4) = [];
dataL(:,4) = [];

dataR = readmatrix('lidar_data.txt');
dataR(:,1) = [];
dataR(:,1) = [];
dataR(:,1) = [];

dataL = [-dataL(:,3) dataL(:,2) dataL(:,1) dataR(:,4)];

gps = readmatrix('gps_data.txt');
for i = 1:15312
   for j = 1:348
       if(dataL(i,4) == gps(j,4))
           dataL(i,1) = dataL(i,1) + gps(j,1);
           dataL(i,2) = dataL(i,2) + gps(j,2);
           dataL(i,3) = dataL(i,3) + gps(j,3);
           break
       end
   end
end

figure(1)
scatter3(dataL(:,1),-dataL(:,3),dataL(:,2),10,'blue')

figure(3)
scatter3(gps(:,1),-gps(:,3),gps(:,2),10,'blue')