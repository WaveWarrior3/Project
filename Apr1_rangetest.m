clc; clear all;

v = 15; %m/s, speed of car
h = 0.6; %m, height of sensor
theta = 15*(pi/180); %angle of sensor

x = linspace(1,100,1001);
ground = zeros(1,1001);
for i = 300:600
   ground(i) = -0.3;
end

readings = zeros(1,1001);
for i = 1:970
    height = h + ground(i);
    j = i;
    while true
        deltax = x(j+1) - x(j);
        deltay = deltax*tan(theta);
        height = height - deltay;
        if height <= ground(j)
            readings(i) = sqrt((h - height)^2 + (x(j) - x(i))^2);
            break
        end
        j = j + 1;
    end
end

d = h/sin(theta);
horiz = h/tan(theta);

calcs = zeros(1,1001);
%data = readings - d;
data = readings;
data = data * sin(15*pi/180);
data = data - 0.6;
%for i = 1:970
%    
%end

figure(1)
plot(x,ground)
title('Ground Level')
xlabel('Distance (m)')
ylabel('Depth (m)')
ylim([-0.5 0.5])

figure(2)
plot(x,readings)

figure(3)
plot(x,data)
xlim([0 95])
title('Sensor Interpretation of Dip and Rise in Ground')
xlabel('Distance (m)')
ylabel('Upcoming Change in Ground Height (m)')