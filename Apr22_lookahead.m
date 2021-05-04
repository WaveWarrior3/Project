clc; clear all;

%Wheel Speed Lookahead
R = 0.34;
w = 1.63;
dt = 0.001;

time = linspace(0,10,10001);
Lpos = zeros(2,10001);
Rpos = zeros(2,10001);
pos = zeros(2,10001);

%thetaL = 0;
omegaL = 1;
%thetaR = 0;
omegaR = 1.4;

theta = 0;
x = -2.5;
y = 0;

for i = 1:10001
    %thetaL = thetaL + omegaL*dt;
    %thetaR = thetaR + omegaR*dt;
    
    xdot = R*(omegaR+omegaL)*cos(theta)/2;
    ydot = R*(omegaR+omegaL)*sin(theta)/2;
    omega = R*(omegaR-omegaL)/w;
    
    x = x + xdot*dt;
    y = y + ydot*dt;
    theta = theta + omega*dt;
    
    pos(1,i) = x;
    pos(2,i) = y;
    
    Lpos(1,i) = x-sin(theta)*w/2;
    Lpos(2,i) = y+cos(theta)*w/2;
    
    Rpos(1,i) = x+sin(theta)*w/2;
    Rpos(2,i) = y-cos(theta)*w/2;
end

figure(1)
scatter(-pos(2,:),pos(1,:),'red'); hold on; scatter(-Lpos(2,:),Lpos(1,:),'blue'); hold on; scatter(-Rpos(2,:),Rpos(1,:),'green')

pltpos = round(pos,1);
test = pltpos(1,:).*exp(-pltpos(1,:).^2-pltpos(2,:).^2);

figure(2)
ecks = -2.5:0.1:2.5;
[X,Y] = meshgrid(ecks);
F = X.*exp(-X.^2-Y.^2);
surf(X,Y,F);hold on;scatter3(pltpos(1,:),pltpos(2,:),test,'red','filled')