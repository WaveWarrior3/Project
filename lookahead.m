function [Lz,Rz,time,Lpos,Rpos] = lookahead(xdat,ydat,zdat,omegaL,omegaR,alphaL,alphaR)
%Wheel Speed Lookahead Function
%   Looks ahead in time and projects where the wheels will be in xy-plane,
%   then uses k-nearest algorithm to estimate the heights the wheels will
%   encounter over that path.

%xdat is forward
%ydat is left
%zdat is gravity axis (up)

%CAR PARAMETERS AND K-NEAREST NUMBER AND NUMBER OF POINTS
%IF YOU WANNA CHANGE SOMETHING IT'S PROBABLY THIS
R = 0.34;
w = 1.63;
dt = 0.001;
knum = 10;
pts = 5001;

time = linspace(0,5,pts);
Lpos = zeros(2,pts);
Rpos = zeros(2,pts);
pos = zeros(2,pts);

%initial values
theta = 0; %forward, as it were
x = 0;
y = 0;

for i = 1:pts
    %thetaL = thetaL + omegaL*dt;
    %thetaR = thetaR + omegaR*dt;
    
    xdot = R*(omegaR+omegaL)*cos(theta)/2;
    ydot = R*(omegaR+omegaL)*sin(theta)/2;
    omega = R*(omegaR-omegaL)/w;
    
    x = x + xdot*dt;
    y = y + ydot*dt;
    theta = theta + omega*dt;
    
    %The acceleration update is just handled through Euler's method
    %If this proves inaccurate (it should be totally fine, though)
    %then look up Runge-Kutta or just have Andrew do it
    omegaR = omegaR + alphaR*dt;
    omegaL = omegaL + alphaL*dt;
    
    pos(1,i) = x;
    pos(2,i) = y;
    
    Lpos(1,i) = x-sin(theta)*w/2;
    Lpos(2,i) = y+cos(theta)*w/2;
    
    Rpos(1,i) = x+sin(theta)*w/2;
    Rpos(2,i) = y-cos(theta)*w/2;
end

%figure(1)
%scatter(-pos(2,:),pos(1,:),'red'); hold on; scatter(-Lpos(2,:),Lpos(1,:),'blue'); hold on; scatter(-Rpos(2,:),Rpos(1,:),'green'); hold on; scatter(xdat,ydat);
%figure(2)
%scatter(xdat,ydat)

Lz = zeros(1,pts);
Rz = zeros(1,pts);

%K NEAREST
for i = 1:pts
    xdattestL = xdat - Lpos(1,i);
    ydattestL = ydat - Lpos(2,i);
    
    xdattestR = xdat - Rpos(1,i);
    ydattestR = ydat - Rpos(2,i);
    
    distsL = sqrt(xdattestL.^2 + ydattestL.^2);
    distsR = sqrt(xdattestR.^2 + ydattestR.^2);
    [B_L,I_L] = mink(distsL,knum);
    [B_R,I_R] = mink(distsR,knum);
    
    %LEFT WHEEL
    nrm = 0;
    val = 0;
    for j = 1:knum
        nrm = nrm + 1/(0.01+B_L(j));
        val = val + zdat(I_L(j))/(0.01+B_L(j));
    end
    val = val/nrm;
    Lz(i) = val;
    
    %RIGHT WHEEL
    nrm = 0;
    val = 0;
    for j = 1:knum
        nrm = nrm + 1/(0.01+B_R(j));
        val = val + zdat(I_R(j))/(0.01+B_R(j));
    end
    val = val/nrm;
    Rz(i) = val;
end

end