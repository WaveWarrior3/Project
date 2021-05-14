clc; clear; close all;
% EVERYTHING IS IN SI


% PHYSICAL PARAMETERS

% masses
ms = 625;                   % kg
mu = 25;                    % kg

% shock
ks = 15500;                 % N/m
bs = 2*sqrt(ms*ks);         % N-s/m

% tire
ku = 10*ks;                 % N/m
bu = 2*sqrt(mu*ks);         % N-s/m

% moments of inertia
Jxx = ms/12*(2.5^2 + 2^2);  % kg-m^2    +x pointing right
Jyy = ms/12*(2^2 + 2^2);    % kg-^2     +y pointing forward

% geometry
beta = 2.5;                 % m
tau = 2;                    % m

% generalized inverse masses
xi = 1/ms + tau^2/(4*Jyy) + beta^2/Jxx;
xibar = 1/ms - tau^2/(4*Jyy) + beta^2/Jxx;

% PLANT MODEL
% x = [ysR dysR yuR dyuR ysL dysL yuL dyuL theta]'
% y = [eta phi theta]'
% u = [lR lL]'
% w = [wR wL wF]'
A = [0 1 0 0 0 0 0 0 0;
    -xi*ks -xi*bs xi*ks xi*bs -xibar*ks -xibar*bs xibar*ks xibar*bs 0;
    0 0 0 1 0 0 0 0 0;
    ks/mu bs/mu -(ks+ku)/mu -(bs+bu)/mu 0 0 0 0 0;
    0 0 0 0 0 1 0 0 0;
    -xibar*ks -xibar*bs xibar*ks xibar*bs -xi*ks -xi*bs xi*ks xi*bs 0;
    0 0 0 0 0 0 0 1 0;
    0 0 0 0 ks/mu bs/mu -(ks+ku)/mu -(bs+bu)/mu 0;
    0 -1/(2*beta) 0 0 0 -1/(2*beta) 0 0 0];
Butild = [0 0;
    xi*ks xi*ks;
    0 0;
    -ks/mu -ks/mu;
    0 0;
    xi*ks xi*ks;
    0 0;
    -ks/mu -ks/mu;
    0 0];
Budottild = [0 0;
    xi*bs xi*bs;
    0 0;
    -bs/mu -bs/mu;
    0 0;
    xi*bs xi*bs;
    0 0;
    -bs/mu -bs/mu;
    0 0];
Bwtild = [0 0 0;
    0 0 0;
    0 0 0;
    ku/mu ku/mu 0;
    0 0 0;
    0 0 0;
    0 0 0;
    ku/mu ku/mu 0;
    0 0 0];
Bwdottild = [0 0 0;
    0 0 0;
    0 0 0;
    bu/mu bu/mu 0;
    0 0 0;
    0 0 0;
    0 0 0;
    bu/mu bu/mu 0;
    0 0 1/beta];
C = [1/2 0 0 0 1/2 0 0 0 0;
    1/tau 0 0 0 -1/tau 0 0 0 0;
    0 0 0 0 0 0 0 0 1];
C = eye(9);


% get rid of trailer disturbance (if we want) now w is just left and right
A = A(1:8,1:8);
Butild = Butild(1:8,:);
Budottild = Budottild(1:8,:);
Bwtild = Bwtild(1:8,1:2);
Bwdottild = Bwdottild(1:8,1:2);
C = C(1:8,1:8);

% redefine state
Bu = Butild + A*Budottild;
Bw = Bwtild + A*Bwdottild;
Du = C*Budottild;
Dw = C*Bwdottild;

% plant transfer funcitons
s = tf('s');
Pu = C/(s*eye(8) - A)*Bu + Du;
Pw = C/(s*eye(8) - A)*Bw + Dw;

Pu_red = minreal(Pu,0.5);
Pw_red = minreal(Pw,0.5);


%% ACTUATOR MODEL

% physical parameters
Ra = 0.208;                         % Ohm
Kt = 0.129;                         % N-m/A
Ke = 13.5;                          % V/kRPM
wmax = 5000;                        % RPM
Jrotor = 148.37;                    % kg-mm^2
Gam = 1/(2*pi*0.53);                % in/rad
b = 0;      % WOULD BE GOOD TO INCLUDE

% make those ^ all in SI lol:
Ke = Ke * 1/1000 * 60/(2*pi);       % V/(rad/s)
wmax = wmax * 2*pi/60;              % rad/s
Jrotor = Jrotor * (1/1000)^2;       % kg-m^2
Gam = Gam * 0.0254;                 % m/rad

kap = Kt/(b*Ra + Kt*Ke);
tau = Ra*Jrotor/(b*Ra + Kt*Ke);
G = Gam*kap/(s*(tau*s + 1));

%% MOTOR CONTROLLER
close all

% requirements
tp = 0.003;
ts = 0.01;

wd = pi/tp;
sig = 4.6/ts;
wn = sqrt(wd^2 + sig^2);
zeta = sig/wn;

Kp = wn^2*tau/kap;
Kd = (2*sig*tau - 1)/kap;

a = 0.001;
Ca = (Kd*s + Kp)/(a*s + 1);

% closed-loop system
Aclp = minreal(G*Ca/(1 + G*Ca));

% figure
% step(0.2*Aclp)
% title('rl to l')
% ylabel('actuator length [m]')
% 
% figure
% yline(wmax*Gam,'r--')
% hold on
% step(0.2*s*Aclp)
% title('rl to ldot')
% legend('velocity','max velocity')
% ylabel('actuator velocity [m/s]')
% ylim([-0.5 4.5])
% 
% figure
% step(0.2*Ca/(1 + G*Ca));
% title('rl to u [V]')

% make it MIMO
Aclp_MIMO = [Aclp 0; 0 Aclp];


%% FEEDBACK CONTROLLER
% plotdefaults(14,50,1.5,'northeast')

% generalized forward-path plant
H = Pu_red*Aclp_MIMO;
H_ss = ss(H);
n = 8;

H_ss_red = balred(H_ss,n);

AH = H_ss_red.A;
BH = H_ss_red.B;
CH = H_ss_red.C;
DH = H_ss_red.D;

% LQR controller
Q = eye(n);
r = 1e-5;
R = r*eye(2);
Kb = lqr(AH,BH,Q,R);

% closed-loop system
% might want to reduce H cuz it's still huge here
L = H*Kb;
L_red = balred(L,5);

[p,~] = size(L_red);
S = minreal((eye(p) - L_red)\eye(p));
T = minreal((eye(p) - L_red)\L_red);

figure(1)
pzplot(T)

figure(2)
step(T);
title('fuck me')