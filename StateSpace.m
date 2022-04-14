%% Boeing 747 cruising at 20000feet
%% Name: Abinay Brown
%% Email: abinay.joel@gmail.com
%% Georgia Institute of Technology
clear; clc;

M = 0.8; % Mach Number;
h = 20000;
alpha = 0;
g0 = 32.17;
W = 636636;
m = W/g0;


rho = 0.0012674; % slugs/ft^3 density at 20000ft
T = 447.4151; % Rankine at 20000ft
rho0 = 0.0023772;
gamma = 1.4;
R = 1717; %gas constant
vec = M * sqrt(gamma*R*T); % V infinity at 20000ft
u0 = vec; % trim condition forward velocity
Q = 0.5*rho*vec^2; % Dynamic Pressure

S = 5500; % wing area ft^2
c = 27.3; % chord length ft
b = S/c;  % wing span ft
theta0 = 0; % Assuming level flight
%% Moment of Inertias (slugs-ft^2)
Ix = 18.2*10^6;
Iz = 49.7*10^6;
Ixz = 0.97*10^6;
%% Aerodynamic Coefficients
Cyb = -0.81;
Clb = -0.164;
Cnb = 0.179;

Cyp = 0;
Clp = -0.315;
Cnp = 0.0028;

Cyr = 0;
Clr = 0.0979;
Cnr = -0.265;

Clda = 0.0120;
Cnda = 0.0008;
Cydr = 0.0841;
Cldr = 0.0090;
Cndr = -0.0988;
%% Stability Derivatives
Yv = (Q*S*Cyb)/(m*u0);
Yp = (Q*S*b*Cyp)/(2*m*u0);
Yr = (Q*S*b*Cyr)/(2*m*u0);

Lv = (Q*S*b*Clb)/(Ix*u0);
Lp = (Q*S*(b^2)*Clp)/(2*Ix*u0);
Lr = (Q*S*(b^2)*Clr)/(2*Ix*u0);

Nv = (Q*S*b*Cnb)/(Iz*u0);
Np = (Q*S*(b^2)*Cnp)/(2*Iz*u0);
Nr = (Q*S*(b^2)*Cnr)/(2*Iz*u0);

%% Control Derivatives
Lda = (Q*S*b*Clda)/(Ix);
Ldr = (Q*S*b*Cldr)/(Ix);

Nda = (Q*S*b*Cnda)/(Iz);
Ndr = (Q*S*b*Cndr)/(Iz);

%Yda = (Q*S*Cyda)/(m);
Ydr = (Q*S*Cydr)/(m);

%% Lateral State-Space Model

A = [Yv, Yp, g0*cos(theta0), Yr-u0;...
    Lv, Lp, 0, Lr;...
    0, 1, 0, 0;...
    Nv, Np, 0, Nr];

B = [ Ydr, 0;...
    Ldr, Lda;...
    0, 0;...
    Ndr, Nda];
C = [1, 0, 0, 0;...
     0, 1, 0, 0;...
     0, 0, 1, 0;...
     0, 0, 0, 1];
% Observe v, roll angle, yaw rate
D = zeros(4,2);    

%% Accuracy & Effort Weights

Q = diag([0, 0, 1, 0]); % Accuracy Required
R = diag([0, 1]); % Actuator Effort (rudder, aileron)

%% State Transition Matrix
Np = 3; % prediction horizon
[vec,val] = eig(A); 

MID = diag([exp(val(1,1)*Np), exp(val(2,2)*Np),exp(val(3,3)*Np),exp(val(4,4)*Np)]);
phi = vec * MID * inv(vec);
F = real(C * phi);
G = real(C * inv(A) * (phi-eye(4)) * B);
H = G'*Q*G + R;

%% Discretized State Space
dt = 0.05;
Np = 3;
sys = ss(A, B, C, D);
sysd = c2d(sys, dt);
Ad = sysd.A;
Bd = sysd.B;
Cd = sysd.C;
Dd = sysd.D;
SP = [0, 0, 20*(pi/180), 0]';
%% Save Variables
save('StateSpace.mat','A', 'B', 'C', 'D','Ad', 'Bd', 'Cd', 'Dd', 'Np', 'dt',...
    'Q', 'R', 'F', 'G', 'H', 'SP');