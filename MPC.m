clear; clc;
%% Load State State Matrices
load('StateSpace.mat');




%% Initial Conditions
Time = 600;
k = 1;
ki = 0; % Initial Prediction time
kf = ceil(Time/dt); % Final Prediction time
dk = ceil(Np/dt); % Prediction Step size

del_X = [0, 0, 0, 0]'; % Initial States
U = [0; 0]; % Initial Inputs

h = 20000; % cruising altitude

yaw_angle = 0;

%% Logging Outputs & Inputs
v = zeros(kf, 1); % rightward velocity
phi = zeros(kf, 1); % Bank angle
r = zeros(kf, 1); % yaw rate

yaw = zeros(kf, 1); % yaw angle

del_a = zeros(kf); % aileron control inputs
del_r = zeros(kf); % rudder control inputs


%% Start Simulation Loop
while (k < kf-dk)
    
    %% Discretized state space Prediction
    del_X = (Ad*del_X) + (Bd*U);
    del_Y =  Cd*del_X;
    
    %% Predicted states
    Y_pred = F * del_X;
    
    %% Logging Outputs
    v(k) = del_X(1,1);
    phi(k) = del_X(3,1);
    r(k) = del_X(4,1);
    
    %% Integrating Yaw rate (trapezoid rule)
    if k >= 2
        yaw_angle = yaw_angle + ((dt/2) * (r(k)+r(k-1)));
    end
    yaw(k) = yaw_angle*180/pi;


    %% Guidance Commands
    R1 = 15*pi/180;
    
    %% Error
    E = [0; 0; R1 - Y_pred(3, 1); 0];
    f = -G'*Q*E;
    H = G'*Q*G + R;
    %% Constrained Optimization
    func = @(U) (E'*Q*E) + U'*H*U + 2*f'*U;

    bcon     = [0.523599;  
             0.523599; 
             0.523599; 
             0.523599];
    Acon = [1, 0;
        -1, 0;
         0, 1;
         0,-1];
    options = optimoptions('fmincon','Display', 'off', 'Algorithm','sqp', 'MaxIterations', 50);
    lb =  zeros(2,1) - Inf;
    ub =  zeros(2,1) + Inf;
    U = fmincon(func, U, Acon, bcon, Acon, bcon, lb, ub, [], options);
    
    
    % Increment loop counter
    k = k + 1;
end


%% Post-Processing
t = [0:dt:Time-dt];
subplot(1,3,1);
plot(t, phi * 180/pi);
xlabel('time (sec)');
ylabel('bank angle (deg)');
grid on;


subplot(1,3,2);
plot(t, r* 180/pi);
xlabel('time (sec)');
ylabel('Yaw rate (deg/s)');
grid on;
grid on;


subplot(1,3,3);
plot(t, yaw);
xlabel('time (sec)');
ylabel('Yaw (deg)');
grid on;
