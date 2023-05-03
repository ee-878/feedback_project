%% Design
clear all
close all
clc

% parameters
K_t = 0.2966;
K_b = K_t;
m = 0.19845; % kg (.35 is apprx 3/4 lb)
l = 231.5/1E3; % m
J = m*l^2; % assuming point mass at end of pendulum
% L = 22.3284; % H
L = 4.89; % H
% b = 6.266E-4; % damping
% b = 6E-1;
b = 0.1; % damping coeff
g = 9.81; % m/s^2
R = 38.235; % Ohm
N = 35; % gear ratio

s = tf('s');
theta_V = N*K_t/(J*L*s^3 + (J*R + L*b)*s^2 + (K_t*K_b - m*g*l*L + R*b)*s - m*g*l*R); % motor w/ pendulum, including damping

T_s = 0.025;
theta_V_d = c2d(theta_V, T_s)

z = tf('z');
K = 1.5;
a = 0.95;
b = 0.5;
C_d = K*(z-a)/(z-b)

L_d = C_d*theta_V_d;

figure
rlocus(L_d);

%% Simulation
clear all
close all
clc

% parameters
K_t = 0.2966;
K_b = K_t;
m = 0.17; % kg (.35 is apprx 3/4 lb)
l = 231.5/1E3; % m
J = m*l^2; % assuming point mass at end of pendulum
% L = 22.3284; % H
L = 4.89; % H
% b = 6.266E-4; % damping
b = 0.1;
g = 9.81; % m/s^2
R = 38.235; % Ohm
N = 35; % gear ratio

theta(1) = deg2rad(30);
theta_d(1) = 0;
V(1) = 0;
I(1) = 0;
dt = 0.001;
t_end = 30; 
for i = 2:(t_end/dt - 1)
    t(i) = (i-1)*dt;
    V(i) = 0; % control law goes here
    I_dot(i) = (V(i) - I(i-1)*R + K_b*theta_d(i-1)/N)/L;
    I(i) = I(i-1) + I_dot(i)*dt;
    theta_dd(i) = (N*K_t*I(i) - b*theta_d(i-1) + m*g*l*sin(theta(i-1)))/J;
    theta_d(i) = theta_d(i-1) + theta_dd(i)*dt;
    theta(i) = theta(i-1) + theta_d(i)*dt;
end

figure
plot(t, rad2deg(theta));
title('Position');
xlabel('Time (s)')
ylabel('Angle (deg)')