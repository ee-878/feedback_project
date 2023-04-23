% Script for calculating the motor parameters given spec sheet data 
% and estimate of 1st order model (Tau and G_dc)
clear all
close all
clc

G_dc = 2.65;
tau = 0.125;

% motor resistance test
V = [3 7 12]; % V
I = [160 190 203]./1E3; % A
R = V./I;
R = mean(R);

% from spec sheet
I_stall = 5; % A
T_stall = 210; % oz-in
oz_in_to_Nm = 0.00706155;
T_stall = oz_in_to_Nm*T_stall;
no_load_rpm = 208;
no_load_current = 0.15; %A
gear_ratio = 35; % 35:1

K_t = T_stall/I_stall;
K_b = K_t;

% calculate L and b by matching w 1st order transfer function
tau = 0.125;
G_dc = 2.65;
b_ = (K_t/G_dc - K_b*K_t)/R
L_ = ((K_t/tau - K_b*K_t)/R)/b_
syms L b
eq2 = K_t/(K_b*K_t + R*b) == G_dc;
% assume(b, 'positive');
% assume(L, 'positive');
Y = solve(eq2, b);
b = double(Y)
eq1 = L*b/(K_b*K_t + R*b) == tau;
Y = solve(eq1, L)
L = double(Y)

s = tf('s');
sys = K_t/(L*b*s + K_t*K_b+R*b)

% omega/v step response
figure
step(sys)