%% Problem 2a)
clear all; clc;
% System values
k1 = 1;
k2 = 1;
k3 = 1;
T = 0.1;

% Continous time system model
Ac = [0  1;
    -k2 -k1];
Bc = [0 k3]';
Cc = [1 0];

% Descrete time system model
A = [1      T;
    -k2*T  1-k1*T];
B = [0 k3*T]';
C = [1 0];

% Initial values
x0 = [5 1]';    % Initial condition
x0_hat = [6 0]';% Initial state estimate



% LQR
Q = diag([4 4]);
R = 1;

[K,S,e] = dlqr(A,B,Q/2,R/2,[]);



