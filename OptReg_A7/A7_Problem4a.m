%% Problem 4a)
clear all; clc;

%% System
% System values
k1 = 1;
k2 = 1;
k3 = 1;
T = 0.1;

% Continous time system model
Ac = [0  1;
    -k2 -k1];

Bc = [0 k3]';

% Descrete time system model
A = [1      T;
    -k2*T  1-k1*T];
B = [0 k3*T]';
C = eye(2);

% Number of states and inputs
mx = size(A,2); % Number of states (number of columns in A)
mu = size(B,2); % Number of inputs(number of columns in B)

% Initial values
x0 = [5 1]';    % Initial condition

%% Cost function
N = 10;

Rt = 1;
R = kron(eye(N), Rt); 
P1 = R;

Qt = diag([4 4]);
Q = kron(eye(N-1), Qt);
G = blkdiag(Q, R);

%% Riccati and dlqr

[K,P,e] = dlqr(A,B,Qt/2,Rt/2,[]);