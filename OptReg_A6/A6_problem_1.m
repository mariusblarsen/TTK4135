%% Problem 1 d)
clear;

% System values
A = [1  0.5;
     0  1];

b = [0.125 0.5]';

Q = diag([2 2]);

R = 2;

%Calculate the optimal gain matrix K
[K,S,e] = dlqr(A,b,Q/2,R/2);

%% Problem 1 e)

e = eig(A - b*K);
l = abs(e);