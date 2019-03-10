%% Problem 2c)

% Fetch values from b
A7_Problem2b; close all;

phi11 = A - B*K;
phi12 = B*K;
phi22 = A - KF*C;


phi=zeros(numel(phi11),numel(phi11));
phi(1:2,1:2) = phi11;
phi(1:2,3:4) = phi12;
phi(3:4,3:4) = phi22;

e_phi = eig(phi);

