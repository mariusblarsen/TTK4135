clear all; close all; clc;

%% Initialize
% Objective function = (x_1 - 1)^2 + (x_2 - 2.5)^2
% S.t -x_1 + 2x_2 >= 0

x_0 = [2 0]';
% z = [x, lambda, p]

G = diag([2 2]);

%% Constraints

a_1 = [1 -2]';  a_2 = [-1 -2]'; a_3 = [-1 2]';
a_4 = [1 0]';   a_5 = [0 1]';

a = [a_1 a_2 a_3 a_4 a_5];

b = [-2 -6 -2 0 0]';    c = [-2 -5]';

%% Iteration

i = 3; % Number of iterations

w_0 = [3]; % Active constraints for working set

A_0 = 

% Creating A_0
for k = 0:1:i - 1
    disp("Iteration " + k)
    for j = 1:1:length(w_0)
        disp("Active set number " + j)
        disp(a(:,w_0(j))) % Prints out all active sets
    end
end


