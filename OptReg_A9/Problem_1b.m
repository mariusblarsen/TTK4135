%% Problem 1b - Assignment 9
% BFGS Method

clear all; close all;

%% Initial conditions
x0 = [1.2, 1.2]';
x0_hard = [-1.2, 1]';

%% BFGS
[x_opt_bfgs, fval_opt_bfgs, x_iter_bfgs, f_iter_bfgs, alpha_bfgs] = min_rosenbrock_bfgs(x0);
[x_opt_bfgs_h, fval_opt_bfgs_h, x_iter_bfgs_h, f_iter_bfgs_h, alpha_bfgs_h] = min_rosenbrock_bfgs(x0_hard);

%% Plotting
% "Easy" x0
plot_iter_rosenbrock(x_iter_bfgs, 1, x0);

% "Hard" x0
plot_iter_rosenbrock(x_iter_bfgs_h, 2, x0_hard);
