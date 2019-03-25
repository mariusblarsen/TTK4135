%% Problem 1a - Assignment 9
% Steepest descent and Newton algorithm

clear all; close all;

%% Initial conditions
x0 = [1.2, 1.2]';
x0_hard = [-1.2, 1]';

%% Newton algorithm



[x_opt_n, fval_opt_n, x_iter_n, f_iter_n, alpha_n] = min_rosenbrock_newton(x0);
[x_opt_n_h, fval_opt_n_h, x_iter_n_h, f_iter_n_h, alpha_n_h] = min_rosenbrock_newton(x0_hard);

%% Steepes descent

[x_opt_sd, fval_opt_sd, x_iter_sd, f_iter_sd, alpha_sd] = min_rosenbrock_sd(x0);
[x_opt_sd_h, fval_opt_sd_h, x_iter_sd_h, f_iter_sd_h, alpha_sd_h] = min_rosenbrock_sd(x0_hard);


%% Plotting
% "Easy" x0
plot_iter_rosenbrock(x_iter_n, 1, x0);
plot_iter_rosenbrock(x_iter_sd, 2, x0);

% "Hard" x0
plot_iter_rosenbrock(x_iter_n_h, 3, x0_hard);
plot_iter_rosenbrock(x_iter_sd_h, 4, x0_hard);