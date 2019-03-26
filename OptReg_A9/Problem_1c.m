%% Problem 1c - Assignment 9
% Steplength, alpha, for BFGS and Newton method

clear all; close all;

%% Initial conditions
x0 = [1.2, 1.2]';
x0_hard = [-1.2, 1]';

%% BFGS
[x_opt_bfgs, fval_opt_bfgs, x_iter_bfgs, f_iter_bfgs, alpha_bfgs] = min_rosenbrock_bfgs(x0);
[x_opt_bfgs_h, fval_opt_bfgs_h, x_iter_bfgs_h, f_iter_bfgs_h, alpha_bfgs_h] = min_rosenbrock_bfgs(x0_hard);

%% Newton algorithm

[x_opt_n, fval_opt_n, x_iter_n, f_iter_n, alpha_n] = min_rosenbrock_newton(x0);
[x_opt_n_h, fval_opt_n_h, x_iter_n_h, f_iter_n_h, alpha_n_h] = min_rosenbrock_newton(x0_hard);

%% Number of iterations
k_bfgs = length(alpha_bfgs);
k_bfgs_h = length(alpha_bfgs_h);
k_n = length(alpha_n);
k_n_h = length(alpha_n_h);

fig = ["BFGS, x0 = [1.2; 1.2]" 'BFGS, x0 = [-1.2; 1.0]' 'Newton, x0 = [1.2, 1.2]' 'Newton, x0 = [-1.2, 1.0]'];
k = [k_bfgs k_bfgs_h k_n k_n_h];
a = [alpha_bfgs alpha_bfgs_h alpha_n alpha_n_h];
%% Plotting
j = 1; % Fig number
prev = 1;
for i=1:length(k)
    figure (j)
    ks = 1:1:k(i);
    plot(ks, a(prev:prev + k(i)-1));
     
    fig_title = join(['x0 = ', mat2str(x0)]);
    title(fig(i));
    xlabel('k');
    ylabel('\alpha_k');
    
    prev = k(i)+1;
    j=j+1;
end
