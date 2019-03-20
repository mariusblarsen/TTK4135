%% Problem 3 - Assignment 8
% Gradient and Hessian of a function

syms x1 x2

%% Gradient
% |f1|
% |f2|

f = 100*(x2 - x1^2)^2 + (1-x1)^2;
f1 = -400*(x1*x2 - x1^3) + 2*x1 - 2;
f2 = 200*(x2-x1^2);

%% Hessian
% |H1 H2|
% |H3 H4|

hessian(f,[x1,x2])
