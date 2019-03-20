%% Problem 2d) - Assignment 8
% Newton direction and properties of convexity

%% System
G = [2 1;
     1 2];
 
c = [1 0]';

fun = @(z) 0.5*z'*G*z+z'*c;

%% Inequality
syms x1 x2 y1 y2 a real

x = [x1 x2]';
y = [y1 y2]';

% Left side
LS = fun(a*x+(1-a)*y)

% Right side
RS = a*fun(x) + (1-a)*fun(y)


isAlways(LS <= RS | a^2 <= 1 & a >= 0 & x1^2 + x2^2 <= 1)


