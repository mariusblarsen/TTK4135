%% Problem 3b - Assignment 9
% Gradient calculation

%% System values

all_x = [[0.5 0.5]' [1 1]'];
 
% Values of epsilon, where 0 gives analytical gradient
eps = [0 10^-2 10^-3 10^-4];

grad_f = zeros(2,length(eps));

i = 1;

%% Approximating gradients

for x = all_x
    x1 = x(1);
    x2 = x(2);    
    for e=eps
        grad_f(:,i) = [200*(x1-x2)+2*(x1-1)+101*e
                       200*(x2-x1) + 100*e];
        i = i+1;  
    end
    
end
    