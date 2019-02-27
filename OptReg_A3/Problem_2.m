clear all; clc;

%% Initialize
% Objective function = (x_1 - 1)^2 + (x_2 - 2.5)^2
% S.t -x_1 + 2x_2 >= 0

x_0 = [2 0]';
c = [-2 -5]';
% z = [x, lambda, p]

% 0.5*p'*G*p + g_0'*p gives 
G = diag([2 2]);
g_0 = G*x_0 + c;
%% Constraints

a_1 = [1 -2]';  a_2 = [-1 -2]'; a_3 = [-1 2]';
a_4 = [1 0]';   a_5 = [0 1]';

a = [a_1 a_2 a_3 a_4 a_5];

b = [-2 -6 -2 0 0]';    

%% Iteration

i = 4; % Number of iterations, k = 0, 1, ... , i

w_0 = [3];                                % Active constraints
nc = length(w_0);                         % Number of active constraints
A_0 = zeros(size(a,1), nc);               % Matrix for working set

% Creating A_0
for j = 1:1:nc 
    A_0(:,j) = a(:, w_0(j));
end

K = [G -A_0;A_0' zeros(size(A_0,2))];

% Initialize matrixes
p = zeros(2,i); % Matrix for p_0, p_1 .. p_i
x = zeros(2,i); % Matrix for x_0, x_1 .. x_i
g = zeros(2,i);

%disp("Iteration, k = " + 0);

x(:,1) = x_0;
g(:,1) = g_0;
v = linsolve(K,[-g_0; 0]);
% v = [p_0 lambda_0]'
p(:,1) = v(1:2);
p_flag = 0; % False mean p != 0
if isempty(v(1:2))
    p_flag = 1;
end

    
lambda = v(3);

% WARNING: Iterations >= 2
for k = 2:1:i
    x(:,k) = x(:,k-1) + p(:,k-1); % From iteration k-2
    
    %disp("Iteration, k = " + (k-1))
    
    g(:,k) = G*x(:,k) + c; % Update g matrix
    
    if p_flag
        v = linsolve(G,-g(:,k));
    else
        v = linsolve(K,[-g(:,k); 0]);
    end
    
    
    
    p(:,k) = v(1:2);
    
    % Testing for p = 0
    treshold = 0.001;
    if p(1,k)<treshold && p(2,k)<treshold && lambda<0
        w_0 = []; % Special case for this problem, not generalized      
        p_flag = 1;
    else
        p_flag = 0;
    end
end

%% Report
disp("First colon is iteration k = 0 and so on.");
x, g, p
