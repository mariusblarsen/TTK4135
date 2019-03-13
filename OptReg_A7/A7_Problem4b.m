%% Problem 4a)
%clear all; clc;

%% System
% System values
k1 = 1;
k2 = 1;
k3 = 1;
T = 0.1;

% Continous time system model
Ac = [0  1;
    -k2 -k1];

Bc = [0 k3]';

% Descrete time system model
A = [1      T;
    -k2*T  1-k1*T];
B = [0 k3*T]';
C = eye(2);

% Number of states and inputs
mx = size(A,2); % Number of states (number of columns in A)
mu = size(B,2); % Number of inputs(number of columns in B)

% Initial values
x0 = [5 1]';    % Initial condition

%% Cost function
N = 10;

Rt = 1;
R = kron(eye(N), Rt); 
P1 = R;

Qt = diag([4 4]);
Q = kron(eye(N-1), Qt);

%% Riccati and dlqr

[K,P,e] = dlqr(A,B,Qt/2,Rt/2,[]);

G = blkdiag(Q, 2*P, R);

%% Equality constraint

Aeq_c1 = eye(N*mx);                         % Component 1 of A_eq
Aeq_c2 = kron(diag(ones(N-1,1),-1), -A);    % Component 2 of A_eq
Aeq_c3 = kron(eye(N), -B);                     % Component 3 of A_eq
Aeq = [Aeq_c1 + Aeq_c2, Aeq_c3];

%% Inequality constraint

% Bounds
ul 	    = -4*ones(N*mu,1);                   % Lower bound on control
uu 	    = 4*ones(N*mu,1);                   % Upper bound on control

xl      = -Inf*ones(N*mx,1);              % Lower bound on states (no bound)
xu      = Inf*ones(N*mx,1);               % Upper bound on states (no bound)

[vlb] = [xl; ul]; % Vector of lower bounds
[vub] = [xu; uu]; % Vector of upper bounds

%% MPC

% Simulate
tf = 50;
x = NaN(2,tf+1);
u = NaN(1,tf+1);
y = NaN(1,tf+1);

x(:,1) = x0;
beq = [zeros(mx,1); zeros((N-1)*mx,1)];

opt = optimset('Display','notify', 'Diagnostics','off', 'LargeScale','off', 'Algorithm', 'interior-point-convex');

for t = 1:tf
    
    % Update equality constraint 
    beq(1:mx) = A*x(:,t);
    
    % Solve optimization problem
    [z,fval,exitflag,output,lambda] = quadprog(G,[],[],[],Aeq,beq,vlb,vub,[],opt);
    
    % Extract optimal control from solution z
    u_ol = z(N*mx+1:N*mx+N*mu);    % Control, open-loop optimal
    u(t) = u_ol(1); % Only first element is used
    
    % Simulate system one step ahead
    x(:,t+1) = A*x(:,t) + B*u(t);    
end
 output
 
%% Plotting

% Time vector
t = 0:1:tf;

hold on
grid on

figure(1)

% x
subplot(2,1,1);
plot(t,x),grid
hold on
hleg = legend('$x_1(t)$, N = 10', '$x_2(t)$, N = 10','$x_1(t)$, N = 3', '$x_2(t)$, N = 3''$x_1(t)$, N = 1', '$x_2(t)$, N = 1');
set(hleg, 'Interpreter', 'Latex');
ylabel('x')
hold on
box('on')
grid on

% u_t
subplot(2,1,2);
plot(t,u),grid
hleg = legend('N = 10', 'N = 3', 'N = 1');
set(hleg, 'Interpreter', 'Latex');
box('on')
grid on
ylabel('u_t')
xlabel('t')
