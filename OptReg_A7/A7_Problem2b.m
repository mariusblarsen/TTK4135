%% Problem 2b)
clear all; clc;

% System values
k1 = 1;
k2 = 1;
k3 = 1;
T = 0.1;

% Continous time system model
Ac = [0  1;
    -k2 -k1];
Bc = [0 k3]';
Cc = [1 0];

% Descrete time system model
A = [1      T;
    -k2*T  1-k1*T];
B = [0 k3*T]';
C = [1 0];

% Initial values
x0 = [5 1]';    % Initial condition
x0_hat = [6 0]';% Initial state estimate



% LQR
Q = diag([4 4]);
R = 1;

[K,S,e] = dlqr(A,B,Q/2,R/2,[]);

% Observer
obs_poles = 0.5 + 0.03j*[1; -1];
KF = place(A',C',obs_poles).';

% Simulate
tf = 50;
x = NaN(2,tf+1);
u = NaN(1,tf+1);
y = NaN(1,tf+1);
x_hat = NaN(2,tf+1);

x(:,1) = x0;
x_hat(:,1) = x0_hat;

for t = 1:tf
    % System simulated one step ahead:
    u(:,t) = -K*x_hat(:,t);
    x(:,t+1) = A*x(:,t) + B*u(:,t);
    y(:,t) = C*x(:,t);
    % Calculate state estimate based on measurement y:
    x_hat(:,t+1) = A*x_hat(:,t) + B*u(:,t) + KF*(y(:,t) - C*x_hat(:,t));
end

%% Plotting

% Time vector
t = 0:1:tf;
%t2 = 0:1:N-1;

hold on
grid on

figure(1)

% x and x_hat
subplot(2,1,1);
plot(t,x),grid
hold on
plot(t,x_hat, '--'),grid
hold off
hleg = legend('$x_1(t)$', '$x_2(t)$', '$\hat{x}_1(t)$', '$\hat{x}_2(t)$');
set(hleg, 'Interpreter', 'Latex');
ylabel('x and x\_hat')
hold on
box('on')
grid on

% u_t
subplot(2,1,2);
plot(t,u),grid
box('on')
grid on
ylabel('u_t')
xlabel('t')


