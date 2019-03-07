clc; clear all;

% Code from 1f) - Assignment 5 
%% Initialization and model definition

% Discrete time system model. x = [lambda r p p_dot]'
delta_t	= 0.25; % sampling time
A = [0    0    0;
     0    0    1;
     0.1 -0.79 1.78];
 
B = [1 0 0.1]';
C = [0 0 1];
x0 = [0 0 1]';
% Number of states and inputs
mx = size(A,2); % Number of states (number of columns in A)
mu = size(B,2); % Number of inputs(number of columns in B)

% Time horizon and initialization
N  = 30;                                % Time horizon for states
M  = N;                                 % Time horizon for inputs
nb = 6;                                 % Number of time blocks
b_length = N/nb;                           % Length of each timeblock
z  = zeros(N*mx+M*mu,1);                % Initialize z for the whole horizon
z0 = z;                                 % Initial value for optimization

% Generate the matrix Q and G
Q1 = [0  0   0;  
     0  0   0;
     0  0   2];
r = 5; 
I_N = eye(N);
Rt = 2*r;
R = kron(b_length*eye(nb), Rt);
P1 = R;

Qt = 2*diag([0, 0, 1]);
Q = kron(I_N, Qt);
G = blkdiag(Q, R);
Q1 = zeros(mx,mx);

%% Generate system matrixes for linear model
% Equality constraint
Aeq = gen_aeq(A,B,N,mx,nb,b_length);
beq = [A*x0; zeros((N-1)*mx,1)];

%% Part f)
% Inequality constraint

% Bounds
ul 	    = -ones(mu,1);                   % Lower bound on control
uu 	    = ones(mu,1);                   % Upper bound on control

xl      = -Inf*ones(mx,1);              % Lower bound on states (no bound)
xu      = Inf*ones(mx,1);               % Upper bound on states (no bound)

% Generate constraints on measurements and inputs
[vlb,vub]       = gen_constraints(N,M,xl,xu,ul,uu); % hint: gen_constraints
vlb(N*mx+M*mu)  = 0;                    % We want the last input to be zero
vub(N*mx+M*mu)  = 0;                    % We want the last input to be zero

%% Solve QP problem with linear model
opt = optimset('Display','notify', 'Diagnostics','off', 'LargeScale','off');
[z,fval,exitflag,output,lambda] = quadprog(G,[],[],[],Aeq,beq,vlb,vub,[],opt);
output

%% Extracting valiables for plotting
y = [x0(3); z(mx:mx:N*mx)];        
u_blocks = z(N*mx+1:N*mx+nb*mu);   

ones_block = kron(eye(nb),ones(b_length,1));
u = ones_block*u_blocks;


%% Plotting

% Time vector
t = 1:N;

hold on
grid on
figure(1)
subplot(2,1,1);
plot(t,u,'r',t,u,'ro'),grid
ylabel('u_t')
hold on
grid on
subplot(2,1,2);
plot([0,t],y,'r',[0,t],y,'ro'),grid
ylabel('y_t')
xlabel('t')