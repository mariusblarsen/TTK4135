% TTK4135 - Helicopter lab
% Hints/template for problem 2.
% Updated spring 2018, Andreas L. Flåten

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
N  = 30;                                  % Time horizon for states
M  = N;                                 % Time horizon for inputs
z  = zeros(N*mx+M*mu,1);                % Initialize z for the whole horizon
z0 = z;                                 % Initial value for optimization

% Generate the matrix Q and G
Q1 = [0  0   0;  
     0  0   0;
     0  0   2];
r = 5; 
I_N = sparse(eye(N));
Rt = 2*r;
R = sparse(kron(I_N, Rt));
P1 = R;

Qt = 2*diag([0, 0, 1]);
Q = sparse(kron(I_N, Qt));
G = blkdiag(Q, R);
Q1 = zeros(mx,mx);

%% Generate system matrixes for linear model
Aeq = gen_aeq(A,B,N,mx,mu); % Generate A, hint: gen_aeq
beq = sparse([A*x0; zeros((N-1)*nx,1)]); % Generate b

%% Part f)
% Inequality constraint

% Bounds
ul 	    = -ones(mu,1);                   % Lower bound on control
uu 	    = ones(mu,1);                   % Upper bound on control

xl      = -Inf*ones(mx,1);              % Lower bound on states (no bound)
xu      = Inf*ones(mx,1);               % Upper bound on states (no bound)
%xl(3)   = ul;                           % Lower bound on state x3
%xu(3)   = uu;                           % Upper bound on state x3

% Generate constraints on measurements and inputs
[vlb,vub]       = gen_constraints(N,M,xl,xu,ul,uu); % hint: gen_constraints
vlb(N*mx+M*mu)  = 0;                    % We want the last input to be zero
vub(N*mx+M*mu)  = 0;                    % We want the last input to be zero

%% Solve QP problem with linear model
opt = optimset('Display','notify', 'Diagnostics','off', 'LargeScale','off');
[z,fval,exitflag,output,lambda] = quadprog(G,[],[],[],Aeq,beq,vlb,vub,[],opt);



%% Extract control inputs and states
u  = [z(N*mx+1:N*mx+M*mu);z(N*mx+M*mu)]; % Control input from solution

x1 = [x0(1);z(1:mx:N*mx)];              % State x1 from solution
x2 = [x0(2);z(2:mx:N*mx)];              % State x2 from solution
x3 = [x0(3);z(3:mx:N*mx)];              % State x3 from solution


%% Plotting

% Time vector
t = 0:1:N;

hold on
grid on
figure(1)
subplot(2,1,1);
plot(t,u,'r',t,u,'ro'),grid
ylabel('u_t')
hold on
grid on
subplot(2,1,2);
plot(t,x3,'r',t,x3,'ro'),grid
ylabel('y_t')
xlabel('t')