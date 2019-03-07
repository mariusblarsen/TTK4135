%% OptReg A6 - Problem 3e)
% Modified code from OptReg A5 - Problem 2b)
%% Initialization and model definition

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
b_length = [1, 1, 2, 4, 8, 14]'; % Lenghts of blocks
nb = numel(b_length);
% Generate the matrix Q and G
r = 1; 
Rt = 2*r;
R = kron(diag(b_length), Rt);

Qt = 2*diag([0, 0, 1]);
Q = sparse(kron(eye(N), Qt));
G = blkdiag(Q, R);

%% Generate system matrixes for linear model
Aeq = gen_aeq_mpc(A,B,N,mx,b_length); % Generate A, hint: gen_aeq

% Inequality constraint
ul 	    = -ones(mu,1);                   % Lower bound on control
uu 	    = ones(mu,1);                   % Upper bound on control

xl      = -Inf*ones(mx,1);              % Lower bound on states (no bound)
xu      = Inf*ones(mx,1);               % Upper bound on states (no bound)

% Generate constraints on measurements and inputs
[vlb,vub]       = gen_constraints(N,M,xl,xu,ul,uu); % hint: gen_constraints
vlb(N*mx+M*mu)  = 0;                    % We want the last input to be zero
vub(N*mx+M*mu)  = 0;                    % We want the last input to be zero

%% MPC

opt = optimset('Display','notify', 'Diagnostics','off', 'LargeScale','off', 'Algorithm', 'interior-point-convex');

u = NaN(mu, N);
x = NaN(mx, N + 1);
x(:,1) = x0;
beq = [zeros(mx,1);zeros((N-1)*mx,1)]; % Initialize b, varies with time
ones_block = gen_ones_block(N,b_length);

% Simulation and solve for each step
for t = 1:N
    % Update beq, with most recent measurment x_t = x(:,t)
    beq(1:mx) = A*x(:,t);
    % Solve optimization problem
    z = quadprog(G,[],[],[],Aeq,beq,vlb,vub,x0,opt);
    
    % Extract optimal control from solution z
    u_blocks = z(N*mx+1:N*mx+nb*mu);    % Control blocks, open-loop optimal
    
    u_ol = ones_block*u_blocks;         % Control
    u(t) = u_ol(1); % Only first element is used
    
    % Simulate system one step ahead
    x(:,t+1) = A*x(:,t) + B*u(t);
    
end

y = C*x;

%% Plotting

% Time vector
t = 0:1:N;
t2 = 0:1:N-1;

hold on
grid on

figure(1)

subplot(2,1,1);
plot(t,y,'r',t,y,'ro'),grid
grid('on');
ylabel('y_t')
hold on
box('on')
grid on

subplot(2,1,2);
stairs(t2,u); % Plot steps instead of continious u
%plot(t2,u,'r',t2,u,'ro'),grid
box('on')
grid on
ylim([-1.5,1]);
ylabel('u_t')
xlabel('t')
