%% Part a)

A = [0    0    0;
     0    0    1;
     0.1 -0.79 1.78];
 
B = [1 0 0.1]';

%disp(eig(A));

%% Part d)
C = [0 0 1];
x0 = [0 0 1]';

% Init
delta_t	= 0.25; % sampling time
N = 30;
M = N; %Same for states and inputs

r = 1;

Q1 = [0  0   0;  
     0  0   0;
     0  0   2];
 
I_N = sparse(eye(N));
Rt = 2*r;
R = sparse(kron(I_N, Rt));
P1 = R;

Qt = 2*diag([0, 0, 1]);%gen_q(Q1,P1,N,M);
Q = sparse(kron(I_N, Qt));
G = blkdiag(Q, R);
% Number of states and inputs
mx = size(A,2); % Number of states (number of columns in A)
mu = size(B,2); % Number of inputs(number of columns in B)

% Equality constraint
Aeq_c1 = sparse(eye(N*nx));                         % Component 1 of A_eq
Aeq_c2 = sparse(kron(diag(ones(N-1,1),-1), -A));   % Component 2 of A_eq
Aeq_c3 = sparse(kron(I_N, -B));                    % Component 3 of A_eq
Aeq = [Aeq_c1 + Aeq_c2, Aeq_c3];

beq = sparse([A*x0; zeros((N-1)*nx,1)]);

% Zero-matrix and -vector for the KKT system
zero_m = sparse(zeros(N*nx));
zero_v = sparse(zeros(N*(nx+nu),1));

% KKT system
KKT_matrix = [G     -Aeq' ;
           Aeq  zero_m];
KKT_vector = [zero_v; beq];

% Solving the KKT system
KKT_sol = KKT_matrix\KKT_vector;

% Extracting variables
z = KKT_sol(1:N*(nx+nu));   % Variable vector (KKT_sol includes lambdas)

%% Extract control inputs and states
u  = [z(N*mx+1:N*mx+M*mu);z(N*mx+M*mu)]; % Control input from solution

x1 = [x0(1);z(1:mx:N*mx)];              % State x1 from solution
x2 = [x0(2);z(2:mx:N*mx)];              % State x2 from solution
x3 = [x0(3);z(3:mx:N*mx)];              % State x3 from solution


%% Plotting

% Time vector
t = 0:1:N;

figure(1)
subplot(2,1,1);
plot(t,u,'m',t,u,'mo'),grid
ylabel('u_t')
subplot(2,1,2);
plot(t,x3,'m',t,x3,'mo'),grid
ylabel('y_t')
xlabel('t')