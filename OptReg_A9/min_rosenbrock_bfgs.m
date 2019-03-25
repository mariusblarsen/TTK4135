function [x_opt, fval_opt, x_iter, f_iter, alpha] = min_rosenbrock_bfgs(x0)

maxiter = 1e3;
grad_tol = 1e-3;

%% Iniitializing
x0 = x0(:);
n = size(x0,1); % Number of variables
x =     NaN(n,maxiter);
p =     NaN(n,maxiter);
grad =  NaN(n,maxiter);
alpha = NaN(n,maxiter);
fval =  NaN(n,maxiter);

k = 1;

x(:,k) = x0;
grad(:,k) = gradient(x(:,k)); % To be updatet

%% Hessian
% Initial
H = NaN(n, n, maxiter);
e = diag(ones(1, n)); % Initial Hessian

% Approximating the columns of the Hessian, based on the gradients
epsilon = 10^-5; % Convergence tolerance
H_inv_col_1 = (gradient(x0+epsilon*e(:,1)) - gradient(x0-epsilon*e(:,1)))/epsilon*2;
H_inv_col_2 = (gradient(x0+epsilon*e(:,2)) - gradient(x0-epsilon*e(:,2)))/epsilon*2;
H_inv = [H_inv_col_1 H_inv_col_2];

H(:,:,k) = inv(H_inv);



%% BFGS
while (k < maxiter) && (norm(grad(:,k)) >= grad_tol)
    fval(k) = f(x(:,k));
    p(:,k) = -H(:,:,k)*grad(:,k); % Computing search direction
    alpha_0 = 1; % Step length
    alpha(k) = linesearch(x(:,k), p(:,k), fval(k), grad(:,k), alpha_0);
    x(:,k+1) = x(:,k) + alpha(k)*p(:,k);
    grad(:,k+1) = gradient(x(:,k+1));
    %s_k and y_k
    s_k = x(:,k+1) - x(:,k);
    y_k = grad(:,k+1) - grad(:,k);
    % Compute next step of Hessian, (6.17) in textbook
    rho_k = (y_k'*s_k)^-1;
    I = eye(numel(s_k));
    H_next = (I - rho_k*s_k*y_k')*H(:,:,k)*(I - rho_k*y_k*s_k')+rho_k*(s_k*s_k');
    H(:,:,k+1) = H_next;
    % Go to next step
    k = k+1;
end
fval(k) = f(x(:,k));

% Delete unused space
x = x(:,1:k);
p = p(:,1:k);
grad = grad(:,1:k);
alpha = alpha(1:k);
fval = fval(1:k);

% Return values
x_opt = x(:,end);
fval_opt = f(x_opt);
x_iter = x;
f_iter = fval;

end

function alpha_k = linesearch(xk, pk, fk, gradk, alpha_0)
    alpha = alpha_0;
    rho = 0.95;
    c1 = 1e-4;
    while f(xk + alpha*pk) > fk + c1*alpha*gradk'*pk
        alpha = rho*alpha;
    end
    alpha_k = alpha;
end

function fval = f(x)
    fval = 100*(x(2)-x(1)^2)^2 + (1-x(1))^2;
end

function grad = gradient(x)
    grad = [ -400*(x(1)*x(2)-x(1)^3) + 2*x(1) - 2 ;
             200*(x(2)-x(1)^2)                   ];
end