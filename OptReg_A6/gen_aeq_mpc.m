function Aeq = gen_aeq_mpc(A,B,N,mx,b_length)


Aeq_c1 = eye(N*mx);                             % Component 1 of A_eq
Aeq_c2 = kron(diag(ones(N-1,1),-1), -A);        % Component 2 of A_eq
ones_block = gen_ones_block(N,b_length);    % Block-diagonal matrix of 1-vectors
Aeq_c3 = kron(ones_block, -B);                  % Component 3 of A_eq
Aeq = [Aeq_c1 + Aeq_c2, Aeq_c3];