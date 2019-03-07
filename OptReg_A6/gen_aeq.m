function Aeq = gen_aeq(A,B,N,mx,nb,b_length)
A_c1 = eye(N*mx);                            
A_c2 = kron(diag(ones(N-1,1),-1), -A);      
ones_block = kron(eye(nb),ones(b_length,1)); 
A_c3 = kron(ones_block, -B);     
Aeq = [A_c1 + A_c2, A_c3];