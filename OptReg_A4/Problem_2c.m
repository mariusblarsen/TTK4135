%% Modified
% G, c, A and b modified, with values from a)

G = [0.8 0 ;
     0 0.4]; % Remember the factor 1/2 in the objective
c = [-3 ; -2];

% Linear constraints
A = [2 1 ; 
     1 3];
b = [8 ; 15];