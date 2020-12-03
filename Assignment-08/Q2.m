% Tuning Matrices
Q=eye(2);
R=1;

% Start your solution below (using Algebraic Ricatti Equation)
% Report the values of Linf and LQRpoles
A = [0.2 -0.4;0 0.25];
B = [1;0];
[X,L,G] = dare(A,B,Q,R,0,eye(2));
Linf = G;
LQRpoles = L;