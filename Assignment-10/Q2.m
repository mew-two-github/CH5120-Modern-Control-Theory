% Load system matrices (A, B, Bw, C, H, R1, R2)  
load sysMat
Qbar=diag([10, 1]);
R=diag([0.25 0.25]);

% Obtain phi, gamma, psi, xi and Q matrices
size(Bw)
phi = [A zeros(20,4);[C;H]*A eye(4)];
psi = [Bw;[C;H]*Bw];
gamma = [B;[C;H]*B];
xi = [zeros(2,20) eye(2) zeros(2,2)];
Q = [zeros(20,2);zeros(2);eye(2)]*Qbar*[zeros(20,2);zeros(2);eye(2)]';