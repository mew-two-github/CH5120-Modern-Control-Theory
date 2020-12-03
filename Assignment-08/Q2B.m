Roll_No='CH18B020';     % Please replace with your roll number

% Tuning Matrices
Q=eye(2);
R=1;
% Initial value
Sp=eye(2);

% Start your solution using Ricatti Difference Equation below
% (*) Report S(p-1)  in 2*2 matrix S1
% (*) Report S(p-2)  in 2*2 matrix S2
% (*) Report S(p-3)  in 2*2 matrix S3
% (*) Report S(p-20) in 2*2 matrix Sinf
A = [0.2 -0.4;0 0.25];
B = [1;0];
S1 = A'*Sp*A + Q - A'*Sp*B*pinv(B'*Sp*B+R)*pinv(B)*Sp*A;
S2 = A'*S1*A + Q - A'*S1*B*pinv(B'*S1*B+R)*pinv(B)*S1*A;
S3 = A'*S2*A + Q - A'*S1*B*pinv(B'*S2*B+R)*pinv(B)*S2*A;
S = Sp;
for i = 1:20
    S = A'*S*A + Q - A'*S*B*pinv(B'*S*B+R)*pinv(B)*S*A;
end
Sinf = S;