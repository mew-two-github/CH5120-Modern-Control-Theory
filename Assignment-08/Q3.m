% Please enter your solution below
Roll_No='CH18B020';     % Please replace with your roll number

% Start your solution below
% Report the values of 
%   (i)  LQR gain in vector "Linf_#", and
%   (ii) closed-loop eigenvalues in "Lam#"
A = [0.2 -0.4;0 0.25];
B = [1;0];
% Tuning Matrices: Baseline
Q=eye(2);
R=1;

% 1. Aggressive (Please provide controller gain in Linf_1 and eigenvalues in Lam1)
Q = 100*eye(2);
R = 1;
[Linf_1 S Lam1] = dlqr(A,B,Q,R,0)

% 2. Sluggish (Please provide controller gain in Linf_2 and eigenvalues in Lam2)
Q=eye(2);
R=100;
[Linf_2 S Lam2] = dlqr(A,B,Q,R,0);

% 3. Very Aggressive (Please provide only the eigenvalues in Lam3)
Q=10^4*eye(2);
R=1;
[Linf_3 S Lam3] = dlqr(A,B,Q,R,0);


% 4. Very Sluggish Control (Please provide only the eigenvalues in Lam4)
Q=eye(2);
R=1*10^4;
[Linf_4 S Lam4] = dlqr(A,B,Q,R,0);