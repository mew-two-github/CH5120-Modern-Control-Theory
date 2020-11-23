% Please enter your solution below
Roll_No='CH18B020';     % Please replace with your roll number
%% State space model
A = [0.2 -0.4;0 0.25];
B = [0;0];
C = [1 0];
G = [0.2;1.2];
D = 0;
H = 0;
sys = ss(A,[B G],C,[D H],-1);
% Please do your calculations below this line
%% Solving the Riccati equation
QN = 1; RN = 0.25;
[KEST,L,P,~] = kalman(sys,QN,RN);
% KEST are the optimal estimators of u and y (not needed for this problem)
% L - Predictor Gain and P - error covariance of xe
K_inf = A\L; Pbar_inf = P;
%% Getting Pbar_inf 
% P_inf = (I-KC)*Pbar
P_inf = (eye(size(Pbar_inf))-K_inf*C)*Pbar_inf;
%% Getting the poles
Lam_Kf = eig(A-K_inf*C);
%% 1. Calculations with Q=1 and R=2.5e-5
%    Provide Kalman gain in K_kf1 and observer eigenvalues in Lam_1
% Solving the Riccati equation
QN = 1; RN = 2.5*10^-5;
[KEST,L,P,~] = kalman(sys,QN,RN);
K_kf1 = A\L;
Lam_1 = eig(A-L*C);
%% 2. Calculations with Q=1e-4 and R=0.25
%    Provide Kalman gain in K_kf2 and observer eigenvalues in Lam_2
QN = 10^-4; RN = 0.25;
[KEST,L,P,~] = kalman(sys,QN,RN);
K_kf2 = A\L;
Lam_2 = eig(A-L*C);