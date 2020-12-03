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
[KEST,L,P,Mx,Z,My] = kalman(sys,QN,RN);
% KEST are the optimal estimators of u and y (not needed for this problem)
% L - Predictor Gain and P - error covariance of xe
K_inf = A\L; Pbar_inf = P;
%% Getting Pbar_inf 
% P_inf = (I-KC)*Pbar
P_inf = (eye(size(Pbar_inf))-K_inf*C)*Pbar_inf;
%% Getting the poles
Lam_Kf = eig(A-K_inf*C);