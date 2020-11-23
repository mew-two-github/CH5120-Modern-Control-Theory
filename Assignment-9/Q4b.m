% Simulations of a Kalman Filter
% Please enter your solution below
Roll_No='CH18B020';     % Please replace with your roll number
[XALL,YALL]=estimation_data(Roll_No);

% Please return estimates in array   XHAT
% Please return sum of square error:  SSE
% ---------------------------------------
% Please start typing from the line below
% ---------------------------------------
A = [0.2 -0.4;0 0.25];
B = [0;0];
C = [1 0];
G = [0.2;1.2];
D = 0;
H = 0;
sys = ss(A,[B G],C,[D H],-1);
%% Solving the Riccati equation
QN = 1; RN = 0.25;
[KEST,L,P,~] = kalman(sys,QN,RN);
% KEST are the optimal estimators of u and y (not needed for this problem)
% L - Predictor Gain and P - error covariance of xe
K_inf = A\L;
XHAT = zeros(2,200);
x0 = [0;0];
XHAT(:,1) = x0;
K = K_inf;
%% Estimate x
for k = 2:200
    x_1step = A*XHAT(:,k-1);
    XHAT(:,k) = x_1step + K*(YALL(k)-C*x_1step);
end


%% Plot the results
subplot(2,1,1)
plot(1:200,XALL(1,:), 1:200,XHAT(1,:));
subplot(2,1,2)
plot(1:200,XALL(2,:), 1:200,XHAT(2,:));

%% Compute SSE
sqErr=(XALL-XHAT).^2;
SSE=sum(sum(sqErr));
