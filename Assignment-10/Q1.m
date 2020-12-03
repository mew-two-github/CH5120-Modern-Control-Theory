%% Simulations of dead-beat estimator
% Load system matrices (A, B, Bw, C, H, R1, R2)  
load sysMat

% Load inputs (L, V), measurements (ym) and expected controlled outputs (yc)
load kfExample

% Initialization of x(0) and xHat(0)
x0=zeros(20,1);
xhat0=zeros(20,1);
P0=0.01*eye(20);
%% ===== END OF FIXED SECTION OF THE CODE =====
ninputs = 2; noutputs = 2; nstate = 20;
R1 = Bw*R1*Bw';
YHAT = zeros(200,2);
%% ===== KALMAN FILTER SIMULATIONS (with w(k) as white noise) =====
xhat = zeros(200,20);
P = P0;
x= x0;
YHAT(1,:) = H*x0;
for k = 2:200
    % Prediction
    x = A*x + B*[L(k);V(k)];
    %size(x)
    % Calculation of filtering error covariance matrix
    Pbar = A*P*A' + R1;
    % Kalman Filter gain
    K = Pbar*C'*inv(C*Pbar*C'+ R2);
    % Estimate
    %size(ym(k,:))
%     size((ym(k,:)-H*x))
%     size(K)
    P = (eye(size(K*C))-K*C)*Pbar;    
    xhat(k,:) = x + K*(ym(k,:)'-C*x);
    x = xhat(k,:)';
    % Obtain the controlled variable value
    YHAT(k,:) = H*xhat(k,:)';
end
size(YHAT)
size(yc)
size(ym)
%% ===== CALCULATIONS AND PLOTTING =====
% Please report the sum of square error in yc_1 as SSE1
SSE1 = sum((yc(:,1)-YHAT(:,1)).^2);  % Sums over all time-points for the first output
disp(SSE1);      % Displays SSE

SSE2 = sum((yc(:,2)-YHAT(:,2)).^2);  % Sums over all time-points for the second output
disp(SSE2);      % Displays SSE


subplot(2,1,1)  % Plot of x1
plot(1:200,yc(:,1),'--b',1:200,YHAT(:,1),'-r'); xlabel('Distillate, x_D(k)');
subplot(2,1,2)  % Plot of x2
plot(1:200,yc(:,2),'--b',1:200,YHAT(:,2),'-r'); xlabel('Bottoms, x_B(k)');

