%% Simulations of dead-beat estimator
% Load system matrices (A, B, Bw, C, H, R1, R2)  
load sysMat

% Load inputs (L, V), measurements (ym) and expected controlled outputs (yc)
load kfExample

% Initialization of x(0) and xHat(0)
x0=zeros(20,1);
xhat0=zeros(20,1);
P=0.01*eye(20);
%% ===== END OF FIXED SECTION OF THE CODE =====

%% ===== KALMAN FILTER SIMULATIONS (with w(k) as integrated white noise) =====
% First, augment the model to handle IWN
phi = [A zeros(20,2);C*A eye(2)];
psi = [Bw;C*Bw];
gamma = [B;C*B];
xi = [zeros(2,20) eye(2)];
P = 0.01*eye(22);
%P(21:22,:) = zeros(2,22);
size(phi)
%
y0 = zeros(2,1);
z = [x0;y0];
ninputs = 2; noutputs = 2; nstate = 24;
R1 = Bw*R1*Bw';
R1 = [R1 zeros(20,2);zeros(2,20) R2];
YHAT = zeros(200,2);
yc_pred= zeros(2,1);
ym_pred = zeros(2,1);
%% ===== KALMAN FILTER SIMULATIONS (with w(k) as white noise) =====
zhat = zeros(200,22);
for k = 1:200
    % Prediction
    if k==1
        z = phi*z + psi*[L(k);V(k)]; % assuming L(k),V(k) indicate input at time step k-1
    else
        z = phi*z + psi*[L(k)-L(k-1);V(k)-V(k-1)];
    end
    %size(x)
    % Calculation of filtering error covariance matrix
    % size(R1)
%     disp('phi');size(phi)
%     disp('R1');size(R1)
%     disp('P');size(P)
    Pbar = phi*P*phi' + R1;
    % Kalman Filter gain
   % R2 = 
%     disp('xi');
%     size(xi)
%     disp('Pbar');
%     size(Pbar)
%     disp('R2'); 
%     size(R2)
    K = Pbar*xi'*inv(xi*Pbar*xi'+ R2);
    % Estimate
    %size(ym(k,:))
%     size((ym(k,:)-H*x))
%     size(K)
    P = (eye(size(K*xi))-K*xi)*Pbar;    
    %size(C)
    ym_pred = C*z(1:20) + ym_pred;
    zhat(k,:) = z + K*(ym(k,:)'- ym_pred);
    z = zhat(k,:)';
    % Obtain the controlled variable value
    yc_pred = H*z(1:20) + yc_pred;
    YHAT(k,:) = yc_pred;
end



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


