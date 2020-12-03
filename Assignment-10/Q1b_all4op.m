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
phi = [A zeros(20,4);[C;H]*A eye(4)];
psi = [Bw;C*Bw];
gamma = [B;[C;H]*B];
xi = [zeros(4,20) eye(4)];
y0 = zeros(4,1);
z = [x0;y0];
R1 = [Bw;[C;H]*Bw]*R1*[Bw;[C;H]*Bw]';
YHAT = zeros(200,2);
yc_pred= zeros(2,1);
ym_pred = zeros(2,1);
P=0.01*eye(24);
zhat = zeros(200,24);
R2 = [R2 zeros(2,2);zeros(2,4)];
for k = 1:200
    % Prediction
    if k==1
        z = phi*z + gamma*[L(k);V(k)]; % assuming L(k),V(k) indicate input at time step k-1
    else
        z = phi*z + gamma*[L(k)-L(k-1);V(k)-V(k-1)];
    end
    % Calculation of filtering error covariance matrix
    Pbar = phi*P*phi' + R1;
   % size(xi*Pbar*xi')
    % Kalman Filter gain
    K = Pbar*xi'*inv(xi*Pbar*xi'+ R2);
    P = (eye(size(K*xi))-K*xi)*Pbar;   
    ym_pred = xi*z;
    ym_pred = ym_pred(1:2);
    zhat(k,:) = z + K*[(ym(k,:)'- ym_pred);0;0];
    z = zhat(k,:)';
    % Obtain the controlled variable value
    yc_pred = xi*z;
    YHAT(k,:) = yc_pred(3:4);
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


