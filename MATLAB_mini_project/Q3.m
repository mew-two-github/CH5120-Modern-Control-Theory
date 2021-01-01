clear; close all;
RollNo='CH18B020';      % Please replace with your roll number
n= 25;                  % n>p ensured
h= 5;                   % Sampling interval
maxTime=50;             % Run this case for 50 time-steps
%% Converting transfer function model to Step response model (for input)
G=tf(4,[40 1],'inputdelay',6);
[y,t]=step(G,0:h:n*h);
S=y(2:end);
%% Disturbance Response model
Gd = tf(0.8,[24 1],'inputdelay',4);
[yd,td]=step(Gd,0:h:n*h);
Sd=yd(2:end);
%% Controller Parameters
ySP=0.6;          % Setpoint
m=5;            % Control horizon
p=12;           % Prediction horizon
Q=1;            % Output weight
R=0.2;         % Input weight
duMax=0.2;     % Input rate constraint
duMin=-duMax;
uMin=-1;      % Input constraint
uMax=1;

%% Initialization
uPrev=0.0;
Yk0=zeros(n,1);
Y_SAVE=zeros(maxTime+1,1);
T_SAVE=zeros(maxTime+1,1);
U_SAVE=zeros(maxTime,1);

%% Precompute Matrices
S_col = S(1:p);
bigSu = zeros(p,m);
for i = 1:m
    bigSu(:,i) = S_col;
    S_col = [0;S_col(1:p-1)];
end
bigR=ones(p,1)*ySP;
% Weight matrices
gamma_U = eye(m)*R;
gamma_Y = eye(p)*Q;
% Hessian
Hessian = bigSu'*gamma_Y*bigSu + gamma_U;
% Some constants for constraints
Im = eye(m);
bigIm = tril(ones(m),0);
c_LHS = [Im;-1*Im;bigIm;-1*bigIm];
Yhat = [];
err = zeros(maxTime+1);
%% Implementation
uPrev = 0;
d = zeros(maxTime+1,1)+0.2; %Step disturbance; d(1) denotes 0th step
dPrev = (0);
for k = 1:maxTime+1
    t = (k-1)*h;
    delta_d = d(k)-dPrev;
    dPrev = d(k);
    if k == 1
        Yhat = Yk0;
    end
    % Compute Gradient
    predErr = Yhat(2:p+1)+Sd(1:p)*delta_d-bigR;
    g = bigSu'*gamma_Y*(predErr);
    c_RHS = [repmat(duMax,m,1);repmat(-duMin,m,1);repmat(uMax-uPrev,m,1);repmat(-(uMin-uPrev),m,1)];
    % Solve the Quadratic Programming Problem
    big_dU = quadprog(Hessian,g,c_LHS,c_RHS);
    du = big_dU(1);
    uk = uPrev+du;
    % Perform the input move
    Yhat=[Yhat(2:end);Yhat(end)] + S*du + Sd*delta_d;
    % Storing Results
    uPrev = uk;
    U_SAVE(k) = uk;
    Y_SAVE(k+1) = Yhat(1);
    T_SAVE(k+1) = k*h;
end

%% Plotting the results
subplot('position',[0.12 0.58 0.8 0.4]); 
plot(T_SAVE(1:maxTime),Y_SAVE(1:maxTime),'-b',...               % Plots the output
    T_SAVE(1:maxTime),ones(maxTime,1)*ySP,'--r','linewidth',1);     % Plots  set-point
ylabel('Output, y_k');
subplot('position',[0.12 0.12 0.8 0.4]); 
stairs(T_SAVE(1:maxTime),U_SAVE(1:maxTime),'-b','linewidth',1); % Plots the  input 
ylabel('Input, u_k'); xlabel('time, t')
