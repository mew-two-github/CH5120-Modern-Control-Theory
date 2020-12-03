clear; close all;
RollNo='CH18B020';      % Please replace with your roll number
n=24;
% Please replace with your chosen n
h=5;                  % Sampling interval: Don't change
maxTime=50;             % Run this case for 50 time-steps
% Please start typing from the line below
%% Step response model
G=tf(2.5,[20 1],'inputdelay',7);
[y,t]=step(G,0:h:n*h);
S=y(2:end);
%% Plant model
Gplant = tf(2.75,[18.5 1],'inputdelay',6.2);
[yplant,tplant]=step(Gplant,0:h:n*h);
Splant=yplant(2:end);
%% Controller Parameters
ySP=1;          % Setpoint
m=4;            % Control horizon
p=10;           % Prediction horizon
Q=1;            % Output weight
R=0.04;         % Input weight
duMax=0.05;     % Input rate constraint
duMin=-duMax;
uMin=-0.5;      % Input constraint
uMax=0.5;

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
bigR=ones(p,1)*ySP; %Since ny=1
gamma_U = eye(m)*R;
gamma_Y = eye(p)*Q;
Hessian = bigSu'*gamma_Y*bigSu;
Im = eye(m);
bigIm = tril(ones(m),0);
c_LHS = [Im;-1*Im;bigIm;-1*bigIm];
Yhat = [];
% Accomodating the plant model
Yplant = [];
Ip = ones(p,1);
err = zeros(maxTime+1);
%% Implementation
uPrev = 0;
d = zeros(maxTime+1,1)+0.5; %Step disturbance; d(1) denotes 0th step
d(13:20,:) = zeros(20-13+1,1)+1;
d(21:maxTime+1) = zeros(length(21:1:maxTime+1),1)+0.2;
dPrev = (0);
for k = 1:maxTime+1
    t = (k-1)*h;
    % Find delta dk
    delta_d = d(k)-dPrev;
    % Update dPrev
    dPrev = d(k);
    % Initialise Yhat and Yplant
    if k == 1
        Yhat = Yk0;
        Yplant = Yk0;
    end
    % Compute model-plant mismatch error
    if k ~= 1
        err(k) = Yplant(1) - Yhat(1);
    end
    % Compute Gradient
    g = bigSu'*gamma_Y*(Yhat(2:p+1)+Ip*err(k)-bigR);
    c_RHS = [repmat(duMax,m,1);repmat(-duMin,m,1);repmat(uMax-uPrev,m,1);repmat(-(uMin-uPrev),m,1)];
    % Solve the Quadratic Programming Problem
    big_dU = quadprog(Hessian,g,c_LHS,c_RHS);
    du = big_dU(1);
    uk = uPrev+du;
    % Perform the input move
    Yhat=[Yhat(2:end);Yhat(end)] + S*du + err(k);
    Yplant = [Yplant(2:end);Yplant(end)] + Splant*du;
    % Storing Results
    uPrev = uk;
    U_SAVE(k) = uk;
    Y_SAVE(k+1) = Yhat(1);
    T_SAVE(k+1) = k*h;
end

%% Plotting the results (you may edit these lines, if needed)
subplot('position',[0.12 0.58 0.8 0.4]); 
plot(T_SAVE(1:maxTime),Y_SAVE(1:maxTime),'-b',...               % Plots the output
    T_SAVE(1:maxTime),ones(maxTime,1),'--r','linewidth',1);     % Plots  set-point
ylabel('Output, y_k');
subplot('position',[0.12 0.12 0.8 0.4]); 
stairs(T_SAVE(1:maxTime),U_SAVE(1:maxTime),'-b','linewidth',1); % Plots the  input 
ylabel('Input, u_k'); xlabel('time, t')
