clear; close all;
RollNo='CH18B020';      % Please replace with your roll number
n= 25;                  % n>p ensured
h= 5;                   % Sampling interval
maxTime=50;             % Run this case for 50 time-steps
ySP = 0.6;              % Set-point    
%% Case-1: m=p=5
m=5;           % Control horizon
p=5;           % Prediction horizon
[t1,u1,y1] = dmc(p,m);
figure();
subplot(2,1,1); 
plot(t1(1:maxTime),y1(1:maxTime),'-b',...               % Plots the output
    t1(1:maxTime),ones(maxTime,1)*ySP,'--r','linewidth',1);     % Plots  set-point
ylabel('Output, y_k'); title('m = 5, p = 5')
subplot(2,1,2); 
stairs(t1(1:maxTime),u1(1:maxTime),'-b','linewidth',1); % Plots the  input 
ylabel('Input, u_k'); xlabel('time, t');
% Comparing with part-(a), the output curves look identical, both reach and  
% settle at the new set point in ~ 50 units (=10 time steps). The input  
% moves are also largely same.
%% Case-2 m=1, p = 12
m=1;            % Control horizon
p=12;           % Prediction horizon
[t2,u2,y2] = dmc(p,m);
figure();
subplot(2,1,1); 
plot(t2(1:maxTime),y2(1:maxTime),'-b',...               % Plots the output
    t2(1:maxTime),ones(maxTime,1)*ySP,'--r','linewidth',1);     % Plots  set-point
ylabel('Output, y_k'); title('m = 1, p = 12')
subplot(2,1,2); 
stairs(t2(1:maxTime),u2(1:maxTime),'-b','linewidth',1); % Plots the  input 
ylabel('Input, u_k'); xlabel('time, t');
% Compared to the other two cases, the control is sluggish, it takes a lot
% more time to make the system reach the set point: ~150 units (=30 time
% steps)
%% Conclusion:
% Low prediction horizon (p) and high input horizon (m) enables the 
% controller to act more aggresively, making the system have a sharp rise 
% in the initial stage.
% On the other hand decreasing m and increasing p promotes sluggishness as 
% seen in the slow rise to the set point in case of m=1.
%% A function to perform the DMC operation
function [T_SAVE,U_SAVE,Y_SAVE] = dmc(p,m)
    n= 25;                  % n>p ensured
    h= 5;                   % Sampling interval
    maxTime=50;             % Run this case for 50 time-steps
    % Step response model
    G=tf(4,[40 1],'inputdelay',6);
    [y,t]=step(G,0:h:n*h);
    S=y(2:end);
    % Disturbance Response model
    Gd = tf(0.8,[24 1],'inputdelay',4);
    [yd,td]=step(Gd,0:h:n*h);
    Sd=yd(2:end);
    ySP=0.6;          % Setpoint
    Q=1;            % Output weight
    R=0.2;         % Input weight
    duMax=0.2;     % Input rate constraint
    duMin=-duMax;
    uMin=-1;      % Input constraint
    uMax=1;
    % Initialization
    uPrev=0.0;
    Yk0=zeros(n,1);
    Y_SAVE=zeros(maxTime+1,1);
    T_SAVE=zeros(maxTime+1,1);
    U_SAVE=zeros(maxTime,1);

    % Precompute Matrices
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
    Im = eye(m);
    bigIm = tril(ones(m),0);
    c_LHS = [Im;-1*Im;bigIm;-1*bigIm];
    Yhat = [];
    err = zeros(maxTime+1);
    % Implementation
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
end