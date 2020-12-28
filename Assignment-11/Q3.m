% DMC SISO with plant model mismatch
clear; close all;
%% Model
a = 0;
tau = (a+1)/2;
% Plant Model
G = tf(5,[tau,1],'inputdelay',0.15);
sys = ss(G);
sampling_time = 0.25;
model = c2d(sys,sampling_time);
% Actual Plant
Gplant = tf(5.4,[tau 1],'inputdelay',0.12);
p = ss(Gplant);
plant = c2d(p,sampling_time);
%% Setting up the Controller
p = 10; % Prediction horizon
m = 4; % Control Horizon
Q = 1; % Output weight
R = 0.1; % Input rate weight
Weights.ManipulatedVariablesRate = R;
Weights.OutputVariables = Q;
Weights.ManipulatedVariables = 0;
mp1 = mpc(model,sampling_time,p,m,Weights);
% Input constraints
mp1.ManipulatedVariables.Min = -0.4;mp1.ManipulatedVariables.Max = 0.4;
% Input rate constraints
mp1.ManipulatedVariables.RateMin = -0.025; mp1.ManipulatedVariables.RateMax = 0.025;
%% Running the controller
ySP = 1; % Setpoint
T = 50; % Number of simulation steps
options = mpcsimopt();
options.Model = plant; % Incorporating the actual plant model
options.InputNoise = 0; % Setting measurement noise to zero
options.OutputNoise = 0;
[y,t,u,xp,xmpc,SimOptions,status] = sim(mp1,T,ySP,options);
%% Plotting the results
plot(t,y,'-b',...               % Plots the output
    t,ones(size(t)),'--r','linewidth',1);  % Plots  set-point
set(gca,'fontsize',12,'fontweight','bold');
title('Output vs time','fontsize',14,'fontweight','bold');
xlabel('Time (in seconds)');ylabel('Output y');
figure();
stairs(t,u); % Plots the input
set(gca,'fontsize',12,'fontweight','bold');
title('Input moves vs time','fontsize',14,'fontweight','bold')
xlabel('Time (in seconds)');ylabel('Input u');
