% DMC SISO with no disturbances and measurement errors
clear; close all;
%% Models
a = 0;
tau = (a+1)/2;
% Plant model
G = tf(5,[tau,1],'inputdelay',0.15);
sys = ss(G);
sampling_time = 0.25;
plant_model = c2d(sys,sampling_time);
% Disturbance model
Gdist = tf(0.4,[2,1],'inputdelay',0.1);
dist = ss(Gdist);
dist_model = c2d(dist,sampling_time);
%Combined Model
tf_model = [G Gdist];
ss_model = ss(tf_model);
model = c2d(ss_model,sampling_time);
%% Setting up the Controller
p = 10; % Prediction horizon
m = 4; % Control Horizon
Q = 1; % Output weight
R = 0.1; % Input rate weight
Weights.ManipulatedVariablesRate = R;
Weights.OutputVariables = Q;
Weights.ManipulatedVariables = 0;
model2 = setmpcsignals(model,'MD',2);
mp1 = mpc(model2,sampling_time,p,m,Weights);
mp1.ManipulatedVariables.Min = -0.4;mp1.ManipulatedVariables.Max = 0.4;
mp1.ManipulatedVariables.RateMin = -0.025; mp1.ManipulatedVariables.RateMax = 0.025;
% Dist.Name = 'd';
% Dist.Units = 1;
% mp1.DisturbanceVariables = Dist;
% mp1.Disturbance = dist_model;
%% Running the controller
ySP = 1; % Setpoint
T = 50; % Number of simulation steps
d = 0.5*ones(T,1);  % Measured Disturbance
options = mpcsimopt();
options.InputNoise = 0;
options.OutputNoise = 0;
[y,t,u,xp,xmpc,SimOptions,status] = sim(mp1,T,ySP,d,options);
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