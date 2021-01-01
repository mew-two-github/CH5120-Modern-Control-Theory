%% Converting to discrete time state space model
% Declaring transfer function model
ny = 1; nu =2;
NUM = {[2.5] [0.8]};
DEN = {[40,16,1] [24,1]};
G = tf(NUM,DEN);
G.InputGroup.controls = 1;
G.InputGroup.noise = 2;
% Converting to state space
sys = ss(G);
% Discretising the model
sampling_time = 5;
model = c2d(sys,sampling_time);
A = model.A;
B = model.B;
% column of B multiplying with second input is the one that multiplies with
% the noise (epsilon). (because noise is given as the second input)
Be = B(:,2); 
% The other column multiplies with the actual input u
B = B(:,1);
C = model.C;
%% Determining steady state Kalman Filter
QN = 0.25; RN = 0.1;
N =0;
%sys = ss(A,[B Be],C,[0 0],-1);
[KEST,L,P,Mx,Z,My] = kalman(model,QN,RN,N);
Kinf = A\L; % Steady state Kalman Filter Gain
Kbar_inf = L; % Steady state predictor gain