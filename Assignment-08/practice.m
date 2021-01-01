% First problem we just have to convert it tf to ss and then apply LQR and
% pole placement same as q1 and q2
%% Problem-2
sampling_time = 2;
NUM = {[2] [0.5];[1.2] [1]};
DEN = {[40 16 1] [20 7 1];[10 5 1] [36 12 1]};
G = tf(NUM,DEN);
sysc = ss(G);
sysd = c2d(sysc,sampling_time);
%% Pole placement controller
% Maximum multiplicity of the poles <= rank(B)
p = [0.1 0.1 0.2 0.2 0.3 0.3 0.15 0.15]; 
[K,prec,message] = place(sysc.A,sysc.B,p);
%% LQR controller
% Tuning Matrices
Q = eye(8);
R = eye(2);
[X,L,G] = dare(sysd.A,sysd.B,Q,R,zeros(size(sysd.B)),eye(8));
Linf = G;
LQRpoles = L;