clear; close all;
%% Given conditions
n = 4; % Number of states
A = [0.65 0.1 0 -0.45;0.35 0.4 0.3 -0.15;0 0.25 0.35 -0.2;-0.2 -0.05 -0.15 0.4];
B = [0.5;0.5;0;0];
C = [2 0 0 -2];
%% Computing Observability Grammian
Wo = repmat(C,[n,1]);
for i = 1:n
    Wo(i,:) = C*A^(i-1);
end
% Finding its rank
rank_Wo = rank(Wo);
isObsv = logical(rank_Wo==n);