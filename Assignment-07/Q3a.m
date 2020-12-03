clear;
%% Given conditions
n = 4; % Number of states
A = [0.65 0.1 0 -0.45;0.35 0.4 0.3 -0.15;0 0.25 0.35 -0.2;-0.2 -0.05 -0.15 0.4];
B = [0.5;0.5;0;0];
C = [2 0 0 -2];
%% Computing Grammian
Wc = repmat(A*B,[1,4]);
for i = 1:4
    Wc(:,i) = A^(i-1)*B;
end
% Finding its rank
rank_Wc = rank(Wc);
isCtrl = logical(rank(Wc) == n);
%% Hautus Condition
lambda = eig(A);
isStab = true;
for i = 1:length(lambda)
    if lambda(i) > 1
        M = [A-lambda(i)*eye(size(A)) B];
        if rank(M) ~= n
            isStab = false;
        end
    end
end