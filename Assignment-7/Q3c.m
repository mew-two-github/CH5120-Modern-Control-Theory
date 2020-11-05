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
%% Performing SVD & Coord Transformation
[U,S,V] = svd(Wc);
Ahat = U\A*U;
Bhat = U\B;
Chat = C*U;
