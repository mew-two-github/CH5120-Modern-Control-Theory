function [bigSu,Hess]=mimo_dmc_fcn(p,m)
load SVal_MIMO_hw5 Smodel n nu ny

% *** Smodel, n, nu, ny are pre-loaded for you ****
% Please start typing your solution below this line
size(Smodel)
bigSu = [Smodel(1:ny*p,1) Smodel(1:ny*p,2)];
for i = 1:(m-1)
    s1 = zeros(ny*p,1);
    s2 = zeros(ny*p,1);
    s1(i*ny+1:p*ny) = Smodel(1:p*ny-i*ny,1);
    s2(i*ny+1:p*ny) = Smodel(1:p*ny-i*ny,2);
    %size(s2)
    bigSu = [bigSu s1 s2];
end
bigSu
gamma_y = eye(ny*p);
gamma_u = eye(nu*m)*0.25;
%size(bigSu)
%size(gamma_y)
Hess = bigSu'*gamma_y*bigSu + gamma_u;