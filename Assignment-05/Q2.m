%% NOTE: CAN'T BE RUN WITHOUT CREATING SVal_CSTR_hw5.mat
load SVal_CSTR_hw5.mat;  % Load Smodel and nu,ny,n
p=5;
m=2;

% The code must return the matrices bigSu and Hess
% -- Start writing the code from the line below --
s2 = zeros(1,ny*p)';
s2(ny+1:p*ny) = Smodel(1:p*ny-ny);
bigSu = [Smodel(1:ny*p) s2]
gamma_y = zeros(10,10);
Q = [0.25 0;0 1];
for i = 1:5
    gamma_y(i*2-1:2*i,i*2-1:2*i) = Q;
end
%size(bigSu)
%size(gamma_y)
gamma_u = eye(2)*0.1;
Hess = bigSu'*gamma_y*bigSu + gamma_u