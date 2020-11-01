G=tf(5,[4.2 1],'inputdelay',0.15);
h=0.5;

% State space model parameters
Gd=c2d(ss(G),h);
A=Gd.A; B=Gd.B; C=Gd.C;

% Step response model parameters
n=40;
S=step(G,[0:h:n*h]); S=S(2:end);
S=round(S,1);   % This line is added to ensure Y_fsr and Y_ss are different

% Input moves
U=randi([-3,3],1,21)/2;

% NOTE
% Y_fsr is 1*20 array of results from step-response model
% Y_ss  is 1*20 array of results from state-space model
% -------------------------------------------------------
% Please start typing from lines below
Y_fsr = S(1:20)'.*(U(1));
%ith element of U denotes (i-1)th time step
for i = 2:20
    Y_fsr(i:20) = Y_fsr(i:20) + (S(1:(21-i))'.*(U(i)-U(i-1)));
end
Y_ss = zeros(1,20);
X = zeros(2,20);
for k = 1:20
    if k ~=1
        X(:,k) = A*X(:,k-1) + B*U(k);
    else
        X(:,k) = B*U(k);
    end
    Y_ss(k) = C*X(:,k);
end
