a=0;     % Replace with last digit of your roll number
tau=(1+a)/2;
Gc=tf(5,[tau,1],'inputdelay',0.15);
% Getting the step-response matrix
h=0.5;
n=40;
S=step(Gc,0:h:n*h);
S=S(2:end);
S=round(S,2);    % Rounding off to introduce some errors
S = S(1:5);
S2 = zeros(5,1);
S2(2:5) = S(2:5);
Su = [S,S2];
H = Su'*Su + eye(2)*0.5;