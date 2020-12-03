h=0.2; tEnd=5;
x0=[0;0];
uStep=0.05;
% Please report the concentrations in the vector Cdash
% Please start typing from line below
Area = 0.2;
uss =0.5;
k = 1.5;
kappa = 0.5;
cin = 4;
%Steady State values
xss = fsolve(@(x) rxtrFun(x,uss), [1;1]);
hss = xss(1); css=xss(2);
%A,B,C calculated in the previous assignment
A = [-kappa/(2*Area*(hss)^0.5) 0;-uss*(cin-css)/(Area*hss^2) -2*k*css-uss/Area*hss];
B = [1/Area;(cin-css)/(Area*hss)];
C = eye(2);
%dx'/dt = Ax'+Bu' where ' indicates deviation variable
%So discretising it
M = expm(A*h);
N = A\(expm(A*h)-eye(2))*B;
u = 0.05;
x = [0;0];
Cdash = zeros(1,25);
for  i = 1:25
    x = M*x + N*u;
    y = C*x;
    Cdash(i) = y(2);
end
%Reactor Function same as previous question, used to find steady state
%values
function dx=rxtrFun(x,u)
Area=0.2; kappa=0.5; k=1.5; 
dx = zeros(2,1);
Cin=4;
h=x(1);
C=x(2);
disp(u/Area-kappa/Area.*sqrt(h));
dx(1,1)=u/Area-kappa/Area.*sqrt(h);
dx(2,1)=u/(Area*h)*(Cin-C)-k*C^2;
end