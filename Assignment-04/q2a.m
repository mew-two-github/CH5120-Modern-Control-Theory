% Compute the steady state values
uss=0.5;
xss=fsolve(@(x) rxtrFun(x,uss), [1;1]);
% You may use the above as initial condition or use your own
% Please report your solution in 1*25 array C_nonlin
% ***** Start typing from line below *****
h=0.2; tEnd=5;    % <-- Sampling and Final times
diff_eqn = @(t,x)(rxtrFun(x,uss*1.1));
[tOUT,xOUT] = ode15s(diff_eqn,[0:h:tEndx],xss);
C_nonlin = xOUT(2:26,2)';
% =========================================================================
%% Function for use with ode15s (You can delete this function, if required)
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
