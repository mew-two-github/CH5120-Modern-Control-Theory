del_t = 0.5;
a = 0;
tau = 0.5*(1+a);
t = (0:del_t:10*tau)';
% part 4
hy = zeros(length(t),1);
for i = 1:length(t)
    if t(i) < 1
        hy(i) = 5*(1-exp(-t(i)/tau));
    else
        hy(i) = 5*(exp(-(t(i)-tau)/tau)-exp(-t(i)/tau));
    end
end
%part-4 MATLAB
G = tf(5,[tau,1]);
hy_MATLAB = impulse(G,t);
%part 5-run the P1.m and dont clear the variables in WS
hy2 = zeros(length(t),1);
for i = 2:length(t)
    hy2(i) = y(i)-y(i-1);
end

figure();
plot(t,hy,t,hy2);
title('Finite Impulse Response Curves');
legend('Evaluation of ss eqn','From FSR coefficients')