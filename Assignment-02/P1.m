del_t = 0.5;
a = 0;
tau = 0.5*(1+a);
t = (0:del_t:10*tau)';
%part 1
%derived in solution
y  = 5.*(1-exp(-t./tau));
%Using MATLAB solution
G = tf(5,[tau,1]);
y_MATLAB = step(G,t);
%part 2
theta = 0.15;
y2 = zeros(length(t),1);
for i = 1:length(t)
    if t(i) < theta
        y2(i) = 0;
    else
        y2(i) = 5*(1-exp(-(t(i)-0.15)/tau));
    end
end
%Part 2 MATLAB
G = tf(5,[tau,1],'InputDelay',theta);
y2_MATLAB = step(G,t);
%part 3
theta = 1.5;
y3 = zeros(length(t),1);
for i = 1:length(t)
    if t(i) < theta
        y3(i) = 0;
    else
        y3(i) = 5*(1-exp(-(t(i)-1.5)/tau));
    end

end
%part 3 MATLAB
G = tf(5,[tau,1],'InputDelay',theta);
y3_MATLAB = step(G,t);
plot(t,y,t,y2,t,y3);
title('Finite Step Response Curves');
legend('No Delay','0.15 Delay','1.5 Delay');
