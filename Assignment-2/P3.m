% Transfer functions
G11=tf(2,[40,16,1]);
G12=tf(0.5,[20,7,1]);
G21=tf(1.2,[10,5,1]);
G22=tf(1,[36,12,1]);
G=[G11, G12; G21, G22];

% Parameters
ts=2; n=25;

% --- Start typing your code below this line ---
y = step(G,[0:ts:n*ts]);
S = zeros(50,2);
for i=1:25
    S(2*i-1:2*i,:) = y(i+1,:,:);
end