load SVal_hw4.mat       % Load S matrix
nu=1; ny=2;             % Number of inputs and outputs
n=25;                   % Number of time steps in S matrix
U=(0.5-rand(1,41))/10;  % Input moves
% The first line loads your S matrix
% ***** Start typing from the line below *****
h = 0.2;tEnd = 8;
%output vector
Y = zeros(1,50);
%Required values
C_result = zeros(1,40);
for i = 1:tEnd/h
    %Shift
    Y(1:48) = Y(3:50); 
    if i ~= 1
        Y = Y + S.*(U(i)-U(i-1));
    else
        Y = Y + S.*(U(i));
    end
    C_result(i) = Y(2);
end
C_result