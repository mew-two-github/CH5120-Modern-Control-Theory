function [XALL,YALL] = estimation_data(Roll)
    p = Roll(7)-'0';
    q = Roll(8) - '0';
    A = [0.2*p -0.4;0.1*q 0.25];
    B = [0;0];
    C = [1 0];
    G = [0.2;1.2];
    D = 0;
    H = 0;  
    epsilon = randn(200,1);
    mu = randn(200,1)*sqrt(0.25);
    XALL = ones(2,200); YALL = zeros(200,1);
    YALL(1) = C*XALL(:,1) + mu(1);
    for k = 2:200
        XALL(:,k) = A*XALL(:,k-1) + G*epsilon(k);
        YALL(k) = C*XALL(:,k) + mu(k);
    end
end
