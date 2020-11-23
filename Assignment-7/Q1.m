Ac = [-1 0 1;0 0 1;0 0 0];
Bc = [1; 0; 0];
Gs = ss(Ac,Bc,[1 1 1],0);
Gd = c2d(Gs,0.2);
A = Gd.A;B=Gd.B;
ei = eig(A);
M1 = [A-ei(1)*eye(size(A)) B];
M23 = [A-ei(2)*eye(size(A)) B];
r1 = rank(M1);
r2 = rank(M23);