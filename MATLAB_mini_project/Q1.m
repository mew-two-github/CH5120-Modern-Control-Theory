rollNo='CH18B020';       % Please type your roll number
% Please start typing from the lines below
A = [2.700 2 0 0;-1.23 0 1 0;0.436 0 0 0.25;-0.192 0 0 0];
B = [0;0.5;-0.35;0.2];
C = [1 0 0 0];
%% Stability
lambda = eig(A);
isStable = false;
if sum(lambda>1) == 0
    isStable = true;
end
% The eigen values are 1.2,0.8,0.5 and 0.2
% Since one of the eigen values is greater than one the system is unstable
%% Controllability
Wc = zeros(4,4);
for i = 1:4
    Wc(:,i) = A^(i-1)*B;
end
r_Wc = rank(Wc);
isCont = false;
if r_Wc == 4
    isCont = true;
end
% Since rank(Wc) < n (Wc is not full rank), the system is uncontrollable
%% Uncontrollable subspace
[u,s,v] = svd(Wc);
% We see that the last 2 of the singular values are 0. So,
% Uncontrollable subspace is (eigen vectors corresponding to 0 singular values 
% span the uncontrollable subspace)
uncont_sub = u(:,3:4);
% Uncontrollable subspace is spanned by column vectors of uncont_sub
%% Hautus condition
M = zeros(size(A));
isStabi = true;
for i = 1:4
    M  = [A-lambda(i)*eye(4) B];
    if lambda(i)>=1 && rank(M) < 4
        isStabi = false;
        break;
    end
end
% For eigen values >= 1 the rank of M is = 4; other eigen values are stable. So 
% So according to Hautus condition the system is stabilizable.
%% Observability
Wo = zeros(4,4); 
% Observability grammian
for i = 1:4
    Wo(i,:) = C*A^(i-1); 
end
isObs = false;
if rank(Wo) == 4
    isObs = true;
end
% Observability grammian is full rank. So, the system is observable
%% Unobservable subspace
[u2,s2,v2] = svd(Wo);
unobs_sub = [];
% None of the singular values are zero(as expected, because Wo is full rank)
% so the system is observable