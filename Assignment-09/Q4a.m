% Please enter your solution below
Roll_No='CH18B020';     % Please replace with your roll number
[XALL,YALL]=estimation_data(Roll_No);

% Please start typing from the line below
% ---------------------------------------
A=[0.2 -0.4;0 0.25];                    % <<<<< Please enter your 'A' matrix here <<<<<
Be=[0.2;1.2];
C=[1, 0];
Q=1;
R=0.25;

% Observer gain
K_db=[0.45;-0.15625];                 % <<<<< Please enter deadbeat observer gain <<<<<

%% Running the estimator
xhat=[0;0];
XHAT=zeros(2,200);
for k=1:200
    % Deterministic observer
    xhat=A*xhat + K_db*(YALL(k)-C*xhat);
        
    % Storing the result
    XHAT(:,k)=xhat;
end

%% Plot the results
subplot(2,1,1)
plot(1:200,XALL(1,:), 1:200,XHAT(1,:));
subplot(2,1,2)
plot(1:200,XALL(2,:), 1:200,XHAT(2,:));

%% Compute SSE
sqErr=(XALL-XHAT).^2;
SSE=sum(sum(sqErr));
