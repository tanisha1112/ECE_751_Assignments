seed = 1344; 
rng(seed,'twister');
clc; 
clear;
close all;

% at t=0
% P = position
% S = speed
% T = sample time  


% T = no. of samples

T = 0.1;
N = 100; 
t = (1:N)*T;

% a = acceleration a ~ N(0,sigma)
sigma_a = sqrt(40);

%measurement variables
C = [1 0 ; 0 1];
sigma_w = sqrt(100); 


% P_0 = 1000;
% S_0 = -50;
X_0 = [1000, -50];

%initialise state vectors 
F = [1 T ;0 1];      
G = [1/2 * (T^2); T];
% X = zeros(2,T);

[X, Y] = kalman_filter(N,F,G,C,sigma_w,sigma_a, X_0);

%% Testing 

C = [1 0];
Q = sigma_a ^2;
R = sigma_w ^2;
X_hat_0 = [0 0];
P_0 = [1000 0 ; 0 1000];

[X_hat,P, K_hat, P_mse] = kalman_filter_estimate(N,F,C,G,Q,R,X_hat_0, P_0,Y);


figure;
subplot(2,1,1);
plot(t,Y(1,:),'LineWidth',2);
hold on; 
plot(t,X(1,:),".",'LineWidth',2);
hold on ;
plot(t,X_hat(1,:),"--",'LineWidth',2);
ylim([400 1000])
legend('True', 'Noisy', 'Estimate');
xlabel('t');
ylabel('Position (m)');

subplot(2,1,2);
plot(t,X(2,:),'LineWidth',2);
hold on; 
plot(t,Y(2,:),".",'LineWidth',2);
hold on ;
plot(t,X_hat(2,:),"--",'LineWidth',2);
legend('True', 'Noisy', 'Estimate');
ylim([-100 200])
xlabel('t');
ylabel('Velocity (m/s)');

figure;
subplot(1,2,1);
plot(t,K_hat(1,:),'LineWidth',1);
ylim([0 1.5]);
xlabel('t');
ylabel('Kalman gain position');
subplot(1,2,2);
hold on;
plot(t,K_hat(2,:),'LineWidth',1);
ylim([0 1.5]);
xlabel('t');
ylabel('Kalman gain velocity');

figure;
subplot(1,2,1);
semilogy(t,P_mse(1,:),'LineWidth',1);
xlabel('t');
ylabel('MSE position');
subplot(1,2,2);
semilogy(t,P_mse(2,:),'LineWidth',1);
xlabel('t');
ylabel('MSE velocity');

function [X_hat,P, K_hat, P_mse] = kalman_filter_estimate(N,F,C,G,Q,R,X_hat_0, P_0, Y)

Nx = size(F,1);
X_hat = zeros(Nx, N);
r_hat = zeros(Nx, N);
X_hat(:,1) =X_hat_0;
I = [1 0 ; 0 1];
K_hat = zeros(Nx, N);
P_mse =  zeros(Nx, N);

P = P_0;

for k = 1:N-1
    X_hat(:,k +1) = F * X_hat(:,k);
    P = F * P * transpose(F) + G * Q * transpose(G);
    r_hat(:,k +1) = C * X_hat(:,k);
    P_hat = C *P * transpose(C) + R;

    K = P * transpose(C)/(P_hat);
    K_hat(:,k)= K;
    X_hat(:,k +1) = F * X_hat(:,k) + K*(Y(1,k) - C * F * X_hat(:,k));
    P = (I - K*C)*P;
    P_mse(:,k) = [P(1),P(4)]; 

end

end




function [X,Y] = kalman_filter(N,F,G,C,sigma_w,sigma_a, X_0)

Nx = size(F,1);
Nr = size(C,1);
X = zeros(Nx, N);
X(:,1) =X_0;

Y = zeros(Nr,N);

w = normrnd(0,sigma_w ^0.5,[Nr,N]);
a = normrnd(0,sigma_a ^0.5,[Nx,N]);


for k = 1:N-1
% initialise equations
    X(:, k+1) = F * X(:,k) + G * a(k);

end

Y = C * X + w;
end









