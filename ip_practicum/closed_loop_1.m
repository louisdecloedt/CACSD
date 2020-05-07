clear;
close all;

R_m = 2.6; % ohm, Motor armature resistance
K_m = 0.00767; % N * m / A, Motor torque constant
K_b = 0.00767; %V / (rad/s), Motor back EMF constant
K_g = 3.7; %:1, (The output is lower)
M = 0.455; %kg, Cart mass with motor and parts
l = 0.305; %m, Rod length
m = 0.210; %kg, Rod mass
r = 0.635e-2; %m, Radius of motor output gear
g = 9.81; %m/(s^2)

%State-Space model:
A = zeros(4,4);
A(1,3) = 1;
A(2,4) = 1;
A(3,2) = - m*g/M;
A(3,3) = - (K_g^2*K_m*K_b)/(M*R_m*r^2);
A(4,2) = (M+m)*g/(M*l);
A(4,3) = (K_g^2*K_m*K_b)/(M*R_m*r^2*l);

B = zeros(4,1);
B(3,1) = (K_m*K_g)/(M*R_m*r);
B(4,1) = -(K_m*K_g)/(r*R_m*M*l);

%Assumption whole state vector is available.
C = eye(4);
 
D = zeros(4,1);


%LQR controllor
%initital Q,R values given in the assignment;
% Q0 = [0.25 0 0 0
%       0 4 0 0
%       0   0 0 0
%       0  0  0 0];
% R0 = 0.003;

Q0 = [2 0 0 0
      0 3 0 0
      0 0 0 0
      0 0 0 0];
R0 = 0.005;

%R0 0.006 -6.4550  -41.2918  -12.7949   -5.4042

%K0       -9.1287  -52.5082  -14.9874   -6.3524

%R0 0.001 -15.8114  -81.7871  -20.9428   -8.8041 

% 
%K0 -9.1287  -52.5082  -14.9874   -6.3524

%Determine K
%[K,S,e] = lqr(SYS,Q,R,N)
%https://nl.mathworks.com/help/control/ref/lqr.html
%N = optional, default zero (N = 0 is what we need)
sys = ss (A,B,C,D);
[K,S,e] = lqr(sys,Q0,R0);
%size(K)
K = K(1,:);

K

% figure
% pzmap(sys);


%K = [-818.244 -513.84 -212.95 -72.08]
eig(A-B*K)

K

% Defining the closed-loop system
% x_dot = (A - B*K)*x
% y = c*x

A_cl = A - B*K;
B_cl = [0;0;0;0];
C_cl = C;
D_cl = D;
sys_cl = ss(A_cl,B_cl,C_cl, D_cl);
sys_cl.OutputName={'z','\alpha','dz/dt', 'd \alpha / dt'};

% disp('closed-loop poles:');
% eig(A_cl)

% Response to initial conditions

%Setting the initial conditions
x0 = [0  10*pi/180 0 0 ];

%Plotting the response of the closed-loop system to 
%the initial conditions
T_final = 4;
% res = initial(sys_cl,x0,T_final);
% initial(sys_cl,x0,T_final)
% grid


% res = step(sys_cl)
% 
% S = stepinfo(res);
% S(1,1)
% S(2,1)
% S(3,1)
% S(4,1)

%stepplot)

% [y,t,x] = initial(sys_cl,x0,T_final);
% figure
% plot(t,y)

% sys2 = ss(A_cl,B_cl,C_cl,0);
% step(sys2)

t = (0:0.001:20);
%u = ones(size(t));
%res = initial(sys_cl,u,t,x0)
%[y,t,x] = initial(sys_cl,x0,T_final)
figure;
initial(sys_cl,x0,T_final)
% [y,t,x] = step(sys_cl);
% stepinfo(y(:,1),t)
% stepinfo(y(:,2),t)
% stepinfo(y(:,3),t)
% stepinfo(y(:,4),t)
%S = stepinfo(res(:,2),t)

x_d = [0.1 0 0 0];

% disp('closed-loop poles:');
EV = eig(A_cl)

alpha = 2.2245;
beta = 1.4768;
eps = sin(atan(alpha/beta)) 





