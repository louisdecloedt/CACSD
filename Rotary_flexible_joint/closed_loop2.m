clear;
close all;

%first closed Loop analysis
%based on a.o. ip_example.m

%Constants:
R_m = 2.6; % ohm
K_m = 0.00767; % N m / A
K_b = 0.00767; % V / rad/s
K_g = 70; %70:1 (The ouput is slower)
J_l = 0.0059; %kg m^2
J_h = 0.0021; %kg m^2
d = 0.0318; %m
R = 0.076; %m
r = 0.0318; %m
K_stiff = 1.60856; %N/m
F_r = 1.33; %N
L = 0.0318; %m


%Initialize linear system:
A = zeros(4,4);
A(1,3) = 1;
A(2,4) = 1;
A(3,2) = K_stiff/J_h;
%TODO: K?2_g == squared??
A(3,3) = - K_g*K_g*K_m*K_b / (J_h * R_m);
A(4,2) = - (J_l + J_h)*K_stiff / (J_l*J_h);
A(4,3) =  K_g*K_g*K_m*K_b / (J_h * R_m);

%single input V (u)
B = [0
    0
    0
    0];
B(3) = K_m*K_g / (R_m*J_h);
B(4) = - K_m*K_g / (R_m*J_h);

C = [1  0  0  0
     0  1  0  0];
% sys_cl.OutputName -- check this lines further
% C = [1  0  0  0
%      0  1  0  0
%      0 0 1 0
%      0 0 0 1];
 
% D = [0
%      0];
%D = 0;
D = [0
    0];
% 



sys = ss (A,B,C,D);
 

% Checking Stability
disp('Poles:')
eig (A);

%plotting the location of the poles
figure
pzmap(sys);


%LQR controllor
%initial values;
Q0 = [350 0 0 0
      0 1500 0 0
      0   0 3 0
      0  0  0 0.5];
R0 = 10;

%Determine K
%[K,S,e] = lqr(SYS,Q,R,N)
%https://nl.mathworks.com/help/control/ref/lqr.html
%N = optional, default zero (N = 0 is what we need)
[K,S,e] = lqr(sys,Q0,R0);

size(K)
!K = K(1,:)





% SETTING UP NEW SYSTEM
% Defining the closed-loop system
% x_dot = (A - B*K)*x
% y = c*x
A_cl = A - B*K;
B_cl = [0;0;0;0];
C_cl = C;
D_cl = D;
sys_cl = ss(A_cl,B_cl,C_cl, D_cl);
sys_cl.OutputName={'z','\theta'};
%sys_cl.OutputName={'\theta','\alpha','d\theta/dt','d\alpha/dt'};

disp('closed-loop poles:');
eig(A_cl)

% disp('closed-loop poles (e):');
% e

%Note to myself
%K is always the same - since it is STATE feedback control
%Thus doesnt matter which C is used




%SIMULATION
% Response to initial conditions
%Setting the initial conditions
x0 = [0 0 0 0];

%Plotting the response of the closed-loop system to 
%the initial conditions
figure
initial(sys_cl,x0)
grid

theta_desired = pi/4; %has to be in [-PI/2,PI/2]
x_d = [theta_desired 0 0 0];

