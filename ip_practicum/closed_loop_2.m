close all;
clear;

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

C = [1 0 0 0
     0 1 0 0];
 
D = [0
     0];

Q0 = [2 0 0 0
      0 3 0 0
      0 0 0 0
      0 0 0 0];
R0 = 0.005;

sys = ss (A,B,C,D);
[K,S,e] = lqr(sys,Q0,R0);
K = K(1,:);
K;

x_d = [0.1 0 0 0];

%Setting the initial conditions
x0 = [0  10*pi/180 0 0 ];
 
%Sampling Period:
T_s = 0.005; %s
%
%W_c = 2; %Hz
f = 2;
W_c = 2*f*pi; %rad/s

%Digital part
A2 = zeros(4,4);
A2(1,2) = 1;
A2(2,2) = 1/(1 + W_c*T_s);
A2(3,4) = 1;
A2(4,4) = 1/(1 + W_c*T_s);

B2 = zeros(4,2);
B2(2,1) = W_c*T_s/(1 + W_c*T_s);
B2(4,2) = W_c*T_s/(1 + W_c*T_s);

C2 = zeros(2,4);
C2(1,1) = - 1/T_s;
C2(1,2) =   1/T_s;
C2(2,3) = - 1/T_s;
C2(2,4) =   1/T_s;

D2 = zeros(2,2);


 
 
 
