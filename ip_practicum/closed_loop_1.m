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


