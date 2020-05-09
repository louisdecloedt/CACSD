close all;
clear;

%Part 1: Model and open loop analysis

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
B = zeros(4,1);
B(3) = K_m*K_g / (R_m*J_h);
B(4) = - K_m*K_g / (R_m*J_h);

C = [1  0  0  0
     0  1  0  0];
 
D = [0
     0];


%TODO: Open loop analysis: Poles and zeros of the system, 
%System: stable, controllable, observable, stabilizable, detectable, 
%minimal?

%Eigenvalues A:
eig (A)
%geen positief reeel deel

%controllable?
% disp ('Controllability matrix');
CO = ctrb(A,B);
disp('Rank of the controllability matrix:');
% size(CO)
rank(CO)
%Of full rank thus controllable

%Observable
C = [1 0 0 0;
     0 1 0 0];
CA = C*A;
CA_2 = C*A*A;
CA_3 = C*A*A*A;

O = [C; CA; CA_2; CA_3]
rank(O)

%Transmission zeros
z = tzero(A,B,C,D)


































































































































































































