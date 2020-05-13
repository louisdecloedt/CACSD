clear all;
close all


% Parameters
Ts = 0.05;
m = 0.5;
L = 0.25;
k = 3E-6;
b = 1E-7;
g = 9.81;
kd = 0.25;
Ixx = 5E-3;
Iyy = 5E-3;
Izz = 1E-2;
cm = 1E4;

% Reference
load references_17

angle_ref = zeros(3,1);

% linear state space
u_e = g*m/(cm*k*4)*ones(4,1); %40.875
A = zeros(12,12);
B = zeros(12,4);
C = zeros(6,12);
D = zeros(6,4);

A(1:3,4:6) = eye(3);
A(4:6,4:6) = -kd/m *eye(3);
A(7:9,10:12) = eye(3);
A(4,8) = g;
A(5,7) = -g;

B(6,1:4) = k*cm/m;
B(10,1) = L*k*cm/Ixx;
B(10,3) = -B(10,1);
B(11,2) = L*k*cm/Iyy;
B(11,4) = -B(11,2);
B(12,1:4) = b*cm/Izz;
B(12,2) = -B(12,2);
B(12,4) = -B(12,4);

C(1:3,1:3) = eye(3);
C(4:6,7:9) = eye(3);

% discrete transformation
sysc = ss(A,B,C,D);
sysd = c2d(sysc,Ts,"zoh");
[A,B,C,D] = ssdata(sysd);

% solve ricatti equation
Aic = [eye(size(C,1)) C; zeros(12,6) A];
Bic = [D;B];
Q = eye(18);
R = eye(4);
[~,K,~,info] = idare(Aic,Bic,Q,R);
K1 = K(1:4,1:6);
K1 = 20*K1;
K1(1:4,3) = 1.5*K1(1:4,3);
K1(1:4,6) = 2*K1(1:4,6);
K0 = 1.4*K(1:4,7:18);

% covariance matrices
Wxyz = 2.5E-5*eye(3);
Wptp = 7.57E-5*eye(3);
W = eye(12);
W(1:3,1:3) = Wxyz;
W(7:9,7:9) = Wptp;
V = [Wxyz zeros(3);zeros(3) Wptp];
Qw = W*transpose(W);
Rv = V*transpose(V);

% rank(obsv(A,C))
% [~,flag] = chol([Qw zeros(12,6);zeros(6,12) Rv])

open("LQG_control_quadcopter.slx");
sim("LQG_control_quadcopter.slx",Tmax);
generate_report(1);