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
Aic = [ A zeros(12,3); C(1:3,:) eye(3)];
Bic = [B;D(1:3,:)];
Q = eye(15,15);
Q(1:3,1:3) = 2.5E1*Q(1:3,1:3);
Q(4:6,4:6) = 6E1*Q(4:6,4:6);
Q(3,3) = 1E2*Q(3,3);
Q(6,6) = 1E2*Q(6,6);
Q(13,13) = 2;
Q(14,14) = 2;
Q(15,15) = 1e2;
Igain = 19;

R = 1*eye(4);
[~,K,~] = idare(Aic,Bic,Q,R,[],[],[]);
K0 = K(1:4,1:12);
K1 = K(1:4,13:15);


% open("LQR_integral_control_quadcopter.slx");
sim("LQR_integral_control_quadcopter.slx",Tmax);
generate_report(0);