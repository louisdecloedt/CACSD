clear;
close all;

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
Q = eye(size(C,2));
Q(3,3) = 10000;
Q(6,6) = 10000;
R = eye(size(B,2));
[~,K,~] = idare(A,B,Q,R,[],[],[]);

%Reference K out of working LQR controller
K_d = K;
p_cl_z = eig(A-B*K_d)
%p_cl_ref_s =  log(p_cl_ref_z)./Ts;

%Try to reconstruct K from poles:
%K_d = place(A,B,p_cl_z');

%Attempt via sylvester equation.
%Matlab: Solve Sylvester equation AX + XB = C for X
%X = sylvester(A,B,C)

lambda = diag(p_cl_z);
G = randn(4,12);

% X = sylvester(sysd.A,-lambda,sysd.B*G)
% K_d = G*inv(X);

test = eig(A - B*K_d);
% determine N
r = 6;
N = [A-eye(size(A)) B; C D]\[zeros(18-r,r); eye(r)];
Nx = N(1:12,:);
Nu = N(13:16,:);



sim("AA_pole_placement.slx",Tmax);
generate_report(0);
