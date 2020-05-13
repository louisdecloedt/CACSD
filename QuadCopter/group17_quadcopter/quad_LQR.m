clear;
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

% initial condition
x0 = zeros(12,1);

% Reference
x = 0;
y = 0;
z = 1;
% x = 5;
% y = -1;
% z = 2;
references = zeros(1,7);
size(references)
references(1,1) = 1; % 1sec delay
references(1,2:4) = [x y z];

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
[A,B,C,D] = bilinear(A,B,C,D,1/Ts);

sysc = ss(A,B,C,D);
sysd = c2d(sysc,Ts,"zoh");
[Ad,Bd,Cd,Dd] = ssdata(sysd);

% solve ricatti equation
Q = eye(size(C,2));
R = eye(size(B,2));
[~,K,~] = icare(A,B,Q,R,[],[],[]);

% determine N
r = 6;
N = [A-eye(size(A)) B; C D]\[zeros(18-r,r); eye(r)];
Nx = N(1:12,:);
Nu = N(13:16,:);






