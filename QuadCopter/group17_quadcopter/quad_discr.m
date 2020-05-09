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

% initial condition
x0 = zeros(1,12);

% linear state space
u_e = 40.875*ones(4,1);
A = zeros(12,12);
B = zeros(12,4);
C = zeros(6,12);
D = zeros(6,4);

A(1:3,4:6) = eye(3);
A(4:6,4:6) = -kd/m *eye(3);
A(7:9,10:12) = eye(3);


A(4,8) = 40.875*k*cm/m;
A(5,7) = - 40.875*k*cm/m;


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

Add = inv(eye(size(A)) - (Ts/2)*A)*(eye(size(A)) + (Ts/2)*A)
Bdd = inv(eye(size(A)) - (Ts/2)*A)*(Ts)*B;
% Cdd = C*inv(eye(size(A)) - (Ts/2)*A);
% Ddd = D + C*inv(eye(size(A)) - (Ts/2)*A)*(Ts/2)*B;

[Ad,Bd,Cd,Dd] = bilinear(A,B,C,D,1/Ts);

%open_system("template_quadcopter2019.slx")

disp("Poles discrete system:")
eig(Ad)

%Check if (A,B) is controllable
CO_d = ctrb(Add,Bdd);
rank(CO_d)

% %Check if (A,C) is observable
% O_d = obsv(Ad,Cd);
% rank(O_d);
% % = 12: (A,C) is observable
% 
% %hautus_test = rank([eye(12)-Ad,Bd])
% 
% %Transmission zeros
% tzero(Ad,Bd,Cd,Dd)

sysc = ss(A,B,C,D);
sysd = c2d(sysc,Ts,"zoh");
[Addd,Bddd,Cddd,Dddd] = ssdata(sysd);
%eig(Addd)

rank(ctrb(Addd,Bddd))

B