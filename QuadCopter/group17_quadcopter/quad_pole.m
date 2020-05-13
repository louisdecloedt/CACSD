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
[Ad,Bd,Cd,Dd] = ssdata(sysd);


%Where to place the poles of the estimator:
%Rules of thumb:
%For a fast response of the estimator, set poles 2 and 5 times
%faster than the desired closed-loop controller poles.
% error decays two to five times faster than the state vector x(t)
%Discrete time: same rules choose in s-domain and map with:
%z = e^(sT_s)


%Poles in continuous time:
%Closed loop (A - BK)
%Dominant poles: via second order interpretation dominant poles:
%damping ratio: 0.75, settle time 4 sec
p_cl = [-1.15+1.1014i 
        -1.15-1.1014i 
        -6 
        -6.5 
        -7 
        -7.5 
        -8 
        -8.5 
        -9 
        -9.5 
        -10 
        -10.5];
    
% p_cl = [-1.15+1.1014i 
%         -1.15-1.1014i 
%         -6 
%         -6.5 
%         -7 
%         -7.5 
%         -8 
%         -8.5 
%         -9 
%         -9.5 
%         -10 
%         -10.5];

%p_cl = p_cl -4;
    
% %Poles state estimator
% %Should go 2-5 times faster to zero than x(t)
% p_est = -4.2 + p_cl;
% 
% %transfer from s-domain to z-domain.
% p_d_cl = exp(Ts*p_cl);
% p_d_cl = [0.5545 + 0.0000i
%    0.9234 + 0.1013i
%    0.9234 - 0.1013i
%    0.9234 + 0.1013i
%    0.9234 - 0.1013i
%    0.8611 + 0.0000i
%    0.8611 + 0.0000i
%    0.9834 + 0.0148i
%    0.9834 - 0.0148i
%    0.9512 + 0.0000i
%    0.9512 + 0.0000i
%    0.9511 + 0.0000i];
% %p_d_est = exp(Ts*p_est);
% log(p_d_cl);
% p_d_est = 5*log(p_d_cl)
% p_d_est = exp(Ts*p_d_est)

%Calculate Nx, Nu
r = 6;
N = [A-eye(size(A)) B; C D]\[zeros(18-r,r); eye(r)];
Nx = N(1:12,:);
Nu = N(13:16,:);

% K_d = place(Ad,Bd,p_d_cl);
%size(K_d)

K_d = [ -0.0000   -0.6019   36.7676   -0.0000   -0.7960   38.7104    5.4550  -0.0000    0.4918    1.9686   -0.0000    1.6434
    0.6019    0.0000   36.7676    0.7960    0.0000   38.7104   -0.0000  5.4550   -0.4918   -0.0000    1.9686   -1.6434
   -0.0000    0.6019   36.7676   -0.0000    0.7960   38.7104   -5.4550  -0.0000    0.4918   -1.9686   -0.0000    1.6434
   -0.6019    0.0000   36.7676   -0.7960    0.0000   38.7104   -0.0000  -5.4550   -0.4918   -0.0000   -1.9686   -1.6434];

 

  

%Check if discrete poles are at the desired location.
%eig(Ad - Bd*K_d);

%You can also use place for estimator gain selection by transposing 
%the A matrix and substituting C' for B.

%L_d = place(Ad',Cd',p_d_est).'
%L_d = place(Ad',Cd',p_d_est)';
%size(L_d)
%size(L_d)

sim("AA_pole_placement.slx",Tmax);
generate_report(0);

