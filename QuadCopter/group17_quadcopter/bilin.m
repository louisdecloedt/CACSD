function [Ad,Bd,Cd,Dd] = bilin(A,B,C,D,Ts)

AA = inv(eye(size(A))-A*Ts/2);
Ad = AA*(eye(size(A))+A*Ts/2);
Bd = AA*B*Ts;
Cd = C*AA;
Dd = D + Cd*B*Ts/2;
end

