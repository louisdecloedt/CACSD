function [Ad,Bd,Cd,Dd] = ZOH(A,B,C,D,Ts)
Ad = exp(A*Ts);
Bd = A\(Ad-eye(size(Ad)))*B;
Cd = C;
Dd = D;
end

