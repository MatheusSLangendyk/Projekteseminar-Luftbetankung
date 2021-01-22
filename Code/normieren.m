function [A,B,C] = normieren(A,B,C,eta_max,sigmaf_max,xi_max,zita_max)
p = size(B,2);
n = size(A,2);
q = size (C,1);
Tu = zeros(p,p); %Input transformation matrix
Tx = eye(n,n); %State Transformation Matrix
Ty = zeros(q,q);
Tu(1,1) = eta_max;
Tu(2,2) = sigmaf_max;
Tu(3,3) = xi_max;
Tu(4,4) = zita_max;
Tu(5,5) = eta_max;
Tu(6,6) = sigmaf_max;
Tu(7,7) = xi_max;
Tu(8,8) = zita_max;
Ty(1,1) = 250;
Ty(2,2) = 45*pi/180;
Ty(3,3) = 45*pi/180;
Ty(4,4) = 10000;
Ty(5,5) = 250;
Ty(6,6) = 45*pi/180;
Ty(7,7) = 45*pi/180;
Ty(8,8) = 10000;

A = Tx\A*Tx;
B = Tx\B*Tu;
C = (Ty\C*Tx);
end

