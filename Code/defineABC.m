function [A,B,C,n] = defineABC(A_1,A_2,B_1,B_2)
n = size(A_1,1) +size(A_2,2);
m = size(B_1,2) + size(B_2,2);
A = [A_1 zeros(n/2,n/2);zeros(n/2,n/2) A_2];
B = [B_1, zeros(n/2,m/2);zeros(n/2,m/2), B_2];

A((abs(A)<1e-9)) = 0;

C = zeros(m,n);
C(1,1) = 1; %vA1
C(2,3) = 1; %beta1
C(3,7)= 1; %phi1
C(4,9) = 1; %height1
C(5,10) = 1; %vA2
C(6,12) = 1; %beta2
C(7,16) = 1; %phi2
C(8,18) = 1; %height2

end

