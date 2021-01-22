function [A,B,C,n] = defineABC(A_1,A_2,B_1,B_2)
n = size(A_1,1) +size(A_2,2);
m = size(B_1,2) + size(B_2,2);
A = [A_1 zeros(n/2,n/2);zeros(n/2,n/2) A_2];
B = [B_1, zeros(n/2,m/2);zeros(n/2,m/2), B_2];

%Set values under 10e-13 = 0


A((abs(A)<100*eps)) = 0;

C = zeros(m,n);
C(1,1) = 1; %u1-Speed
C(2,7)= 1; %phi1
C(3,8) = 1; %theta1
C(4,9) = 1; %height1
C(5,10) = 1; %u2-speed
C(6,16) = 1; %phi2
C(7,17) = 1; %theta2
C(8,18) = 1; %height2

end

