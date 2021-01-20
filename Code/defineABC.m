function [A,B,C,n] = defineABC(A_1,A_2,B_1,B_2)
n = size(A_1,1) +size(A_2,2);
m = size(B_1,2) + size(B_2,2);
A = [A_1 zeros(n/2,n/2);zeros(n/2,n/2) A_2];
B = [B_1, zeros(n/2,m/2);zeros(n/2,m/2), B_2];

%Set values under 10e-13 = 0


A((abs(A)<100*eps)) = 0;

C = zeros(m,n);
C(1,1) = 1; %u-Spped
C(2,11)= 1;
C(3,2) = 1;
C(4,12) = 1;
C(5,10) = 1;%height
C(6,20) = 1;
C(7,9) = 1; %psi 1
C(8,19) = 1; %psi 2 
resort = zeros(8,8);
resort(1,1) = 1;
resort(2,3) = 1;
resort(3,7) = 1;
resort(4,5) = 1;
resort(5,2) = 1;
resort(6,4) = 1;
resort(7,8) = 1;
resort(8,6) = 1;
C = resort*C
end

