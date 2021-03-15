function [A,B,C,n] = defineABC(A_1,A_2,B_1,B_2,C_1,C_2)
n = size(A_1,1) +size(A_2,2);
m = size(B_1,2) + size(B_2,2);
A = [A_1 zeros(n/2,n/2);zeros(n/2,n/2) A_2];
B = [B_1, zeros(n/2,m/2);zeros(n/2,m/2), B_2];

C = zeros(m,n);
C(1:4,1:(n/2)) = C_1;
C(5:8,(n/2+1):n) = C_2;

end

