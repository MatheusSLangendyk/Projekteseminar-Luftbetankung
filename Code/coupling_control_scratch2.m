function [R, F] = coupling_control_scratch2(sys,C_tilde,ew,l,P)
% [R, F] = verkopplung(sys,Cvk,ew,P)
%  berechnet eine Zustandsrückführung u=-Rx+Fw für das System sys, welche die
%  Eigenwerte ew im geschlossenen Regelkreis erzeugt und das System
%  bezüglich des Verkopplungsausgangs 0=Cvk*x verkoppelt. 
%  bei der ausgangsseitigen Verkopplungsbedingung nicht berücksichtigt.
%  F wird für stationäre Genauigkeit ausgelegt, sofern möglich.
% Vorlesung "Mehrgroessenreglerentwurf im Zustandsraum"
% Institut fuer Automatisierungstechnik
% TU Darmstadt
%  ew(6) = real(ew(9));
%  ew(7) = real(ew(10));
ctb1 = steuerbarKalman(sys);
ctb2 = steuerbarHautus(sys);
A = sys.A;
B = sys.B;
C = sys.C;
C1_tilde = C_tilde(1:l,:);
C2_tilde = C_tilde(l+1:end,1:end);
% Systemordnung
n = size(A,1);
% Eingangsdimension
p = size(B,2);
k = 1; %Dimension of the core 
if ctb1+ctb2~=2
    error('System ist nicht steuerbar!')
end
% ausgangsseitige Verkopplungsbedingung für i<m
% m vorab noch nicht bekannt
m = n;
% Matrix der Eigenvektoren
V = zeros(n,n);

for i = 1:n
        if i<=m 
            
            %Ausgangsseitigeverkopplungsbedingung
            H = C2_tilde/(ew(i)*eye(n)-A)*B;
            M = null(H);
            p_i = M(:,k);   
            V(:,i) = (ew(i)*eye(n)-A)\B*p_i;
           if rank(V,10^-4)<i
          
                % ausgangsseitige Verkopplungsbedingung 
                
                m = i-1;
                V(:,i) = (ew(i)*eye(n)-A)\B*P(:,i); 
               
            else
               P(:,i) = p_i; %Ausgangsseitigeverkopplungsbedingung bestätigen
            end
            
        else
             %Eingsngsseitigeverkopplungsbedingung
             
                V(:,i) = (ew(i)*eye(n)-A)\B*P(:,i); 
              
             if rank(V,10^-9) < i 
                 %If V does not have full rank, place vector v_i
                 %orthogonally to v_(i-1)
                 H = [ew(i)*eye(n,n)-A, -B;V(:,i-1)' zeros(1,p)];
                 M = null(H);
                 V(:,i) = M(1:n,k);
                 P(:,i) = M(n+1:end,k);  
                 
             end
             
        end
    
end
det(V)
W = inv(V);
Wr1 =  W(1:m,:);
%Wr2 = W(m+1:end,:);
R = -P*V^-1;
R = real(R);
disp(eig(A-B*R))

% Vorfilterentwurf
FM = randn(m,p-l);
F1 = (W*B)\[FM;zeros(n-m,p-l)];

    % stationäre Genauigkeit nur bei quadratischem System

Q = inv(C1_tilde*inv(B*R-A)*B*F1);
F = -inv(C*((A-B*R)\B));

 F1_tilde = F1*Q;
 F(:,1:p-l) = F1_tilde;
F = real(F);


