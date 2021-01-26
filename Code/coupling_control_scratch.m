function [R, F] = coupling_control_scratch(sys,C_tilde,ew,l,P)
% [R, F] = verkopplung(sys,Cvk,ew,P)
%  berechnet eine Zustandsrückführung u=-Rx+Fw für das System sys, welche die
%  Eigenwerte ew im geschlossenen Regelkreis erzeugt und das System
%  bezüglich des Verkopplungsausgangs 0=Cvk*x verkoppelt. 
%  bei der ausgangsseitigen Verkopplungsbedingung nicht berücksichtigt.
%  F wird für stationäre Genauigkeit ausgelegt, sofern möglich.
% Vorlesung "Mehrgroessenreglerentwurf im Zustandsraum"
% Institut fuer Automatisierungstechnik
% TU Darmstadt

ctb1 = steuerbarKalman(sys);
ctb2 = steuerbarHautus(sys);
A = sys.A;
B = sys.B;
C1_tilde = C_tilde(1:l,:);
C2_tilde = C_tilde(l+1:end,1:end);
% Systemordnung
n = size(A,1);
% Eingangsdimension
p = size(B,2);
k = 4; %Dimension of the core 
if ctb1+ctb2~=2
    error('System ist nicht steuerbar!')
end
% ausgangsseitige Verkopplungsbedingung für i<m
% m vorab noch nicht bekannt
m = n;
% Matrix der Eigenvektoren
V = zeros(n,n);
%P = ones(p,n);

for i = 1:n
        if i<=m 
            %EW(i) = ew(i);
            %Ausgangsseitigeverkopplungsbedingung
            H = [ew(i)*eye(n,n)-A, -B;C2_tilde, zeros(l,p)];
            M = null(H);
            V(:,i) = M(1:n,k);
            p_i = M(n+1:end,k);   
            if rank(V,10^-9)<i
                % ausgangsseitige Verkopplungsbedingung kann nicht weiter
                % angewandt werden
                m = i-1;
%                 H_red = [ew(i)*eye(n,n)-A, -B];
%                 M_red = null(H_red);
%                 v_i = M_red(1:n,k);
%                 p_i = M_red(n+1:end,k);
%                 V(:,i) = v_i;
%                 P(:,i) = p_i;
                V(:,i) = (ew(i)*eye(n)-A)\B*P(:,i);
            else
               P(:,i) = p_i; 
            end
            
        else
            
             %Eingsngsseitigeverkopplungsbedingung
%              H_red = [ew(i)*eye(n,n)-A, -B];
%              M_red = null(H_red);
%              v_i = M_red(1:n,k);
%              p_i = M_red(n+1:end,k);
%              V(:,i) = v_i;
%              P(:,i) = p_i;
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

R = -P*V^-1;
R = real(R);
disp(eig(A-B*R))
% Vorfilterentwurf
FM = null([B V(:,1:m)]);
F1 = FM(1:p,:);
F = inv(C_tilde*((B*R - A)\B));
Q_tilde = C1_tilde*((B*R - A)\B*F1);
F1_tilde = F1/Q_tilde;
F(:,1:p-l) = F1_tilde;
F = real(F);