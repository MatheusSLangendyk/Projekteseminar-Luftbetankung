function [R, F] = coupling_control_scratch(sys,C_tilde,ew,l)
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


if ctb1+ctb2~=2
    error('System ist nicht steuerbar!')
end
% ausgangsseitige Verkopplungsbedingung für i<m
% m vorab noch nicht bekannt
m = n;
% Matrix der Eigenvektoren
V = zeros(n,n);
P = ones(p,n);
% P(:,18) = zeros(p,1)
% P(1,18) = 0.5
% P(8,18) = 0.7

for i = 1:n
    if i<=m
        M = null([ew(i)*eye(n,n)-A, -B;C2_tilde, zeros(l,p)]);
        v_i = M(1:n,:);
        p_i = M(n+1:end,:);
        q = M\[conj(v_i);conj(p_i)];
        M*
       % M = M(:,1);
%         V(:,i) = M(1:n,:);
%         par = M(n+1:end,:);
        if rank(V)<i
            % ausgangsseitige Verkopplungsbedingung kann nicht weiter
            % angewandt werden
            m = i-1;
            V(:,i)=((ew(i)*eye(n)-A)^-1*B)*P(:,i); %Freiheitsgrad
        else
            P(:,i) = par;
        end
    else
        V(:,i)=((ew(i)*eye(n)-A)^-1*B)*(P(:,i)-i); %Freiheitsgrad
%         V1 = V(:,1);
%         V_g = [V1';V(:,i-1)';V(:,i-2)';V(:,i-3)'];
%         M = null([ew(i)*eye(n,n)-A, -B;V_g, zeros(4,p)]);
%         M = M(:,4);
%         V(:,i) = M(1:n,:);
%         P(:,i) = M(n+1:end,:);
    end
end

% Regler Matrix R berechenen
rank(V)
R = -P*V^-1;
% R = real(R);
disp(eig(A-B*R))
% disp(ew-eig(A-B*R))
% Vorfilterentwurf
FM = null([B V(:,1:m)]);
F1 = FM(1:p,:);
F = inv(C_tilde*((B*R - A)\B));
Q_tilde = C1_tilde*((B*R - A)\B*F1);
F1_tilde = F1/Q_tilde;
F(:,1:p-l) = F1_tilde;
% F = real(F);
