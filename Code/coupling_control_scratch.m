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
k = 1;

if ctb1+ctb2~=2
    error('System ist nicht steuerbar!')
end
% ausgangsseitige Verkopplungsbedingung für i<m
% m vorab noch nicht bekannt
m = n;
% Matrix der Eigenvektoren
V = zeros(n,n);
P = ones(p,n);
ew_previous = 1;
for i = 1:n
    
    if ew(i) ~= conj(ew_previous)
        
        ew_previous = ew(i);
        if i<=m 
            %EW(i) = ew(i);
            %Ausgangsseitigeverkopplungsbedingung
            H = [ew(i)*eye(n,n)-A, -B;C2_tilde, zeros(l,p)];
            M = null(H);
            v_i = M(1:n,k);
            p_i = M(n+1:end,k);
            V(:,i) = v_i;
            if imag(ew(i)) ~= 0
                %Ausgangsseitigeverkopplungsbedingung wenn ew(i) imaginär
                H_conj = [conj(ew(i))*eye(n,n)-A, -B;C2_tilde, zeros(l,p)];
                M_conj = null(H_conj);
                v_i_conj = M_conj(1:n,k);
%                 q = ([eye(n), zeros(n,p)]*M)\v_i_conj;
%                 p_i_tilde = [zeros(p,n), eye(p)]*M*q;
                p_i_tilde = conj(p_i);
                V(:,i+1) = v_i_conj;
                
            end
    %        
            if rank(V)<i
                % ausgangsseitige Verkopplungsbedingung kann nicht weiter
                % angewandt werden
                m = i-1;
%                 H_red = [ew(i)*eye(n,n)-A, -B];
%                 M_red = null(H_red);
%                 v_i = M_red(1:n,k);
%                 p_i = M_red(n+1:end,k);
%                 V(:,i) = v_i;
%                 P(:,i) = p_i;
%                 if imag(ew(i)) ~= 0
%                     %Eingangseitigebeverkopplungsbedingung (Übergang) für komplexen ew
%                     H_red_conj = [conj(ew(i))*eye(n,n)-A, -B];
%                     M_red_conj = null(H_red_conj);
%                     v_i_red_conj = M_red_conj(1:n,k);
% %                     q = ([eye(n), zeros(n,p)]*M_red_conj)\v_i_red_conj;
% %                     p_i_tilde = [zeros(p,n), eye(p)]*M*q;
%                     p_i_tilde = conj(p_i);
%                     V(:,i+1) = v_i_red_conj;
%                     P(:,i+1) = p_i_tilde;
%                 end
                P(:,i) = zeros(8,1);
                P(1,i) = 1;
                P(end,i) = 1;
                V(:,i) = (ew(i)*eye(n)-A)\B*P(:,i);
                
            else
                
                P(:,i) = p_i;
                if imag(ew(i)) ~= 0
                    P(:,i+1) = p_i_tilde;
                end
            end
        else
             %Eingsngsseitigeverkopplungsbedingung
%              H_red = [ew(i)*eye(n,n)-A, -B];
%              M_red = null(H_red);
%              v_i = M_red(1:n,k);
%              p_i = M_red(n+1:end,k);
%              V(:,i) = v_i;
%              P(:,i) = p_i;
%                 if imag(ew(i)) ~= 0
%                     %Eingangseitigebeverkopplungsbedingung (Übergang) für komplexen ew
%                     H_red_conj = [conj(ew(i))*eye(n,n)-A, -B];
%                     M_red_conj = null(H_red_conj);
%                     v_i_red_conj = M_red_conj(1:n,k);
%                     q_red = ([eye(n), zeros(n,p)]*M_red_conj)\v_i_red_conj;
%                     p_i_tilde = [zeros(p,n), eye(p)]*M_red_conj*q_red;
%                     p_i_tilde = conj(p_i);
%                     V(:,i+1) = v_i_red_conj;
%                     P(:,i+1) = p_i_tilde;
%                 end
                   V(:,i) = (ew(i)*eye(n)-A)\B*P(:,i);
        end
    end
end

% Regler Matrix R berechenenrank(V)
det(V)
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
% F = real(F);
