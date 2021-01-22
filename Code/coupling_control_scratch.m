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
ew_previous = -1;
for i = 1:n
    
    if ew(i) ~= conj(ew_previous)
        ew_previous = ew(i);
        if i<=m 
            H = [ew(i)*eye(n,n)-A, -B;C2_tilde, zeros(l,p)];
            M = null(H);
            v_i = M(1:n,1);
            p_i = M(n+1:end,1);
            if imag(ew(i))==0
                V(:,i) = v_i;
            else
                H_conj = [conj(ew(i))*eye(n,n)-A, -B;C2_tilde, zeros(l,p)];
                M_conj = null(H_conj);
                v_i_conj = M_conj(1:n,1);
                q = ([eye(n), zeros(n,p)]*M)\v_i_conj;
                p_i_tilde = [zeros(p,n), eye(p)]*M*q;
                p_i_tilde = conj(p_i);
                V(:,i) = v_i;
                V(:,i+1) = v_i_conj;
                
            end
    %        
            if rank(V)<i
                % ausgangsseitige Verkopplungsbedingung kann nicht weiter
                % angewandt werden
                m = i-1;
                H_red = [ew(i)*eye(n,n)-A, -B];
                M_red = null(H_red);
                V(:,i)=((ew(i)*eye(n)-A)^-1*B)*P(:,i); %Freiheitsgrad
                V(:,i+1)=((conj(ew(i))*eye(n)-A)^-1*B)*P(:,i+1); %Freiheitsgrad
            else
                
                P(:,i) = p_i;
                if imag(ew(i)) ~= 0
                    P(:,i+1) = p_i_tilde;
                end
            end
        else
            V(:,i)=((ew(i)*eye(n)-A)^-1*B)*(P(:,i)); %Freiheitsgrad
            V(:,i+1)=((conj(ew(i))*eye(n)-A)^-1*B)*P(:,i+1); %Freiheitsgrad

        end
    end
end

% Regler Matrix R berechenenrank(V)
R = -P*V^-1;
R = real(R)
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
