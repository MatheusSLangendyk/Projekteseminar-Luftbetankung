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
F0 = zeros(l,1); %List of errors from 1 to l
M_real_possibilities = zeros(n+p,l); %List of all real M to different core dimension
for i = 1:n
    if i<=m
        H = [ew(i)*eye(n,n)-A, -B;C2_tilde, zeros(l,p)];
        assignin('base','H',H)
        M = null(H);
        for j =1:l %Try every possible dimension of the core and pich the one with the smaller error f0
            M_j = real(M(:,j));
            [M_real, f0] = fminsearch('cost_core_deviance',real(M_j)+imag(M_j)/100,optimset('TolX',1e-10,'MaxFunEvals',1000000,'MaxIter',1000000));
            F0(j,1) = f0;
            M_real_possibilities(:,j) = M_real;
        end
        % Pick the M_real with the smaller error
        [~,opt_dimension] = min(F0);
        M_real = M_real_possibilities(:,opt_dimension);
       % [M_real, f0] = fminsearch('cost_core_deviance',real(M(:,1)),optimset('TolX',1e-10,'MaxFunEvals',1000000,'MaxIter',1000000));
        v_i = M_real(1:n,:);
        p_i = M_real(n+1:end,:);
        V(:,i) = v_i;
%        
        if rank(V)<i
            % ausgangsseitige Verkopplungsbedingung kann nicht weiter
            % angewandt werden
            m = i-1;
            V(:,i)=((ew(i)*eye(n)-A)^-1*B)*P(:,i); %Freiheitsgrad
        else
            P(:,i) = p_i;
        end
    else
        V(:,i)=((ew(i)*eye(n)-A)^-1*B)*(P(:,i)-i); %Freiheitsgrad
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
