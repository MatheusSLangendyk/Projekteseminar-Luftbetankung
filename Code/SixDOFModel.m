%Simualtion of a complete 6 Degrees of Freedom Model
clc
clear
close all

system_norm = false;
deltah_offset = 10;

%% Get Model Parameters 
[globalParameters,m,g,he,I_inv] = initializeParameters();

%% Initial Values both planes
vA_init_1 = 150;
alpha_init_1 = 0;
beta_init_1 = 0;
Omega_init_1 = [0;0;0];
Phi_init_1 = [0;0;0];
h_init_1 = 5000;
P_e_init_1 = [0;0;-h_init_1];
%latlon_init = [40.712776;-74.005974]; %New York
latlon_init = [0;0];
X_init_1 = [vA_init_1;alpha_init_1;beta_init_1;Omega_init_1;Phi_init_1;h_init_1];

%plane 2
vA_init_2 = 150;
alpha_init_2 = 0;
beta_init_2 = 0;
Omega_init_2 = [0;0;0];
Phi_init_2 = [0;0;0];
h_init_2 = h_init_1 + deltah_offset;
P_e_init_2 = [0;0;-h_init_2];
X_init_2 = [vA_init_2;alpha_init_2;beta_init_2;Omega_init_2;Phi_init_2;h_init_2];
%% calculate trim points
ap_solver = 0;
if ap_solver == 0
    % AP mit fsolve
    [X_ap_1, U_ap_1] = fsolve_trim([X_init_1;zeros(4,1)], 1); % mit [vA, phi, psi, h] = [150, 0, 0, 5000]
    [X_ap_2, U_ap_2] = fsolve_trim([X_init_2;zeros(4,1)], 2); % mit [vA, phi, psi, h] = [150, 0, 0, 5010]
elseif ap_solver == 1
    % AP mit trimValues
    [X_ap_1,U_ap_1,f0_1] = trimValues(vA_init_1,alpha_init_1,beta_init_1,Omega_init_1,Phi_init_1,h_init_1,1);
    [X_ap_2,U_ap_2,f0_2] = trimValues(vA_init_2,alpha_init_2,beta_init_2,Omega_init_2,Phi_init_2,h_init_2,2);
end
% x_ap_comp = [[X_ap_1;U_ap_1] [X_ap_1_t;U_ap_1_t]];
X_ap = [X_ap_1;X_ap_2];
U_ap = [U_ap_1;U_ap_2];
U_ap((abs(U_ap)<1e-9)) = 0;
X_ap((abs(X_ap)<1e-9)) = 0;
X_ap_simulink = [X_ap(1:8);X_ap(10:18);X_ap(20)];

%% DGL Flugzeug 1
% symbolische nicht-lineare DGL mit psi
plane_selector = 1;
assignin('base','plane_selector',plane_selector)
symbolic_equations;
% Zustands-DGL ohne psi
f = [dvA;dalpha;dbeta;dp;dq;dr;dphi;dtheta;dh];

% Ausgangsgleichung
h = [vA beta phi h];

% 1. Linearisierung durch bilden der Jacoby-Matrizen
A_sym = jacobian(f, x_red_9);   % A = d f(x,u) / dx
B_sym = jacobian(f, u_stell);   % B = d f(x,u) / du
C_sym = jacobian(h, x_red_9);   % C = d y(x,u) / dx
D_sym = jacobian(h, u_stell);   % D = d y(x,u) / du

% 2. Einsetzten der Anfangswerte
A1 = double(subs(A_sym, [x10,u_stell], [X_ap_1; U_ap_1]'));
B1 = double(subs(B_sym, [x10,u_stell], [X_ap_1; U_ap_1]'));
C1 = double(subs(C_sym, [x10,u_stell], [X_ap_1; U_ap_1]'));
D1 = double(subs(D_sym, [x10,u_stell], [X_ap_1; U_ap_1]'));

%% DGL Flugzeug 2
% symbolische nicht-lineare DGL mit psi
plane_selector = 2;
assignin('base','plane_selector',plane_selector)
symbolic_equations;
% Zustands-DGL ohne psi
f = [dvA;dalpha;dbeta;dp;dq;dr;dphi;dtheta;dh];

% Ausgangsgleichung
h = [vA beta phi h];

% 1. Linearisierung durch bilden der Jacoby-Matrizen
A_sym = jacobian(f, x_red_9);   % A = d f(x,u) / dx
B_sym = jacobian(f, u_stell);   % B = d f(x,u) / du
C_sym = jacobian(h, x_red_9);   % C = d y(x,u) / dx
D_sym = jacobian(h, u_stell);   % D = d y(x,u) / du

A2 = double(subs(A_sym, [x10,u_stell], [X_ap_2; U_ap_2]'));
B2 = double(subs(B_sym, [x10,u_stell], [X_ap_2; U_ap_2]'));
C2 = double(subs(C_sym, [x10,u_stell], [X_ap_2; U_ap_2]'));
D2 = double(subs(D_sym, [x10,u_stell], [X_ap_2; U_ap_2]'));

%% System ein Flugzeug
% Steuerbarkeit des einzelnen Flugzeugs
rank(ctrb(A1,B1));
% Beobachtbarkeit des einzelnen Flugzeugs
rank(obsv(A1,C1));

sys1 = ss(A1, B1, C1, D1);
K = lqr(sys1, eye(9), eye(4));
sys1_cl = ss(A1-B1*K, B1, C1, D1);

anz_io = size(C1,1);
delta_k = zeros(1, anz_io);

% Berechnung der Differenzordnungen der Ausgänge
% Nutzen Sie dabei eine for-Schleife, um alle Ausgänge zu durchlaufen
% und eine while-Schleife zur Berechnung der jeweiligen Matrizen ci*(A^j)*B
for k = 1:anz_io     % alle Ausgänge nacheinander
  j=1;
  while C1(k,:)*A1^(j-1)*B1 == zeros(1,anz_io)
    j=j+1;
  end
  delta_k(k)=j;      % Differenzordnung des Ausgangs k
end

% Differenzordnung des Gesamtsystems
delta = sum(delta_k);

%% Zwei Flugzeug Modell
[A,B,C,n] = defineABC(A1,A2,B1,B2);

% Steuerbarkeit
eigenvalues = eig(A);
%Hautus
for i = 1:n
    eig_i = eigenvalues(i);
    if rank([eig_i*eye(n,n)-A, B]) ~=n
        disp(['Eigenvalue ',num2str(eig_i),' of Gesamt-System is not controllable ']);
    end
end
[ctb, ewS, ewNS] = steuerbarHautus(ss(A,B,C,0));

% Beobachtbarkeit 
eigenvalues = eig(A);
% Hautus
for i = 1:n
    eig_i = eigenvalues(i);
    if rank([eig_i*eye(n,n)-A;C]) ~=n
        disp(['Eigenvalue ',num2str(eig_i),' of Gesamt-System is not obsarvable ']);
    end
end
[obs, ewS, ewNS] = steuerbarHautus(ss(A',C',B',0));

sys_2plane = ss(A, B, C, zeros(8,8));

anz_io = size(C,1);
delta_k = zeros(1, anz_io);

% Berechnung der Differenzordnungen der Ausgänge
% Nutzen Sie dabei eine for-Schleife, um alle Ausgänge zu durchlaufen
% und eine while-Schleife zur Berechnung der jeweiligen Matrizen ci*(A^j)*B
for k = 1:anz_io     % alle Ausgänge nacheinander
  j=1;
  while C(k,:)*A^(j-1)*B == zeros(1,anz_io)
    j=j+1;
  end
  delta_k(k)=j;      % Differenzordnung des Ausgangs k
end

% Differenzordnung des Gesamtsystems
delta = sum(delta_k);

%Saturations
eta_max = 10*pi/180; %Elevator
eta_min = - 25*pi/180; 
sigmaf_max = 20*pi/180; %Throttl
sigmaf_min = 0.5*pi/180;
xi_max = 25*pi/180; %Airlon
xi_min = - xi_max;
zita_max = 30*pi/180; %Rudder
zita_min = - zita_max;

%% Normieren
 if system_norm == true
    [A,B,C] = normieren(A,B,C,eta_max,sigmaf_max,xi_max,zita_max);
 end
  
%   %% Transfer Function Open Loop
%    sys_ol = ss(A,B, C,zeros(8,8));
%   
%   %%Riccatti
%   Q = eye(n,n);
%   Q(17,17) = 100000; 
%   Q(9,9) = 1; % Bestrafung Höhe
%   Q(18,18) = 1;
%   Q(3,3) = 100; %Bestrafung Geschw. z-Komoponente
%   Q(13,13) = 100;
%   
%   R = 1000*eye(8,8);
%   R(2,2) = 4000;
%   R(5,5) = 2000;
%   R(6,6) = 9000;
%   K = lqr(sys_ol,Q,R);
%   K((abs(K)<10^-9)) = 0;
%   Ak = A -B*K;
%   F = -inv(C*(Ak\B));
%   ew_ricati = eig(Ak);
%   sys_ricati = ss(Ak,B*F,C,zeros(8,8));
%   %step(sys_ricati)
% %   Gamma = getgamma(A,B,C);
% %   gamma_sum = sum(Gamma);
% %   zero(sys_ol)
%   
%   %% Coupling Control (manual) 
%   l = 4; %coupling conditions
%   C_tilde = zeros(size(C,1), size(A,1));
%   C_tilde(1:4,:) = C(1:4,:);
%   C_tilde(5:8,:) = C(1:4,:) - C(5:8,:);
% 
%   C1_tilde = C_tilde(1:l,:);
%   C2_tilde = C_tilde(l+1:end,1:end);
%   struct_cond.sys_ol = sys_ol ;
%   struct_cond.C_tilde = C_tilde;
%   struct_cond.ew_ricati = ew_ricati;
%   struct_cond.l =l;
%   assignin('base','struct_cond',struct_cond)
%   P = ones(8,n);
%   %P = fminsearch('cost_condition_number',P,optimset('TolX',1e-10,'MaxFunEvals',10000,'MaxIter',10000));
%   [K_coupling, F_coupling] = coupling_control_scratch(sys_ol,C_tilde,ew_ricati,l,P);
%   %P_opt = P;
%   sys_coupling = ss(A-B*K_coupling, B*F_coupling, C_tilde, 0);
%   figure;
%   step(sys_coupling);
%   plt = step(sys_coupling);
%   figure;
%   pzmap(sys_coupling);
%   
%   figure;
%   kk = 0;
%   for i = 1:4
%       for j = 1:4
%           data = plt(:,i,j);
%           subplot(4,4,kk+1);
%           plot(data);
%           title(['In(',num2str(j),') to Out(',num2str(i),')']);
%           kk=kk+1;
%       end
%   end
%   
%   figure;
%   kk = 0;
%   for i = 1:4
%       for j = 1:4
%           data = plt(:,i+4,j);
%           subplot(4,4,kk+1);
%           plot(data);
%           title(['In(',num2str(j),') to Out(',num2str(i+4),')']);
%           kk=kk+1;
%       end
%   end

