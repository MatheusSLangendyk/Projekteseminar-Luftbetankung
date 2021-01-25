%Simualtion of a complete 6 Degrees of Freedom Model
clc
clear
close all

% symbolische nicht-lineare DGL mit psi
symbolic_equations;

system_norm = false;
deltah_offset = 10;
% Get Model Parameters 
[globalParameters,m,g,he,I_inv] = initializeParameters();
%Initial Values
h_init_1 = 5000;
P_e_init_1 = [0;0;-h_init_1];
V_init_1 = [150;0;0];
%latlon_init = [40.712776;-74.005974]; %New York
latlon_init = [0;0];
Omega_init_1 = [0;0;0];
Phi_init_1 = [0;0;0];
X_init_1 = [V_init_1;Omega_init_1;Phi_init_1;h_init_1];

%Plain 2
h_init_2 = h_init_1+deltah_offset;
P_e_init_2 = [0;0;-h_init_2];
V_init_2 = [150;0;0];
Omega_init_2 = [0;0;0];
Phi_init_2 = [0;0;0];
X_init_2 = [V_init_2;Omega_init_2;Phi_init_2;h_init_2];
X_init = [X_init_1;X_init_2];

ap_solver =  1;
if ap_solver == 0
    % AP mit fsolve
    [X_ap_1, U_ap_1] = fsolve_trim([X_init_1;zeros(4,1)]); % mit [u, phi, psi, h] = [150, 0, 0, 5000]
    [X_ap_2, U_ap_2] = fsolve_trim([X_init_2;zeros(4,1)]); % mit [u, phi, psi, h] = [150, 0, 0, 5000]
elseif ap_solver == 1
    % AP mit trimValues
    [X_ap_1,U_ap_1,f0_1] = trimValues(V_init_1,Omega_init_1,Phi_init_1,h_init_1,1);
    [X_ap_2,U_ap_2,f0_2] = trimValues(V_init_2,Omega_init_2,Phi_init_2,h_init_2,2);
end
% x_ap_comp = [[X_ap_1;U_ap_1] [X_ap_1_t;U_ap_1_t]];
X_ap = [X_ap_1;X_ap_2];
U_ap = [U_ap_1;U_ap_2];
X_ap_simulink = [X_ap(1:8);X_ap(10:18);X_ap(20)];
X_ap_simulink((abs(X_ap_simulink)<1e-6)) = 0;
U_ap((abs(U_ap)<1e-6)) = 0;

% Zustands-DGL ohne psi
f = [du;dv;dw;dp;dq;dr;dphi;dtheta;dh];

% Ausgangsgleichung
h = [u phi theta h];

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
rank(ctrb(A,B))
% Beobachtbarkeit 
rank(obsv(A,C))

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
% W_ap = C*X_ap;
W_ap = [X_ap_1(1); X_ap_2(1); 0; 0; X_ap_1(10); X_ap_2(10); 0; 0];


%Saturations
eta_max = 10*pi/180; %Elevator
eta_min = - 25*pi/180; 
sigmaf_max = 10*pi/180; %Throttl
sigmaf_min = 0.5*pi/180;
xi_max = 25*pi/180; %Airlon
xi_min = - xi_max;
zita_max = 30*pi/180; %Rudder
zita_min = - zita_max;
% %Contolable/ Obsarvable
Ms = ctrb(A,B);
Mb = obsv(A,C);
if rank(Ms) ~= min(size(Ms,1),size(Ms,2))
    disp('System is not Kalman Controllable')
else
    disp('System is Kalman Controllable')
end
if rank(Mb) ~= min(size(Mb,1),size(Mb,2))
    disp('System is not Kalman Obsarvable')
else
    disp('System is Kalman Obsarvable')
end

eigenvalues = eig(A);

%Hautus
for i = 1:n
    eig_i = eigenvalues(i);
    if rank([eig_i*eye(n,n)-A;C]) ~=n
        
        disp(['Eigenvalue ',num2str(eig_i),' is not obsarvable ']);
    end
end
%% Normieren
 if system_norm == true
    [A,B,C] = normieren(A,B,C,eta_max,sigmaf_max,xi_max,zita_max);
 end
