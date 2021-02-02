%Simualtion of a complete 6 Degrees of Freedom Model
clc
clear
% close all

system_norm = false;
deltah_offset = 10;

%% Get Model Parameters 
[globalParameters,m,g,he,I_inv] = initializeParameters();

%% Initial Values both planes
u_init_1 = 150;
v_init_1 = 0;
w_init_1 = 0;
V_init_1 = [u_init_1; v_init_1; w_init_1];
Omega_init_1 = [0;0;0];
Phi_init_1 = [0;0;0];
h_init_1 = 5000;
P_e_init_1 = [0;0;-h_init_1];
%latlon_init = [40.712776;-74.005974]; %New York
latlon_init = [0;0];
X_init_1 = [V_init_1;Omega_init_1;Phi_init_1;h_init_1];

%plane 2
u_init_2 = 150;
v_init_2 = 0;
w_init_2 = 0;
V_init_2 = [u_init_2; v_init_2; w_init_2];
Omega_init_2 = [0;0;0];
Phi_init_2 = [0;0;0];
h_init_2 = h_init_1 + deltah_offset;
P_e_init_2 = [0;0;-h_init_2];
X_init_2 = [V_init_2;Omega_init_2;Phi_init_2;h_init_2];
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
X_ap_simulink = [X_ap(1:8);X_ap(11:18)];

%% DGL Flugzeug 1
% symbolische nicht-lineare DGL mit psi
plane_selector = 1;
assignin('base','plane_selector',plane_selector)
symbolic_equations;
% Zustands-DGL ohne psi
f = [du;dv;dw;dp;dq;dr;dphi;dtheta];

% Ausgangsgleichung
out_eq = [u v phi theta];

% 1. Linearisierung durch bilden der Jacoby-Matrizen
A_sym = jacobian(f, x_red_9);   % A = d f(x,u) / dx
B_sym = jacobian(f, u_stell);   % B = d f(x,u) / du
C_sym = jacobian(out_eq, x_red_9);   % C = d y(x,u) / dx
D_sym = jacobian(out_eq, u_stell);   % D = d y(x,u) / du

% 2. Einsetzten der Anfangswerte
A1 = double(subs(A_sym, [x10,u_stell], [X_ap_1; U_ap_1]'));
B1 = double(subs(B_sym, [x10,u_stell], [X_ap_1; U_ap_1]'));
C1 = double(subs(C_sym, [x10,u_stell], [X_ap_1; U_ap_1]'));
D1 = double(subs(D_sym, [x10,u_stell], [X_ap_1; U_ap_1]'));

% A1((abs(A1)<1e-9)) = 0;
% B1((abs(B1)<1e-9)) = 0;

% %% Übertragungsverhalten von Eingängen auf h
% x_h = [u w h];
% dx_h = [du;dw;dh];
% u_h = [u_stell(1:2) u_stell(4);];
% u_h_ap = [U_ap_1(1:2); U_ap_1(4)];
% x_ap_h = [X_ap_1(1:3);X_ap_1(8);X_ap_1(10)];
% out_h = h;
% A_h_sym = jacobian(dx_h, x_h);   % A = d f(x,u) / dx
% B_h_sym = jacobian(dx_h, u_h);   % B = d f(x,u) / du
% C_h_sym = jacobian(out_h, x_h);   % C = d y(x,u) / dx
% D_h_sym = jacobian(out_h, u_h);   % D = d y(x,u) / du
% 
% A_h = double(subs(A_h_sym, [x10,u_h], [X_ap_1; u_h_ap]'));
% B_h = double(subs(B_h_sym, [x10,u_h], [X_ap_1; u_h_ap]'));
% C_h = double(subs(C_h_sym, [x10,u_h], [X_ap_1; u_h_ap]'));
% D_h = double(subs(D_h_sym, [x10,u_h], [X_ap_1; u_h_ap]'));
% 
% K_h = place(A_h, B_h, [-0.7712 -0.05 -0.2]);
% % step(ss(A_h,B_h,C_h,D_h));


%% DGL Flugzeug 2
% symbolische nicht-lineare DGL mit psi
plane_selector = 2;
assignin('base','plane_selector',plane_selector)
symbolic_equations;
% Zustands-DGL ohne psi
f = [du;dv;dw;dp;dq;dr;dphi;dtheta];

% Ausgangsgleichung
out_eq = [u v phi theta];

% 1. Linearisierung durch bilden der Jacoby-Matrizen
A_sym = jacobian(f, x_red_9);   % A = d f(x,u) / dx
B_sym = jacobian(f, u_stell);   % B = d f(x,u) / du
C_sym = jacobian(out_eq, x_red_9);   % C = d y(x,u) / dx
D_sym = jacobian(out_eq, u_stell);   % D = d y(x,u) / du

A2 = double(subs(A_sym, [x10,u_stell], [X_ap_2; U_ap_2]'));
B2 = double(subs(B_sym, [x10,u_stell], [X_ap_2; U_ap_2]'));
C2 = double(subs(C_sym, [x10,u_stell], [X_ap_2; U_ap_2]'));
D2 = double(subs(D_sym, [x10,u_stell], [X_ap_2; U_ap_2]'));

% A2((abs(A2)<1e-9)) = 0;
% B2((abs(B2)<1e-9)) = 0;

%% System ein Flugzeug
% Steuerbarkeit des einzelnen Flugzeugs
rank(ctrb(A1,B1));
% Beobachtbarkeit des einzelnen Flugzeugs
rank(obsv(A1,C1));

sys1 = ss(A1, B1, C1, D1);
% K = lqr(sys1, eye(8), eye(4));
% sys1_cl = ss(A1-B1*K, B1, C1, D1);

anz_io = size(C1,1);
p = size(B1,2);
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

Dstar = [];
for k = 1:anz_io
    Dstar(k,:) = C1(k,:) * A1^(delta_k(k)-1) * B1;
end

% Prüfung auf Minimalphasigkeit
NST = zero(sys1);
minphas = 1;
for k = 1:length(NST)
    % Wenn eine Nullstelle mit positivem Realteil, 
    % System nicht minimalphasig.
    if real(NST) >= 0
       minphas = 0;
    end
end

% Prüfung auf stabile Entkoppelbarkeit
% abs(det(Dstar))<1e-8 besser, anstatt det(Dstar)==0 zur Umgehung von 
% Fehlern mit numerischen Ungenauigkeiten 
if (det(Dstar) == 0) || (minphas == 0)
  disp('System ist NICHT stabil entkoppelbar!')
else
  disp('System ist stabil entkoppelbar!')
end 

% Entkopplungsregelung 
ew_u     = [-0.2];
ew_v  = [-1.5];
ew_phi   = [-0.5,  -0.9];
ew_theta = [-0.8,  -1.6];

wpole.ew_u = ew_u;
wpole.ew_v = ew_v;
wpole.ew_phi = ew_phi;
wpole.ew_theta = ew_theta;
wpole_cell = struct2cell(wpole);

% Bestimmung der Koeffizienten qij und ki
for i=1:p
    denom = poly(wpole_cell{i,1});
    q_coeff{i} = denom ; % Koeffizienten q(i,j)
    k_coeff(i) = q_coeff{i}(end); % Koeffizienten k(i)
end

% Berechnungvorschrift des Vorfilteres F an:
F_ent = inv(Dstar)*diag(k_coeff);
    
% Berechnung der Reglermatrix R
compR = []; % Hilfsmatrix, R = inv_D_star*compR
for i=1:p
    compR(i,:) = C1(i,:)*A1^delta_k(i); % Anteil von C_star
        for j=0:delta_k(i)-1 % Summe ueber j
            compR(i,:) = compR(i,:) + q_coeff{i}(end-j)*C1(i,:)*A1^j;
        end
end
R_ent = inv(Dstar)*compR;

% Geschlossener Regelkreis
A_ent = A1-B1*R_ent; 
B_ent = B1*F_ent;
C_ent = C1;
D_ent = D1;

A_ent((abs(A_ent)<1e-9)) = 0;
B_ent((abs(B_ent)<1e-9)) = 0;

% Geschlossener Regelkreis
sys5_ent = ss(A_ent, B_ent, C_ent, D_ent, ...
          'StateName',{'u';'v';'w';'p';'q';'r';'phi';'theta'}, ...
          'InputName',{'w_{u}';'w_{v}';'w_{\Phi}';'w_{\Theta}'}, ...
          'OutputName',{'u','v','\Phi','\Theta'}, ...
          'Name','ENTKOPPELTES SYSTEM');

sys5e1 = ss(sys5_ent.a,sys5_ent.b(:,1),sys5_ent.c(1,:),sys5_ent.d(1,1));
sys5e2 = ss(sys5_ent.a,sys5_ent.b(:,2),sys5_ent.c(2,:),sys5_ent.d(2,2));
sys5e3 = ss(sys5_ent.a,sys5_ent.b(:,3),sys5_ent.c(3,:),sys5_ent.d(3,3));
sys5e4 = ss(sys5_ent.a,sys5_ent.b(:,4),sys5_ent.c(4,:),sys5_ent.d(4,4));

% Sprungantworten der 4 geregelten Regelgrößen (einzeln)
% (Theoretische Betrachtung ohne Stellgrößenbeschränkungen)
figure('Name','Sprungantworten der vier entkoppelten Strecken')
step(sys5e1,sys5e2,sys5e3,sys5e4); grid
legend('Geschwindigkeit u [m/s]','Geschwindigkeit v [m/s]',...
       'Hängewinkel \Phi [rad]', 'Längsneigung \theta [rad]', 'Location','Best')

% Plot der Sprungantworten
figure('Name','Sprungantworten nach Entkopplung')
step(sys5_ent); grid

%% Zwei Flugzeug Modell
[A,B,C,n] = defineABC(A1,A2,B1,B2,C1,C2);
C_tilde = zeros(size(C,1), size(A,1));
C_tilde(1:4,:) = C(1:4,:);
C_tilde(5:8,:) = C(1:4,:) - C(5:8,:);

% Steuerbarkeit
eigenvalues = eig(A);
%Hautus
for i = 1:n
    eig_i = eigenvalues(i);
    if rank([eig_i*eye(n,n)-A, B]) ~=n
        disp(['Eigenvalue ',num2str(eig_i),' of Gesamt-System is not controllable ']);
    end
end
[ctb, ewS, ewNS] = steuerbarHautus(ss(A,B,C_tilde,0));

% Beobachtbarkeit 
eigenvalues = eig(A);
% Hautus
for i = 1:n
    eig_i = eigenvalues(i);
    if rank([eig_i*eye(n,n)-A;C_tilde]) ~=n
        disp(['Eigenvalue ',num2str(eig_i),' of Gesamt-System is not obsarvable ']);
    end
end
[obs, ewS, ewNS] = steuerbarHautus(ss(A',C_tilde',B',0));

sys_2plane = ss(A, B, C_tilde, zeros(8,8));

p = size(B,2);
anz_io = size(C_tilde,1);
delta_k = zeros(1, anz_io);

% Berechnung der Differenzordnungen der Ausgänge
% Nutzen Sie dabei eine for-Schleife, um alle Ausgänge zu durchlaufen
% und eine while-Schleife zur Berechnung der jeweiligen Matrizen ci*(A^j)*B
for k = 1:anz_io     % alle Ausgänge nacheinander
  j=1;
  while C_tilde(k,:)*A^(j-1)*B == zeros(1,anz_io)
    j=j+1;
  end
  delta_k(k)=j;      % Differenzordnung des Ausgangs k
end

% Differenzordnung des Gesamtsystems
delta = sum(delta_k);

Dstar = [];
for k = 1:anz_io
    Dstar(k,:) = C_tilde(k,:) * A^(delta_k(k)-1) * B;
end

% Prüfung auf Minimalphasigkeit
NST = zero(sys_2plane);
minphas = 1;
for k = 1:length(NST)
    % Wenn eine Nullstelle mit positivem Realteil, 
    % System nicht minimalphasig.
    if real(NST) >= 0
       minphas = 0;
    end
end

% Prüfung auf stabile Entkoppelbarkeit
% abs(det(Dstar))<1e-8 besser, anstatt det(Dstar)==0 zur Umgehung von 
% Fehlern mit numerischen Ungenauigkeiten 
if (det(Dstar) == 0) || (minphas == 0)
  disp('System ist NICHT stabil entkoppelbar!')
else
  disp('System ist stabil entkoppelbar!')
end 

% Entkopplungsregelung 
ew_u1     = [-0.2];
ew_v1  = [-1.5];
ew_phi1   = [-0.5,  -0.9];
ew_theta1 = [-5,  -6];
ew_u2     = [-0.2];
ew_v2  = [-1.5];
ew_phi2   = [-0.5,  -0.9];
ew_theta2 = [-5,  -6];

wpole.ew_u1 = ew_u1;
wpole.ew_v1 = ew_v1;
wpole.ew_phi1 = ew_phi1;
wpole.ew_theta1 = ew_theta1;
wpole.ew_u2 = ew_u2;
wpole.ew_v2 = ew_v2;
wpole.ew_phi2 = ew_phi2;
wpole.ew_theta2 = ew_theta2;
wpole_cell = struct2cell(wpole);

% Bestimmung der Koeffizienten qij und ki
for i=1:p
    denom = poly(wpole_cell{i,1});
    q_coeff{i} = denom ; % Koeffizienten q(i,j)
    k_coeff(i) = q_coeff{i}(end); % Koeffizienten k(i)
end

% Berechnungvorschrift des Vorfilteres F an:
F_ent = inv(Dstar)*diag(k_coeff);
% modifiertes Vorfilter da die Koeffizienten für w2 nicht benötigt werden
F_mod = F_ent(:,1:p/2);
    
% Berechnung der Reglermatrix R
compR = []; % Hilfsmatrix, R = inv_D_star*compR
for i=1:p
    compR(i,:) = C_tilde(i,:)*A^delta_k(i); % Anteil von C_star
        for j=0:delta_k(i)-1 % Summe ueber j
            compR(i,:) = compR(i,:) + q_coeff{i}(end-j)*C_tilde(i,:)*A^j;
        end
end
K_coupling = inv(Dstar)*compR;

K_coupling((abs(K_coupling)<1e-9)) = 0;
F_mod((abs(F_mod)<1e-9)) = 0;

% Geschlossener Regelkreis
A_ent = A-B*K_coupling; 
B_ent = B*F_mod;
C_ent = C_tilde;
D_ent = zeros(size(C_tilde,1), size(B,2)/2);

A_ent((abs(A_ent)<1e-9)) = 0;
B_ent((abs(B_ent)<1e-9)) = 0;

% Geschlossener Regelkreis
sys5_ent = ss(A_ent, B_ent, C_ent, D_ent, ...
          'StateName',{'u1';'v1';'w1';'p1';'q1';'r1';'phi1';'theta1';...
          'u2';'v2';'w2';'p2';'q2';'r2';'phi2';'theta2'}, ...
          'InputName',{'w_{u1}';'w_{v1}';'w_{\Phi1}';'w_{\Theta1}'}, ...
          'OutputName',{'u1','v1','\Phi_1','\Theta_1',...
          'u2','v2','\Phi_2','\Theta_2'}, ...
          'Name','VERKOPPELTES SYSTEM');

% Plot der Sprungantworten
figure('Name','Sprungantworten nach Verkopplung')
step(sys5_ent); grid

%Saturations
eta_max = 10*pi/180; %Elevator
eta_min = - 25*pi/180; 
sigmaf_max = 20*pi/180; %Throttl
sigmaf_min = 0.5*pi/180;
xi_max = 25*pi/180; %Airlon
xi_min = - xi_max;
zita_max = 30*pi/180; %Rudder
zita_min = - zita_max;

noise_weight = [10 10 10 1 1 1 1 1 10 10 10 1 1 1 1 1]';
W_ap = C_tilde*X_ap_simulink;
W_ap = W_ap(1:4);
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

