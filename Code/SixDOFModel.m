%Simualtion 6 Degrees of Freedom Model

%% Get Model Parameters 
[globalParameters,m,g,he,I_inv] = initializeParameters();
deltah_offset = 10;

%% Initial Values both planes
u_init_1 = 150;
v_init_1 = 0;
w_init_1 = 0;
V_init_1 = [u_init_1; v_init_1; w_init_1];
Omega_init_1 = [0;0;0];
Phi_init_1 = [0;0;0];
h_init_1 = 5000;
P_e_init_1 = [0;0;-h_init_1];
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
assignin('base','add_mass',0)
% AP mit fsolve
[X_ap_1, U_ap_1] = fsolve_trim([X_init_1;zeros(4,1)], 1); % mit [vA, phi, psi, h] = [150, 0, 0, 5000]
[X_ap_2, U_ap_2] = fsolve_trim([X_init_2;zeros(4,1)], 2); % mit [vA, phi, psi, h] = [150, 0, 0, 5010]

X_ap = [X_ap_1;X_ap_2];
U_ap = [U_ap_1;U_ap_2];
U_ap((abs(U_ap)<1e-9)) = 0;
X_ap((abs(X_ap)<1e-9)) = 0;
% reduzierter trim point Vektor
X_ap_simulink = [X_ap(1:8);X_ap(10);X_ap(11:18);X_ap(20)];

%% DGL Flugzeug 1
% symbolische nicht-lineare DGL mit psi
plane_selector = 1;
assignin('base','plane_selector',plane_selector)
symbolic_equations;
% Zustands-DGL ohne psi
f = [du;dv;dw;dp;dq;dr;dphi;dtheta;dh];

% Ausgangsgleichung
out_eq = [v phi theta h];

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

%% DGL Flugzeug 2
% symbolische nicht-lineare DGL mit psi
plane_selector = 2;
assignin('base','plane_selector',plane_selector)
symbolic_equations;
% Zustands-DGL ohne psi
f = [du;dv;dw;dp;dq;dr;dphi;dtheta;dh];

% Ausgangsgleichung
out_eq = [v phi theta h];

% 1. Linearisierung durch bilden der Jacoby-Matrizen
A_sym = jacobian(f, x_red_9);   % A = d f(x,u) / dx
B_sym = jacobian(f, u_stell);   % B = d f(x,u) / du
C_sym = jacobian(out_eq, x_red_9);   % C = d y(x,u) / dx
D_sym = jacobian(out_eq, u_stell);   % D = d y(x,u) / du

A2 = double(subs(A_sym, [x10,u_stell], [X_ap_2; U_ap_2]'));
B2 = double(subs(B_sym, [x10,u_stell], [X_ap_2; U_ap_2]'));
C2 = double(subs(C_sym, [x10,u_stell], [X_ap_2; U_ap_2]'));
D2 = double(subs(D_sym, [x10,u_stell], [X_ap_2; U_ap_2]'));

%% Zwei Flugzeug Modell
[A,B,C,n] = defineABC(A1,A2,B1,B2,C1,C2);
C_tilde = zeros(size(C,1), size(A,1));
C_tilde(1:4,:) = C(1:4,:);
C_tilde(5:8,:) = C(1:4,:) - C(5:8,:);

%Saturations
eta_max = 10*pi/180; %Elevator
eta_min = - 25*pi/180; 
sigmaf_max = 20*pi/180; %Throttl
sigmaf_min = 0.5*pi/180;
xi_max = 25*pi/180; %Airlon
xi_min = - xi_max;
zita_max = 30*pi/180; %Rudder
zita_min = - zita_max;
