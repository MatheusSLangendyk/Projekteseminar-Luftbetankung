function [systems, AP, X_init] = create_multi_model(multi)
if nargin < 1
    multi = 'one';
end
fields = {'E','A','B','C','C_ref'};
c = cell(length(fields),1);
systems = cell2struct(c,fields);

fields = {'X_ap', 'U_ap'};
c = cell(length(fields),1);
AP = cell2struct(c,fields);

system_norm = false;
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

X_init = [X_init_1; X_init_2];

switch multi
    case 'multi'
        %% trim points nur eigene Flugzeugmasse
        
        assignin('base','add_mass',0)
        % AP mit fsolve
        [X_ap_1, U_ap_1] = fsolve_trim([X_init_1;zeros(4,1)], 1); % mit [vA, phi, psi, h] = [150, 0, 0, 5000]

        assignin('base','add_mass',0)
        [X_ap_2, U_ap_2] = fsolve_trim([X_init_2;zeros(4,1)], 2); % mit [vA, phi, psi, h] = [150, 0, 0, 5010]

        X_ap = [X_ap_1;X_ap_2];
        U_ap = [U_ap_1;U_ap_2];
        U_ap((abs(U_ap)<1e-9)) = 0;
        X_ap((abs(X_ap)<1e-9)) = 0;
        X_ap_simulink = [X_ap(1:8);X_ap(10);X_ap(11:18);X_ap(20)];

        % DGL Flugzeug 1
        % symbolische nicht-lineare DGL mit psi
        plane_selector = 1;
        assignin('base','plane_selector',plane_selector)
        assignin('base','add_mass',0)

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

        % DGL Flugzeug 2
        % symbolische nicht-lineare DGL mit psi
        plane_selector = 2;
        assignin('base','plane_selector',plane_selector)
        assignin('base','add_mass',0)

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

        % Zwei Flugzeug Modell eigene Masse
        [A,B,C,n] = defineABC(A1,A2,B1,B2,C1,C2);
        C_tilde = zeros(size(C,1), size(A,1));
        C_tilde(1:4,:) = C(1:4,:);
        C_tilde(5:8,:) = C(1:4,:) - C(5:8,:);

        AP(1,1).X_ap = X_ap;
        AP(1,1).X_ap_simulink = X_ap_simulink;
        AP(1,1).U_ap = U_ap;
        systems(1,1).E = eye(n);
        systems(1,1).A = A;
        systems(1,1).B = B;
        systems(1,1).C = eye(n);
        systems(1,1).C_ref = C_tilde;

        %% Flugzeug 1 mit fuel_mass F2 nur mit eigener Masse
        % calculate trim points 
        assignin('base','add_mass',1)
        % AP mit fsolve
        [X_ap_1, U_ap_1] = fsolve_trim([X_init_1;zeros(4,1)], 1); % mit [vA, phi, psi, h] = [150, 0, 0, 5000]

        assignin('base','add_mass',0)
        [X_ap_2, U_ap_2] = fsolve_trim([X_init_2;zeros(4,1)], 2); % mit [vA, phi, psi, h] = [150, 0, 0, 5010]

        X_ap = [X_ap_1;X_ap_2];
        U_ap = [U_ap_1;U_ap_2];
        U_ap((abs(U_ap)<1e-9)) = 0;
        X_ap((abs(X_ap)<1e-9)) = 0;
        X_ap_simulink = [X_ap(1:8);X_ap(10);X_ap(11:18);X_ap(20)];

        % DGL Flugzeug 1
        % symbolische nicht-lineare DGL mit psi
        plane_selector = 1;
        assignin('base','plane_selector',plane_selector)
        assignin('base','add_mass',1)

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

        % DGL Flugzeug 2
        % symbolische nicht-lineare DGL mit psi
        plane_selector = 2;
        assignin('base','plane_selector',plane_selector)
        assignin('base','add_mass',0)

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

        % Zwei Flugzeug Modell 
        [A,B,C,n] = defineABC(A1,A2,B1,B2,C1,C2);
        C_tilde = zeros(size(C,1), size(A,1));
        C_tilde(1:4,:) = C(1:4,:);
        C_tilde(5:8,:) = C(1:4,:) - C(5:8,:);

        AP(2,1).X_ap = X_ap;
        AP(2,1).U_ap = U_ap;
        AP(2,1).X_ap_simulink = X_ap_simulink;
        systems(2,1).E = eye(n);
        systems(2,1).A = A;
        systems(2,1).B = B;
        systems(2,1).C = eye(n);
        systems(2,1).C_ref = C_tilde;

        %% FLugzeug 2 mit fuel_mass F1 mit eigener Masse
        % calculate trim points 
        assignin('base','add_mass',0)
        % AP mit fsolve
        [X_ap_1, U_ap_1] = fsolve_trim([X_init_1;zeros(4,1)], 1); % mit [vA, phi, psi, h] = [150, 0, 0, 5000]

        assignin('base','add_mass',1)
        [X_ap_2, U_ap_2] = fsolve_trim([X_init_2;zeros(4,1)], 2); % mit [vA, phi, psi, h] = [150, 0, 0, 5010]

        X_ap = [X_ap_1;X_ap_2];
        U_ap = [U_ap_1;U_ap_2];
        U_ap((abs(U_ap)<1e-9)) = 0;
        X_ap((abs(X_ap)<1e-9)) = 0;
        X_ap_simulink = [X_ap(1:8);X_ap(10);X_ap(11:18);X_ap(20)];

        % DGL Flugzeug 1
        % symbolische nicht-lineare DGL mit psi
        plane_selector = 1;
        assignin('base','plane_selector',plane_selector)
        assignin('base','add_mass',0)

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

        % DGL Flugzeug 2
        % symbolische nicht-lineare DGL mit psi
        plane_selector = 2;
        assignin('base','plane_selector',plane_selector)
        assignin('base','add_mass',1)

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

        % Zwei Flugzeug Modell 
        [A,B,C,n] = defineABC(A1,A2,B1,B2,C1,C2);
        C_tilde = zeros(size(C,1), size(A,1));
        C_tilde(1:4,:) = C(1:4,:);
        C_tilde(5:8,:) = C(1:4,:) - C(5:8,:);

        AP(3,1).X_ap = X_ap;
        AP(3,1).U_ap = U_ap;
        AP(3,1).X_ap_simulink = X_ap_simulink;
        systems(3,1).E = eye(n);
        systems(3,1).A = A;
        systems(3,1).B = B;
        systems(3,1).C = eye(n);
        systems(3,1).C_ref = C_tilde;
        
    case 'one'
        %% trim points nur eigene Flugzeugmasse
        assignin('base','add_mass',0)
        % AP mit fsolve
        [X_ap_1, U_ap_1] = fsolve_trim([X_init_1;zeros(4,1)], 1); % mit [vA, phi, psi, h] = [150, 0, 0, 5000]

        assignin('base','add_mass',0)
        [X_ap_2, U_ap_2] = fsolve_trim([X_init_2;zeros(4,1)], 2); % mit [vA, phi, psi, h] = [150, 0, 0, 5010]

        X_ap = [X_ap_1;X_ap_2];
        U_ap = [U_ap_1;U_ap_2];
        U_ap((abs(U_ap)<1e-9)) = 0;
        X_ap((abs(X_ap)<1e-9)) = 0;
        X_ap_simulink = [X_ap(1:8);X_ap(10);X_ap(11:18);X_ap(20)];

        % DGL Flugzeug 1
        % symbolische nicht-lineare DGL mit psi
        plane_selector = 1;
        assignin('base','plane_selector',plane_selector)
        assignin('base','add_mass',0)

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

        % DGL Flugzeug 2
        % symbolische nicht-lineare DGL mit psi
        plane_selector = 2;
        assignin('base','plane_selector',plane_selector)
        assignin('base','add_mass',0)

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

        % Zwei Flugzeug Modell eigene Masse
        [A,B,C,n] = defineABC(A1,A2,B1,B2,C1,C2);
        C_tilde = zeros(size(C,1), size(A,1));
        C_tilde(1:4,:) = C(1:4,:);
        C_tilde(5:8,:) = C(1:4,:) - C(5:8,:);

        AP(1,1).X_ap = X_ap;
        AP(1,1).U_ap = U_ap;
        AP(1,1).X_ap_simulink = X_ap_simulink;
        systems(1,1).E = eye(n);
        systems(1,1).A = A;
        systems(1,1).B = B;
        systems(1,1).C = eye(n);
        systems(1,1).C_ref = C_tilde;
end
end