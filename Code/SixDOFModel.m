%Simualtion of a complete 6 Degrees of Freedom Model
clc
clear
close all
system_norm = false;
deltah_offset = 10;
% Get Model Parameters 
[globalParameters,m,g,he,I_inv] = initializeParameters();
% symbolische nicht-lineare DGL mit psi
symbolic_equations;
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
Phi_init_2 = [0.2;0;0];
X_init_2 = [V_init_2;Omega_init_2;Phi_init_2;h_init_2];
U_test = [-0.12;1;0;0;-0.1;1;0;0];
X_init = [X_init_1;X_init_2];
%Trim and Lienarisation
% [X_ap_1,U_ap_1,f0_1] = trimValues(V_init_1(1),h_init_1,1);
% [X_ap_2,U_ap_2,f0_2] = trimValues(V_init_2(1),h_init_2,2);
[X_ap_1, U_ap_1] = fsolve_trim([X_init_1;zeros(4,1)]); % mit [u, phi, psi, h] = [150, 0, 0, 5000]
[X_ap_2, U_ap_2] = fsolve_trim([X_init_2;zeros(4,1)]); % mit [u, phi, psi, h] = [150, 0, 0, 5000]
X_ap = [X_ap_1;X_ap_2];
U_ap = [U_ap_1;U_ap_2];
% X_init = X_ap;
% % X_init(3)=0;
% % X_init(13)=0;
% U_test = U_ap;
% %Linearisation
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

% [A_1,B_1] = implicit_linmod(@model_implicit,X_ap_1,U_ap_1,1);
% [A_2,B_2] = implicit_linmod(@model_implicit,X_ap_2,U_ap_2,2);
[A,B,C,n,C_tilde] = defineABC(A1,A2,B1,B2);
% W_ap = C*X_ap;
X_ap_1(9) = [];
X_ap_2(9) = [];
X_init_1(9) = [];
X_init_2(9) = [];
X_ap = [X_ap_1; X_ap_2];
X_init = [X_init_1;X_init_2];
%W_ap = [X_ap_1(1); X_ap_2(1); 0; 0; X_ap_1(10); X_ap_2(10); 0; 0];


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
%% % Test Controller
 if system_norm == true
    [A,B,C] = normieren(A,B,C,eta_max,sigmaf_max,xi_max,zita_max);
  end
  
  
  %% Transfer Function Open Loop
   sys_ol = ss(A,B, C,zeros(8,8));
  
  %%Riccatti
  Q = eye(n,n);
  Q(8,8) = 100000; %Bestrafung theta
  Q(17,17) = 100000; 
  Q(9,9) = 1; % Bestrafung Höhe
  Q(18,18) = 1;
  Q(3,3) = 100; %Bestrafung Geschw. z-Komoponente
  Q(13,13) = 100;
  
  R = 1000*eye(8,8);
  R(2,2) = 4000;
  R(5,5) = 2000;
  R(6,6) = 9000;
  K = lqr(sys_ol,Q,R);
  K((abs(K)<10^-9)) = 0;
  Ak = A -B*K;
  F = -inv(C*(Ak\B));
  ew_ricati = eig(Ak);
  sys_ricati = ss(Ak,B*F,C,zeros(8,8));
  %step(sys_ricati)
%   Gamma = getgamma(A,B,C);
%   gamma_sum = sum(Gamma);
%   zero(sys_ol)
  
  %% Coupling Control (manual) 
  l = 4; %coupling conditions
  C1_tilde = C_tilde(1:l,:);
  C2_tilde = C_tilde(l+1:end,1:end);
  ew_coupling = real(ew_ricati)+imag(ew_ricati)/100;
  struct_cond.sys_ol = sys_ol ;
  struct_cond.C_tilde = C_tilde;
  struct_cond.ew_ricati = ew_ricati;
  struct_cond.l =l;
  assignin('base','struct_cond',struct_cond)
  P = ones(8,n);
  %P = fminsearch('cost_condition_number',P,optimset('TolX',1e-10,'MaxFunEvals',10000,'MaxIter',10000));
  [K_coupling, F_coupling] = coupling_control_scratch(sys_ol,C_tilde,ew_ricati,l,P);
  %P_opt = P;

