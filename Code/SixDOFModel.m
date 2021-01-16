%Simualtion of a complete 6 Degrees of Freedom Model
clc
clear
close all
% Get Model Parameters 
[globalParameters,m,g,he,I_inv] = initializeParameters();
%Initial Values
h_init_1 = 8000;
P_e_init_1 = [0;0;-h_init_1];
V_init_1 = [130;0;0];
%latlon_init = [40.712776;-74.005974]; %New York
latlon_init = [0;0];
Omega_init_1 = [0;0;0];
Phi_init_1 = [0;0;0];
X_init_1 = [V_init_1;Omega_init_1;Phi_init_1;h_init_1];

%Plain 2
h_init_2 = 8010;
P_e_init_2 = [0;0;-h_init_2];
V_init_2 = [130;0;0];
Omega_init_2 = [0;0;0];
Phi_init_2 = [0;0;0];
X_init_2 = [V_init_2;Omega_init_2;Phi_init_2;h_init_2];
U_test = [-0.12;1;0;0;-0.1;1;0;0];
X_init = [X_init_1;X_init_2];
%Trim and Lienarisation
[X_ap_1,U_ap_1,f0_1] = trimValues(V_init_1(1),h_init_1,1);
[X_ap_2,U_ap_2,f0_2] = trimValues(V_init_2(1),h_init_2,2);
X_ap = [X_ap_1;X_ap_2];
U_ap = [U_ap_1;U_ap_2];
%X_init = X_ap;
%U_test = U_ap;
% %Linearisation

[A_1,B_1] = implicit_linmod(@model_implicit,X_ap_1,U_ap_1,1);
[A_2,B_2] = implicit_linmod(@model_implicit,X_ap_2,U_ap_2,2);
[A,B,C,n] = defineABC(A_1,A_2,B_1,B_2);

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

%% Transfer Function Open Loop
sys_ol = ss(A,B, C,zeros(8,8));

Q = eye(n,n);
Q(8,8) = 1; %Bestrafung theta
Q(18,18) = 1; 
Q(9,9) = 100; %Bestrafung psi
Q(19,19) = 100; 
Q(10,10) = 1000; % Bestrafung Höhe
Q(20,20) = 1000;
Q(3,3) = 1000; %Bestrafung Geschw. z-Komoponente
Q(13,13) = 1000;
  
R = 1000*eye(8,8);
R(2,2) = 4000;
R(5,5) = 2000;
R(6,6) = 9000;
K = lqr(sys_ol,Q,R);
Ak = A -B*K;
ew_ricati = eig(Ak);
F = -inv(C*(Ak\B));
sys_ricati = ss(Ak,B*F,C,zeros(8,8));
W_ap = C*X_ap;
   