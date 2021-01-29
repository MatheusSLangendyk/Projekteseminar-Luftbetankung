function [dX] = nonlinear_6DOF(X)
u_init =  evalin('base','u_init');
phi_init =  evalin('base','phi_init');
psi_init =  evalin('base','psi_init');
h_init =  evalin('base','h_init');
plane_selector =  evalin('base','plane_selector');

%--------------States------------%
u = X(1);
v = X(2);
w = X(3);
p = X(4);
q = X(5);
r = X(6);
phi = X(7);
theta = X(8);
psi = X(9);
h = X(10);

%--------------Control------------%
eta = X(11);
sigmaf = X(12);
xi = X(13);
zita = X(14);

%--------------Constants------------%
[globalParameters,m,g,~,I_inv] = initializeParameters();
T   = 288.15 - 0.0065*h;                    % Temperatur bei H
ps  = 101325*(T/288.15)^(g/1.86584);        % Statischer Luftdruck
rho = ps/(287.053*T);                       % Luftdichte = ps/(R*T)

Omega_e_tilde = globalParameters.Omega_e_tilde ;
if plane_selector == 1
    I = globalParameters.I ;
    S = globalParameters.S ;
    St = globalParameters.St ;
    lt = globalParameters.lt ;
    b = globalParameters.b ;
    l = globalParameters.l;
    c = globalParameters.c ;
    P_thrust = globalParameters.P_thrust ;
    P_centerGravity = globalParameters.P_centerGravity ;
    P_aerodynCenter = globalParameters.P_aerodynCenter ;
    i_f = globalParameters.i_f ;
    F_max = globalParameters.Fmax;
elseif plane_selector == 2
    I = globalParameters.I ;
    S = globalParameters.S ;
    St = globalParameters.St ;
    lt = globalParameters.lt ;
    b = globalParameters.b ;
    l = globalParameters.l;
    c = globalParameters.c ;
    P_thrust = globalParameters.P_thrust ;
    P_centerGravity = globalParameters.P_centerGravity ;
    P_aerodynCenter = globalParameters.P_aerodynCenter ;
    i_f = globalParameters.i_f ;
    F_max = globalParameters.Fmax;
end
    
% Airdynamical Coefficients
%CA0 = 1.104;
grad_alpha = 5.5;
alpha_L0 = 11.5*pi/180;
gradient_alpha_epsolon = 0.25;
gradient_CQ_Cbeta = -1.6;
gradientCQ_Czita = 0.24;
CW0 = 0.13;
kappa = 0.07;

% %--------------Variables------------%
% u = sqrt(vA^2*(1-sin(beta)^2)/(1+tan(alpha)^2));
% v = sin(beta)*vA;
% w = u*tan(alpha);
V = [u;v;w];
vA = sqrt(u^2+v^2+w^2);
alpha = atan(w/u);
beta = asin(v/vA);

q_d = 0.5*rho*vA^2; %Dynamic Preassure
Omega = [p;q;r];

%--------------Aerodynamical Coefficients------------%
%Forces Coefficients
CA_F = grad_alpha*(alpha - alpha_L0);
epsolon = gradient_alpha_epsolon*(alpha - alpha_L0); %Downwash [rad]
alpha_t = alpha - epsolon + eta +1.3*q*lt/vA; %Angle of Attack of the Tail [rad]
CA_H = 3.1*(St/S)*alpha_t; %Lift Coefficient of the Control Aereas (Elevator-Tail)
CA = CA_F + CA_H; %Total Lift Coefficient

CW = CW0 + kappa*(CA_F - 0.45)^2; %Drag Coefficient

CQ = gradient_CQ_Cbeta*beta + gradientCQ_Czita*zita; % Sideforce coefficient

%Moments Coefficients
N = [-1.4*beta;-0.59-3.1*(St*lt/(l*S))*(alpha-epsolon);(1-alpha*(180/(15*pi)))*beta]; %Static Moments Effects
Jakobi_CM_X = (l/vA)*[-11,    0,           5;
                      0,-4.03*St*lt^2/(S*l^2),0;
                      1.7,          0   , -11.5];
Jakobi_CM_U = [-0.6, 0, 0.22;
               0,    -3.1*St*lt/(S*l),0;
               0, 0, -0.63];      % Dependence of Control Variables
           
C_torque = N + Jakobi_CM_X*Omega + Jakobi_CM_U*[xi;eta;zita];
CL = C_torque(1);
CM = C_torque(2);
CN = C_torque(3);

%-------------- Forces and Moments ------------%
%Aerodynamics
A = q_d*S*CA; %Lift Force [N]
W = q_d*S*CW; %Air Resistance [N]
Q = q_d*S*CQ; %Transverse Force [N]

% EVTL VORZEICHEN VON BETA ÄNDERN
Tfa = coordTransfMatrix(alpha,2)*coordTransfMatrix(-beta,3); %Transformation Matrix Aerodynamics -> Körperfest

RA_a =[-W;Q;-A]; %Aerodynamical Force in Aerdodynamical Reference Frame
RA = Tfa*RA_a; %Aerodynamical Force in Body Reference Frame
QA_aerodynCenter = q_d*S*[b*CL;c*CM;b*CN]; %Torque on the aerodynamical Center
QA = QA_aerodynCenter + vecToMat(P_centerGravity- P_aerodynCenter)*RA; %Aerodynamical Force in Body Reference Frame


%Thrust
F_res = F_max*sigmaf; %Thrust Force Absolute Value
Rf = F_res*[cos(i_f);0;sin(i_f)]; %Thrust Force 
Qf = vecToMat(P_thrust - P_centerGravity)*Rf; %Thrust Moment

Q_total = Qf + QA; %Sum of external Moments in Body reference Frame [Nm]
R_total = RA + Rf; %Sum of external Forces in Body reference Frame [N]

%-------------- Equations of Motion ------------%
%Dynamic
Tfg =  coordTransfMatrix(phi,1)*coordTransfMatrix(theta,2)*coordTransfMatrix(psi,3); % Transformation Matrix goedetic --> body reference Frame
Tgf = Tfg.';%Transformation to North East Down
Omega_tilde = vecToMat(Omega);
dOmega = I_inv*(Q_total - Omega_tilde*I*Omega); %Derivative of Rotation Rate (body Reference Frame)

% ACHTUNG: Omega_e_tilde wurde in DGL für dV entfernt, da flache ERde
dV = R_total/m + Tfg*[0;0;g] - (Omega_tilde)*V; %Derivatitive of the Speed (body Reference Frame)
% dvA = (u*dV(1) + v*dV(2) + w*dV(3))/sqrt(u^2 + v^2 + w^2); %Derivative of the Approach Speed
% dalpha = (dV(3)*u - dV(1)*w)/(u^2 + u*w); %Derivative of Angle of Attack
% dbeta = (dV(2)*vA - dvA*V(2))/(vA*sqrt(vA^2 - V(2)^2)); %Derivative of Sideslip Angle

%Kinematics
dP_e = Tgf*V; % gilt nur für z-Komponente

% ACHTUNG: dh anders als im Buch ... Teg fehlt, sollte für dh keinen
% Unterschied machen
dh = - dP_e(3); %Derivative of z-position (earth Reference Frame)

J = 1/cos(theta)*[cos(theta) sin(phi)*sin(theta) cos(phi)*sin(theta) ;0 cos(phi)*cos(theta) -sin(phi)*cos(theta);0 sin(phi) cos(phi)]; %Rotation rate matrix
dPhi = J*Omega; %Derivative of Euler Angles
dX = [dV;dOmega;dPhi;dh;u-u_init;phi-phi_init;psi-psi_init;h-h_init];
end