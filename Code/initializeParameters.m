function [globalParameters,m,g,he,I_inv] = initializeParameters(add_mass)
if nargin < 1
    add_mass = false;
end
%Function initializes all Constants of the Model
%Parameters from a Boeing 757 dash 200
g = 9.81; %Gravity [m/s^2]
m = 120000; %Mass [kg]

%Refuel
m_fuel = 14000; %Mass of the fuel
time_refuel = 20*60; % Time to refuel in sec
start_time_refuel = 10; 
refuel = 0; %Without refuel = 1, with refuel = 0
if add_mass == 1
    m = m+m_fuel*1;
elseif add_mass == -1
    m = m-m_fuel*1;
end

% Modelling the atmosphere 
rho_const = 1.225; %Airdensity (simplification)
omega_e = 7.29211510*10^(-5); %Rotation Speed of earth [rad/s]
%omega_e = 0;
he = 6356752; %Earth radius [m]
Omega_e_tilde = vecToMat([0;0;omega_e]);

% Inertia and Paramethers of the plane
S = 260; % Flugfläche (wing platform area)
St = 64; %Aerea of the Tail [m^2] 
lt = 24.8; %Length to tail [m] (estimation)
l = 6.6; % generalized length [m]

%Moments of Inertia of Boeing 757-200 [kg(m^2]
% Ix = 10710000; % 
% Ixz = 0; 
% Iy = 14883800; 
% Iz = 25283271; 
% I = [Ix 0 Ixz;0 Iy 0;Ixz 0 Iz]; %Trägheitstensor
I = m*[40.07 0 2.09;0 64 0;2.09 0 99.92];
I_inv = inv(I);

b = 44.8; %Span [m] 
c = 6.6; % Wing Chord [m]

%Aerodynamical Positions
% P_centerGravity = [0.23*c;0;0];
% P_aerodynCenter = [0.12*c;0;0];
P_centerGravity = [0;0;0];
P_aerodynCenter = P_centerGravity;

%Motor and Thrust
%P_thrust = [0.23*c;0;0.1*c+1.9]; %Position of the motor [m] 
P_thrust = [0;0;1.9]; %Position of the motor [m] 
% P_thrust = P_centerGravity;
i_f = 0 ; %Direction of Thrus [rad];
F_max = m*g; %Warning - This is obviusly not the max thrust produced by the turbines. The max Turbine Force is considered on the maximum value of sigmaf

globalParameters.g = g;
globalParameters.m = m;
globalParameters.Omega_e_tilde = Omega_e_tilde;
globalParameters.rho_const = rho_const;
globalParameters.I = I;
globalParameters.I_inv = I_inv;
globalParameters.S = S;
globalParameters.St = St;
globalParameters.lt = lt;
globalParameters.b = b;
globalParameters.c = c;
globalParameters.l = l;
globalParameters.P_thrust =P_thrust;
globalParameters.P_centerGravity = P_centerGravity;
globalParameters.P_aerodynCenter = P_aerodynCenter;
globalParameters.i_f =i_f;
globalParameters.Fmax = F_max;
globalParameters.m_fuel = m_fuel;
globalParameters.time_refuel = time_refuel;
globalParameters.start_time_refuel = start_time_refuel;
globalParameters.refuel = refuel;
end

