function [F0] = cost_straight_flight(Z)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
X = Z(1:10);
U = Z(11:14);
plain_selector = evalin('base','plain_selector');
vA_init =  evalin('base','vA_init');
phi_init =  evalin('base','phi_init');
psi_init =  evalin('base','psi_init');
h_init =  evalin('base','h_init');
[dX] = nonlinear_6DOF(X,U,plain_selector);

vA = X(1);
phi = X(7);
psi = X(9);
h = X(10);

Q = [dX;vA-vA_init;phi-phi_init;psi-psi_init;h-h_init];
H = eye(14);
% H(4:6,4:6) = 100*eye(3);
% H(7,7) = 100;
% H(9,9) = 100;
F0 = Q'*H*Q;
end

