function [F0] = cost_straight_flight(Z)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
X = Z(1:10);
U = Z(11:14);
plane_selector = evalin('base','plane_selector');
h_init =  evalin('base','h_init');
[dX] = nonlinear_6DOF(X,U,plane_selector);

v = X(2);
phi = X(7);
psi = X(9);
h = X(10);

Q = [dX;v;phi;psi;h-h_init];
H = diag(ones(1,14));
H(11,11) = 100;
H(12,12) = 100;
H(13,13) = 100;
F0 = Q'*H*Q;
end

