function [X_ap, U_ap] = fsolve_trim(x0)
fun = @nonlin;
assignin('base','vA_init',x0(1))
assignin('base','phi_init',x0(7))
assignin('base','psi_init',x0(9))
assignin('base','h_init',x0(10))
ap = fsolve(fun,x0);
X_ap = ap(1:10);
U_ap = ap(11:14);
end