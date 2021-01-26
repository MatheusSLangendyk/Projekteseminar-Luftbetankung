function [X_ap,U_ap,f0] = trimValues(vA_init,alpha_init,beta_init,Omega_init,Phi_init,h_init,plain_selector)
%Find the Trim Point under given Conditions
%AP (equilibrium Points of the System)
%Initialize z_guess: 0 wheather start from Initial guess. 1 wheather use
%values from last run

Z_guess = zeros(14,1);
Z_guess(1) = vA_init; 
Z_guess(2) = alpha_init; 
Z_guess(3) = beta_init; 
Z_guess(4:6) = Omega_init;
Z_guess(7:9) = Phi_init;
Z_guess(10) = h_init;
f_prev = inf;
f0 = inf;
assignin('base','plain_selector',plain_selector)
assignin('base','vA_init',vA_init)
assignin('base','phi_init',Phi_init(1))
assignin('base','psi_init',Phi_init(3))
assignin('base','h_init',h_init)

while f0 > 10e-5
    
%     [Z_ap,f0] = fminsearch('cost_straight_flight',Z_guess,optimset('TolX',1e-10,'MaxFunEvals',1000,'MaxIter',1000));
    [Z_ap,f0] = fminsearch('cost_straight_flight',Z_guess);

     if (f0-f_prev)^2 < eps
         break
     end
     f_prev = f0;
     Z_guess = Z_ap;
         
end
X_ap = Z_ap(1:10);
U_ap = Z_ap(11:14);

end