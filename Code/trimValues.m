function [X_ap,U_ap,f0] = trimValues(u_init,h_init,plane_selector)
%Find the Trim Point under given Conditions
%AP (equilibrium Points of the System)
%Initialize z_guess: 0 wheather start from Initial guess. 1 wheather use
%values from last run

Z_guess = zeros(14,1);
Z_guess(1) = u_init; 
Z_guess(10) = h_init;
f_prev = inf;
f0 = inf;
assignin('base','plane_selector',plane_selector)
assignin('base','h_init',h_init)
while f0 > 10e-8
    
    [Z_ap,f0] = fminsearch('cost_straight_flight',Z_guess,optimset('TolX',1e-10,'MaxFunEvals',1000000,'MaxIter',1000000));
     if (f0-f_prev)^2 < 10e-14
         break
     end
     f_prev = f0;
     Z_guess = Z_ap;
         
end
Z_ap((abs(Z_ap)<10^-9)) = 0;
X_ap = Z_ap(1:10);
U_ap = Z_ap(11:14);

end