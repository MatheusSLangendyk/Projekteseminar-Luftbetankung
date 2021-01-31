function [F0] = cost_coupling_control(P)
%Minimzation of the conditional number of the closed-loop
 struct_cond = evalin('base','struct_cond ');
 sys_ol = struct_cond.sys_ol;
 C_tilde = struct_cond.C_tilde;
 ew_ricati = struct_cond.ew_ricati;
 l = struct_cond.l;
 X_init = struct_cond.X_init;
 opt_modus = struct_cond.opt_modus;
 W = [5;0.01;0;10;0;0;0;0]; %Reference Variable
 A = sys_ol.A;
 B = sys_ol.B;
 p = size(B,2);
 n = size(A,1);
 [K_coupling, F_coupling] = coupling_control_scratch2(sys_ol,C_tilde,ew_ricati,l,P);
 Ar = A- B*K_coupling;
 if opt_modus == 0 %Conditional Number optimization
    F0 = cond(Ar);
 elseif opt_modus == 1 %Control Variable optimization
     u = F_coupling*W- K_coupling*X_init ;
     F0 = sum(u);
 else
     F0 =ones(1,p)*abs(K_coupling)*ones(n,1); %Control-Matrix Values optimization
     
end

