function [cond_number] = cost_condition_number(P)
 struct_cond = evalin('base','struct_cond ');
 sys_ol = struct_cond.sys_ol;
 C_tilde = struct_cond.C_tilde;
 ew_ricati = struct_cond.ew_ricati;
 l = struct_cond.l;
 
 A = sys_ol.A;
 B = sys_ol.B;
 [K_coupling, ~] = coupling_control_scratch(sys_ol,C_tilde,ew_ricati,l,P);
 Ar = A- B*K_coupling;
 cond_number = cond(Ar);
end

