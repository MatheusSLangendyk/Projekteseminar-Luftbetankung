function [F0] = cost_filter(FM)

struct_cond = evalin('base','struct_cond ');
sys_ol = struct_cond.sys_ol;
C_tilde = struct_cond.C_tilde;
ew = struct_cond.ew_ricati;
X_ap = struct_cond.X_ap;
W = struct_cond.W;
P = struct_cond.P;
t = struct_cond.t ;
l = struct_cond.l;

A = sys_ol.A;
B = sys_ol.B;
[K, F] = coupling_control_scratch2(sys_ol,C_tilde,ew,l,P,FM);
Ar = A- B*K;
Br = B*F;

sys = ss(Ar,Br,C_tilde,0);
[Y] = lsim(sys,W,t,X_ap);
Y2 = Y(end,l+1:end);
Q = eye(l);
F0 = Y2*Q*Y2';

end

