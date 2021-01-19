function [F0] = cost_core_deviance(M)
H = evalin('base','H');
Q = H*M;
I = diag(ones(1,24));
F0 = Q'*I*Q;
end

