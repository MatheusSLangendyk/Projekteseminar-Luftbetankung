function [FVAL] = model_implicit(dX,X,U)
%Organize the non-linear model in an implicit form 
plane_selector = evalin('base','plane_selector');
FVAL = nonlinear_6DOF(X,U,plane_selector) - dX;
end

