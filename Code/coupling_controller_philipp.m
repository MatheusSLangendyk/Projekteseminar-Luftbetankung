clear; close all;
addpath('gammasyn');  
startup;
SixDOFModel;
[A,B,C] = normieren(A,B,C,eta_max,sigmaf_max,xi_max,zita_max);
%% Settings
% resort
resort = zeros(8,8);
resort(1,1) = 1;
resort(2,3) = 1;
resort(3,7) = 1;
resort(4,5) = 1;
resort(5,2) = 1;
resort(6,4) = 1;
resort(7,8) = 1;
resort(8,6) = 1;
C_resort = resort*C;
T = [eye(4), zeros(4,4)];
T21 = [1 0 0 0;...
       0 1 0 0;...
       0 0 1 0;...
       0 0 0 1];
   
T22 = [-1 0 0 0;...
       0 -1 0 0;...
       0 0 -1 0;...
       0 0 0 -1];
T = [T; T21 T22];
C_tilde = T*C_resort;

%Riccatti
Q = eye(n,n);
Q(10,10) = 100; % Bestrafung Höhe
Q(20,20) = 100;
Q(3,3) = 100; %Bestrafung Geschw. z-Komoponente
Q(13,13) = 100;

R = 1000000*eye(8,8);
R(2,2) = 400000;
R(5,5) = 200000;
R(6,6) = 900000;

K = lqr(sys_ol,Q,R);
K((abs(K)<100*eps)) = 0;
Ak = A-B*K;
eigenvalues_controlled = eig(Ak);
F = -inv(C*(Ak\B));
F((abs(F)<100*eps)) = 0;

K_0 = [];
RKF_0 = {K, K_0, F};
bounds = {[] [] []};
fix = zeros(size(K));
fix_val = NaN*ones(size(fix));
R_fixed = {fix, fix_val};
fixed = {R_fixed, [], []};
number_references = size(C_tilde, 1);
system = struct('E', eye(n, n), 'A', A, 'B', B, 'C', eye(size(A,1)), 'C_ref', C_tilde);
system_properties = struct(...
		'number_states',				n,...
		'number_controls',				8,...
		'number_references',			number_references,...
		'number_models',				1,...
        'tf_structure',                 [NaN*ones(4,4) NaN*ones(4,4); zeros(4,4) NaN*ones(4,4)],...
        'RKF_0',						{RKF_0},...
        'R_bounds',                     {bounds},...
        'R_fixed',                      []...
	);
% EXACT								structurally constrained controller only EXAKT solution
% APPROXIMATE						structurally constrained controller also approximate solution
% APPROXIMATE_INEQUALITY			structurally constrained controller also approximate solution. Formulate decoupling constraints as bounds.
% NUMERIC_NONLINEAR_EQUALITY		fully numeric design with non-linear equality constraints
% NUMERIC_NONLINEAR_INEQUALITY		fully numeric design with non-linear inequality constraints

% control_design_type = GammaDecouplingStrategy.EXACT;
% control_design_type = GammaDecouplingStrategy.APPROXIMATE;
control_design_type = GammaDecouplingStrategy.APPROXIMATE_INEQUALITY
% control_design_type = GammaDecouplingStrategy.NUMERIC_NONLINEAR_EQUALITY;
% control_design_type = GammaDecouplingStrategy.NUMERIC_NONLINEAR_INEQUALITY;

%% pole area parameters
a = 0.001;
b = 0.001;
r = 5;

%% Pole area
weight = 1;
% polearea = [control.design.gamma.area.Circle(r), control.design.gamma.area.Hyperbola(a, b)];
polearea = control.design.gamma.area.Imag(1,0);
%% gammasyn options
solver = optimization.solver.Optimizer.FMINCON; % solver to use
options = optimization.options.OptionFactory.instance.options(solver,...
	'Retries',						1,...											% number of retries
	'Algorithm',					solver.getDefaultAlgorithm(),...				% algorithm of solver, for not builtin solvers name of the solver, e.g. 'snopt' for SNOPT
	'FunctionTolerance',			1E-5,...
	'StepTolerance',				1E-5,...
	'ConstraintTolerance',			1.4e-5,...
	'MaxFunctionEvaluations',		5E3,...
	'MaxIterations',				5E3,...
	'MaxSQPIter',					5E3,...
	'SpecifyObjectiveGradient',		true,...
	'SpecifyConstraintGradient',	false,...
	'CheckGradients',				false,...
	'FunValCheck',					false,...
	'FiniteDifferenceType',			'forward',...
	'Diagnostics',					false,...
	'Display',						'iter-detailed',...
	'UseParallel',					true...
);
objectiveoptions = struct(...
	'usecompiled',				false,...											% indicator, if compiled functions should be used
	'type',						GammaJType.LINEAR,...								% type of pole area weighting in objective function
	'allowvarorder',			false,...											% allow variable state number for different multi models
	'eigenvaluederivative',		GammaEigenvalueDerivativeType.VANDERAA,...
	'errorhandler',				GammaErrorHandler.ERROR,...
	'strategy',					GammaSolutionStrategy.SINGLESHOT...
);
objectiveoptions.decouplingcontrol = struct(...
	'decouplingstrategy',			control_design_type,...
	'tolerance_prefilter',			1e-1,...
	'tolerance_decoupling',			1e-1,...
	'tf_structure',					system_properties.tf_structure,...
	'solvesymbolic',				false,...
	'round_equations_to_digits',	5,...
	'sortingstrategy_decoupling',	GammaDecouplingconditionSortingStrategy.MINIMUMNORM,...	% EIGENVALUETRACKING
	'weight_decoupling',			[],...
	'allowoutputdecoupling',		true...
);

%% gammasyn_decouplingcontrol
[Kopt, Jopt, information] = control.design.gamma.gammasyn_decouplingcontrol(system, polearea, weight, system_properties.R_fixed, system_properties.RKF_0, options, objectiveoptions, system_properties.R_bounds);
if isempty(Kopt)
	return;
end

R = Kopt{1} %#ok<*NOPTS>
F = Kopt{end}
information

%% Analysis
if ~any(any(isnan(R)))
	[gain_ratio, ~, poles_all, F] = test.develop.analyze_results(system, Kopt, polearea, system_properties.tf_structure);
	Kopt{end} = F;
	minimal_deviation = 100/min(abs(gain_ratio))
end