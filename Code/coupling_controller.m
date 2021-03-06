clear; close all;
addpath('gammasyn');  
startup;
SixDOFModel;
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
Ak = A -B*K;
eigenvalues_controlled = eig(Ak);
F = -inv(C*(Ak\B));
K_0 = [];
RKF_0 = {K, K_0, F};

system = struct('A', A, 'B', B, 'C', eye(size(A,1)), 'C_ref', C_tilde);
system_properties = struct(...
		'number_states',				n,...
		'number_controls',				8,...
		'number_couplingconditions',	4,...
		'number_models',				1,...
        'RKF_0',						{RKF_0}...
	);
% EXACT								structurally constrained controller only EXAKT solution
% APPROXIMATE						structurally constrained controller also approximate solution
% NUMERIC_NONLINEAR_EQUALITY		fully numeric design with non-linear equality constraints
% NUMERIC_NONLINEAR_INEQUALITY		fully numeric design with non-linear inequality constraints

% control_design_type = GammaCouplingStrategy.EXACT;
control_design_type = GammaCouplingStrategy.APPROXIMATE;
% control_design_type = GammaCouplingStrategy.NUMERIC_NONLINEAR_EQUALITY;
% control_design_type = GammaCouplingStrategy.NUMERIC_NONLINEAR_INEQUALITY;

tolerance_NUMERIC_NONLINEAR_INEQUALITY = 0.001;

%% pole area parameters
a = 0.001;
b = 0.001;
r = 5;

%% gammasyn options
solver = optimization.solver.Optimizer.FMINCON; %FMINCON;% solver to use
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
	'SpecifyConstraintGradient',	true,...
	'CheckGradients',				false,...
	'FunValCheck',					false,...
	'FiniteDifferenceType',			'forward',...
	'Diagnostics',					false,...
	'Display',						'iter-detailed'...
	);
objectiveoptions = struct(...
	'usecompiled',				true,...											% indicator, if compiled functions should be used
	'type',						GammaJType.LINEAR,...								% type of pole area weighting in objective function
	'allowvarorder',			false,...											% allow variable state number for different multi models
	'eigenvaluederivative',		GammaEigenvalueDerivativeType.VANDERAA,...
	'errorhandler',				GammaErrorHandler.ERROR,...
	'strategy',					GammaSolutionStrategy.SINGLESHOT...
	);

%% Pole area
weight = 1;
polearea = [control.design.gamma.area.Circle(r), control.design.gamma.area.Hyperbola(a, b)];

%% gammasyn_couplingcontrol
objectiveoptions.couplingcontrol.couplingstrategy = control_design_type;
objectiveoptions.couplingcontrol.tolerance_coupling = tolerance_NUMERIC_NONLINEAR_INEQUALITY;
objectiveoptions.couplingcontrol.couplingconditions = uint32(system_properties.number_couplingconditions);
objectiveoptions.couplingcontrol.solvesymbolic = true;
objectiveoptions.couplingcontrol.round_equations_to_digits = 5;
objectiveoptions.couplingcontrol.sortingstrategy_coupling = [];
[Kopt, Jopt, information] = control.design.gamma.gammasyn_couplingcontrol(system, polearea, weight, [], system_properties.RKF_0, options, objectiveoptions);
if isempty(Kopt)
	return;
end

R = Kopt{1}
F = Kopt{end}
information

%% Analysis
if ~any(any(isnan(R)))
	[gain_ratio, ~, poles_all, F] = test.develop.analyze_results(systems, Kopt, polearea, system_properties.number_couplingconditions);
	Kopt{end} = F;
	minimal_deviation = 100/min(abs(gain_ratio))
end