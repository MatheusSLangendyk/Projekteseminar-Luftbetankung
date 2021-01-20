clear; close all;
addpath('gammasyn');  
startup;
SixDOFModel;
% [A,B,C] = normieren(A,B,C,eta_max,sigmaf_max,xi_max,zita_max);
%% Settings
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
C_tilde = T*C;
% C_tilde(5, :) = [0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0];
% C_tilde(6, :) = [0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 0];
% C_tilde(7, :) = [1 0 0 0 0 0 0 0 0 0 -1 0 0 0 0 0 0 0 0 0];
X_init = [160 0 0 0 0 0 0 0 0 8000 160 0 0 0 0 0 0 0 0 8010]';

%Riccatti
Q = eye(n,n);
% Q(10,10) = 100; % Bestrafung Höhe
% Q(20,20) = 100;
% Q(3,3) = 100; %Bestrafung Geschw. z-Komoponente
% Q(13,13) = 100;

R = eye(8,8);
% R = 1000*R;
% R(2,2) = 400000;
% R(5,5) = 200000;
% R(6,6) = 900000;

K = lqr(ss(A, B, C, 0),Q,R);
K((abs(K)<1e-9)) = 0;
Ak = A-B*K;
eigenvalues_controlled = eig(Ak);
F = -inv(C*(Ak\B));
F((abs(F)<1e-9)) = 0;
F_coupling = F;
K_coupling = K;

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
%         'tf_structure',                 [NaN*ones(6,6) NaN*ones(6,2); zeros(2,6) NaN*ones(2,2)],...

% EXACT								structurally constrained controller only EXAKT solution
% APPROXIMATE						structurally constrained controller also approximate solution
% APPROXIMATE_INEQUALITY			structurally constrained controller also approximate solution. Formulate decoupling constraints as bounds.
% NUMERIC_NONLINEAR_EQUALITY		fully numeric design with non-linear equality constraints
% NUMERIC_NONLINEAR_INEQUALITY		fully numeric design with non-linear inequality constraints

% control_design_type = GammaDecouplingStrategy.EXACT;
control_design_type = GammaDecouplingStrategy.APPROXIMATE;
% control_design_type = GammaDecouplingStrategy.APPROXIMATE_INEQUALITY
% control_design_type = GammaDecouplingStrategy.NUMERIC_NONLINEAR_EQUALITY;
% control_design_type = GammaDecouplingStrategy.NUMERIC_NONLINEAR_INEQUALITY;

%% pole area parameters
a = 0.001;
b = 0.002;
r = 10;

%% Pole area
weight = [10];
% polearea = [control.design.gamma.area.Circle(r), control.design.gamma.area.Hyperbola(a, b)];
polearea = control.design.gamma.area.Imag(1,-5);
% polearea = [control.design.gamma.area.Hyperbola(a, b), control.design.gamma.area.Imag(1,0)];
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
	'tolerance_prefilter',			1e-3,...
	'tolerance_decoupling',			1e-3,...
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

K_coupling = Kopt{1} %#ok<*NOPTS>
F_coupling = Kopt{end}
information
W_ap = [135 0 0 8000 0 0 0 0]';
sys_riccati = ss(A-B*K, B*F, C, 0);
sys_1_riccati = ss(A_1-B_1*K(1:4, 1:10), B_1*F(1:4, 1:4), C(1:4, 1:10), 0); 
sys_coupling = ss(A-B*K_coupling, B*F_coupling, C_tilde, 0);
pzmap(sys_coupling, 'r', sys_riccati, 'g');
[num_tf_coupling, denum_tf_coupling] = ss2tf(A-B*K_coupling, B*F_coupling, C_tilde, zeros(8,8), 1);

%%
C_1=C(1:4, 1:10);
anz_io = size(C_1,1);
delta_k = zeros(1, anz_io);

% Berechnung der Differenzordnungen der Ausgänge
% Nutzen Sie dabei eine for-Schleife, um alle Ausgänge zu durchlaufen
% und eine while-Schleife zur Berechnung der jeweiligen Matrizen ci*(A^j)*B
for k = 1:anz_io     % alle Ausgänge nacheinander
  j=1;
  while C_1(k,:)*A_1^(j-1)*B_1 == zeros(1,anz_io)
    j=j+1;
  end
  delta_k(k)=j;      % Differenzordnung des Ausgangs k
end

% Differenzordnung des Gesamtsystems
delta = sum(delta_k);


% %% Analysis
% if ~any(any(isnan(R)))
% 	[gain_ratio, ~, poles_all, F] = test.develop.analyze_results(system, Kopt, polearea, system_properties.tf_structure);
% 	Kopt{end} = F;
% 	minimal_deviation = 100/min(abs(gain_ratio))
% end