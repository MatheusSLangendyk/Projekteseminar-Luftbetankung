% clear; close all;
addpath('gammasyn');  
startup;
SixDOFModel;
% [A,B,C] = normieren(A,B,C,eta_max,sigmaf_max,xi_max,zita_max);
W_ap = (C*X_ap_simulink)';
% TODO: - test mit anderen solvern -- SNOPT läuft nicht
% TODO: - unterlagerung mit entkopplungsregelung??
% TODO: - test mit Matheus regler als startwert
% %% Settings
% C_tilde = zeros(size(C,1), size(A,1));
% C_tilde(1:4,:) = C(1:4,:);
% C_tilde(5:8,:) = C(1:4,:) - C(5:8,:);
% 
% X_init = [150 0 0 0 0 0 0 0 5000 150 0 0 0 0 0 0 0 5010]';

%% Riccatti als Startwert
Q = 0.00001*eye(n,n);
% Q(3,3) = 1; 
% Q(12,12) = 1; 
% Q(4,4) = 100; 
% Q(6,6) = 100; 
% Q(13,13) = 100; 
% Q(15,15) = 100;
% Q(7,7) = 100; 
% Q(16,16) = 100; 
% 
R = eye(size(B,2), size(B,2));
R(3,3) = 0.00001;
R(4,4) = 0.001;
R(7,7) = 0.00001;
R(8,8) = 0.001;

% eig_1 = eig(A1);
% eig_2 = eig(A2);
% eig_1(8) = -0.01;
% eig_1 = 0.2*real(eig_1);
% eig_2(8) = -0.01;
% eig_2 = 0.2*real(eig_2);
% 
% K_1 = place(A1, B1, eig_1);
% K_2 = place(A2, B2, eig_2);
% K = zeros(8,18);
% K(1:4, 1:9) = K_1;
% K(5:8, 10:18) = K_2;

K = lqr(ss(A, B, C, 0),Q,R);
K((abs(K)<1e-11)) = 0;
Ak = A-B*K;
eigenvalues_controlled = eig(Ak);
F = -inv(C*(Ak\B));
F((abs(F)<1e-9)) = 0;
sys_ricatti = ss(Ak, B*F, C, 0);

% %% Verkopplungsregler scratch als Startwert
% P = ones(8,n);
% l = 4; %coupling conditions
% % P = fminsearch('cost_condition_number',P,optimset('TolX',1e-10,'MaxFunEvals',10000,'MaxIter',10000));
% [K_coupling_scratch, F_coupling_scratch] = coupling_control_scratch(ss(A,B,C,0),C_tilde,eigenvalues_controlled,l,P);
% % F_coupling_scratch = F_coupling_scratch(:,1:4);
% sys_coupling_scratch = ss(A-B*K_coupling_scratch, B*F_coupling_scratch, C_tilde, 0);
% step(sys_coupling_scratch);
  
%% Werte für gammasyn 
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
		'number_controls',				size(B,2),...
		'number_references',			number_references,...
		'number_models',				1,...
        'tf_structure',                 [[[[NaN 0; 0 NaN] zeros(2,2); zeros(2,2) [NaN NaN; NaN NaN]] NaN*ones(4,4)];[zeros(4,4) NaN*ones(4,4)]],...        
        'RKF_0',						{RKF_0},...
        'R_bounds',                     {bounds},...
        'R_fixed',                      []...
	);
%         'tf_structure',                 [[[[NaN 0; 0 NaN] zeros(2,2); zeros(2,2) NaN*ones(2,2)] NaN*ones(4,4)];[zeros(4,4) NaN*ones(4,4)]],...        

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
% control_design_type = GammaDecouplingStrategy.MERIT_FUNCTION;

%% pole area parameters
a = 0.3;
b = 0.2;
r = 100;

%% Pole area
weight = [1];
% polearea = control.design.gamma.area.Hyperbola(a, b);
polearea = [control.design.gamma.area.Circle(r), control.design.gamma.area.Hyperbola(a, b)];
% polearea = control.design.gamma.area.Imag(1,a);
% polearea = [control.design.gamma.area.Hyperbola(a, b), control.design.gamma.area.Imag(1,a)];
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
	'SpecifyConstraintGradient',	true,...
    'SpecifyObjectiveHessian',      true,...
    'SpecifyConstraintHessian',     true,...
	'CheckGradients',				false,...
	'FunValCheck',					false,...
	'FiniteDifferenceType',			'forward',...
	'Diagnostics',					false,...
	'Display',						'iter-detailed',...
	'UseParallel',					false...
);
objectiveoptions = struct(...
	'usecompiled',				true,...											% indicator, if compiled functions should be used
	'type',						[],...								% type of pole area weighting in objective function
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
% modifiziertes Vorfilter
Ak_coupling = A-B*K_coupling;
F1 = F_coupling(:,1:4);
C_tilde_1 = C_tilde(1:4,:);
Q_mod = -inv(C_tilde_1*(Ak_coupling\(B*F1)));
F_mod = F1*Q_mod;

information
sys_riccati = ss(A-B*K, B*F, C, 0);
sys_coupling = ss(A-B*K_coupling, B*F_mod, C_tilde, 0, ...
          'StateName',{'u1';'v1';'w1';'p1';'q1';'r1';'phi1';'theta1';'h1';...
          'u2';'v2';'w2';'p2';'q2';'r2';'phi2';'theta2';'h2'}, ...
          'InputName',{'w_{v1}';'w_{\Phi1}';'w_{\Theta1}';'w_{h1}'}, ...
          'OutputName',{'v1','\Phi_1','\Theta_1','h1',...
          'deltav','deltaphi','deltatheta','deltah'}, ...
          'Name','VERKOPPELTES SYSTEM');
sys_uncontrolled = ss(A,B,C_tilde,0);
% pzmap(sys_uncontrolled);
step(sys_coupling);
figure;
pzmap(sys_coupling);

%% einzelne Systeme
sys_coupling_1 = ss(sys_coupling.a,sys_coupling.b(:,1),sys_coupling.c(1,:),sys_coupling.d(1,1));
sys_coupling_2 = ss(sys_coupling.a,sys_coupling.b(:,2),sys_coupling.c(2,:),sys_coupling.d(2,2));
sys_coupling_3 = ss(sys_coupling.a,sys_coupling.b(:,3),sys_coupling.c(3,:),sys_coupling.d(3,3));
sys_coupling_4 = ss(sys_coupling.a,sys_coupling.b(:,4),sys_coupling.c(4,:),sys_coupling.d(4,4));

%% PID-Regler
% Auslegung der Regler für ein K_coupling mit a = 0.3; b = 0.2; r = 100;
% Circle und Hyperbola, und tf_structure in der sich theta und h
% beeinflussen, sodass die Eigenwerte des äußeren geschlossenen
% Reglekreises denen des innere entsprechen
%PI-Regler für G11
Z11 = 1.4266*[1/1.4266 1];
N11 = [1 0];
G_PI_11 = tf(Z11,N11);

%PI-Regler für G22
Z22 = 0.63*[1/(7.847*0.6966) 1/(7.847)+1/(0.6966) 1];
N22 = [1/8.5436 1 0];
G_PI_22 = tf(Z22,N22);

%PIDT1-Regler für G33
Z33 = 0.161*[1.3877 7.1273 13.7194 6.7500 1.0000];
N33 = [17.2476 22.0200 8.6033 1.0000 0];
G_PIDT1_33 = tf(Z33,N33); 

%PIDT1-Regler für G44
Z44 = 0.16104*[1.3877 7.1273 13.7194 6.7500 1.0000];
N44 = [0.1299 1.1121 1.9754 1.0000 0];
G_PIDT1_44 = tf(Z44,N44); 

% Auslegung der Regler für ein K_coupling mit a = 0.3; b = 0.2; r = 100;
% Circle und Hyperbola, und tf_structure in der sich theta und h
% beeinflussen
%PI-Regler für G11
% Z11 = 1.342*[0.7 1];
% N11 = [1 0];
% G_PI_11 = tf(Z11,N11);
% 
% %PI-Regler für G22
% Z22 = 1.9618*[1.4 1];
% N22 = [1 0];
% G_PI_22 = tf(Z22,N22);
% 
% %PIDT1-Regler für G33
% Z33 = 0.2041*[0.38^2 0.65 1];
% N33 = [2 1 0];
% G_PIDT1_33 = tf(Z33,N33); 
% 
% %PIDT1-Regler für G44
% Z44 = 0.4428*[3.1^2 6.1 1];
% N44 = [0.16 1 0];
% G_PIDT1_44 = tf(Z44,N44); 

% Auslegung der Regler für ein K_coupling mit a = 0.3; b = 0.3; r = 100;
% Circle und Hyperbola, und tf_structure in der h --> theta aber nicht
% theta --> h beeinflusst
%PI-Regler für G11
% Z11 = 0.77739*[1.2 1];
% N11 = [1 0];
% G_PI_11 = tf(Z11,N11);
% 
% %PI-Regler für G22
% Z22 = 0.5*[0.182 1.53 1];
% N22 = [0.5 1 0];
% G_PI_22 = tf(Z22,N22);
% 
% %PIDT1-Regler für G33
% Z33 = 0.25046*[1.6^2 3 1];
% N33 = [1 1 0];
% G_PIDT1_33 = tf(Z33,N33); 
% 
% %PIDT1-Regler für G44
% Z44 = 0.12501*[2.1^2 4.2 1];
% N44 = [0.2 1 0];
% G_PIDT1_44 = tf(Z44,N44); 


