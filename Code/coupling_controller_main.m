clear; close all;
% den Ordner gammasyn in das Code Verzeichnis hinzufügen oder den Pfad für
% gammasyn nachfolgend entsprechend anpassen
addpath('gammasyn');  
startup;
n_model = 'single'; % 'single' für ein Modell für die Optimierung, 
% 'multi' für Multimodellansatz und mehrere Systeme für die Optimierung

%% Multimodell oder single Modell
switch n_model
    % Run für Multi-Modell-Ansatz
    case 'multi'
        [systems, AP, X_init] = create_multi_model('multi');
        % Auslesen der Systemmatrizen für die Auslegung des Ricattirelgers für
        % das nominelle Modell (systems(1,1))
        A = systems(1,1).A;
        B = systems(1,1).B;
        n = size(A,1);
        C_tilde = systems(1,1).C_ref;
        C = zeros(size(B,2),n);
        C(1:4,1:n/2) = C_tilde(1:4,1:n/2);
        C(5:8,n/2+1:n) = C_tilde(1:4,1:n/2);
        X_ap = AP(1,1).X_ap;
        X_ap_simulink = AP(1,1).X_ap_simulink;
        U_ap = AP(1,1).U_ap;
        X_init = X_ap_simulink;
        deltaX_init = X_init-X_ap_simulink;

        % Saturations
        eta_max = 10*pi/180; %Elevator
        eta_min = - 25*pi/180; 
        sigmaf_max = 20*pi/180; %Throttl
        sigmaf_min = 0.5*pi/180;
        xi_max = 25*pi/180; %Airlon
        xi_min = - xi_max;
        zita_max = 30*pi/180; %Rudder
        zita_min = - zita_max;

    % Run für single Modell 
    case 'single'
        % Get Model Parameters for Simulink
        [globalParameters,m,g,he,I_inv] = initializeParameters();
        SixDOFModel;
        X_init = X_ap_simulink;
        deltaX_init = X_init-X_ap_simulink;
end

%% Ricatti als Startwert
Q = eye(n,n);
Q = Q/100000;

R = eye(size(B,2), size(B,2));
R(3,3) = 0.00001;
R(4,4) = 0.001;
R(7,7) = 0.00001;
R(8,8) = 0.001;

K = lqr(ss(A, B, C, 0),Q,R);
K((abs(K)<1e-11)) = 0;
Ak = A-B*K;
eigenvalues_controlled = eig(Ak);
F = -inv(C*(Ak\B));
F((abs(F)<1e-9)) = 0;
  
%% Werte für gammasyn 
K_0 = [];
RKF_0 = {K, K_0, F};
bounds = {[] [] []};
fix = zeros(size(K));
fix_val = NaN*ones(size(fix));
R_fixed = {fix, fix_val};
fixed = {R_fixed, [], []};
number_references = size(C_tilde, 1);
switch n_model
    case 'single'
        systems = struct('E', eye(n, n), 'A', A, 'B', B, 'C', eye(size(A,1)), 'C_ref', C_tilde);
    case 'multi'
        systems;
end
system_properties = struct(...
		'number_states',				n,...
		'number_controls',				size(B,2),...
		'number_references',			number_references,...
		'number_models',				length(systems),...
        'tf_structure',                 [[[[NaN 0; 0 NaN] zeros(2,2); zeros(2,2) NaN*ones(2,2)] NaN*ones(4,4)];[zeros(4,4) NaN*ones(4,4)]],...        
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
polearea = {repmat([
	control.design.gamma.area.Circle(r), control.design.gamma.area.Hyperbola(a, b)],...
    system_properties.number_models, 1)};
polearea = polearea{1};

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
[Kopt, Jopt, information] = control.design.gamma.gammasyn_decouplingcontrol(systems, polearea, weight, system_properties.R_fixed, system_properties.RKF_0, options, objectiveoptions, system_properties.R_bounds);
if isempty(Kopt)
	return;
end

K_coupling = Kopt{1} 
F_coupling = Kopt{end}

% Eigenwerte geregelte Systeme
for i = 1:length(systems)
    eigenvalues(:,i) = eig(systems(i,1).A-systems(i,1).B*K_coupling);
end

% modifiziertes Vorfilter und Sprungantworten
F_mod_all = {};
f1 = figure;
f2 = figure;
for i = 1:length(systems)
    Ak_coupling = systems(i,1).A-systems(i,1).B*K_coupling;
    F1 = F_coupling(:,1:4);
    C_tilde_1 = systems(i,1).C_ref(1:4,:);
    Q_mod = -inv(C_tilde_1*(Ak_coupling\(systems(i,1).B*F1)));
    F_mod = F1*Q_mod;
    F_mod_all(i,1).F_mod = F_mod;
    sys_coupling = ss(systems(i,1).A-systems(i,1).B*K_coupling, systems(i,1).B*F_mod, systems(i,1).C_ref, 0, ...
          'StateName',{'u1';'v1';'w1';'p1';'q1';'r1';'phi1';'theta1';'h1';...
          'u2';'v2';'w2';'p2';'q2';'r2';'phi2';'theta2';'h2'}, ...
          'InputName',{'w_{v1}';'w_{\Phi1}';'w_{\Theta1}';'w_{h1}'}, ...
          'OutputName',{'v1','\Phi1','\Theta1','h1',...
          '\Deltav','\Delta\Phi','\Delta\Theta','\Deltah'}, ...
          'Name','VERKOPPELTES SYSTEM');
     figure(f1);
     opt = stepDataOptions('StepAmplitude',[1 0.2 0.2 1]);
     step(sys_coupling, opt);
     hold on;
     figure(f2);
     pzmap(sys_coupling);
     hold on;
end
information

%% unterlagerte Regelung

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
