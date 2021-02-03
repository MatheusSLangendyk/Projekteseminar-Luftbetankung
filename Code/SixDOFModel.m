%Simualtion of a complete 6 Degrees of Freedom Model
clc
clear
close all

system_norm = false;
deltah_offset = 10;

%% Get Model Parameters 
[globalParameters,m,g,he,I_inv] = initializeParameters();

%% Initial Values both planes
vA_init_1 = 150;
alpha_init_1 = 0;
beta_init_1 = 0;
Omega_init_1 = [0;0;0];
Phi_init_1 = [0;0;0];
h_init_1 = 5000;
P_e_init_1 = [0;0;-h_init_1];
latlon_init = [0;0];
X_init_1 = [vA_init_1;alpha_init_1;beta_init_1;Omega_init_1;Phi_init_1;h_init_1];

%plane 2
vA_init_2 = 150;
alpha_init_2 = 0;
beta_init_2 = 0;
Omega_init_2 = [0;0;0];
Phi_init_2 = [0;0;0];
h_init_2 = h_init_1 + deltah_offset;
P_e_init_2 = [0;0;-h_init_2];
X_init_2 = [vA_init_2;alpha_init_2;beta_init_2;Omega_init_2;Phi_init_2;h_init_2];

%% calculate trim points
ap_solver = 0;
if ap_solver == 0
    % AP mit fsolve
    [X_ap_1, U_ap_1] = fsolve_trim([X_init_1;zeros(4,1)]); % mit [vA, phi, psi, h] = [150, 0, 0, 5000]
    [X_ap_2, U_ap_2] = fsolve_trim([X_init_2;zeros(4,1)]); % mit [vA, phi, psi, h] = [150, 0, 0, 5010]
elseif ap_solver == 1
    % AP mit trimValues
    [X_ap_1,U_ap_1,f0_1] = trimValues(vA_init_1,alpha_init_1,beta_init_1,Omega_init_1,Phi_init_1,h_init_1,1);
    [X_ap_2,U_ap_2,f0_2] = trimValues(vA_init_2,alpha_init_2,beta_init_2,Omega_init_2,Phi_init_2,h_init_2,2);
end
% x_ap_comp = [[X_ap_1;U_ap_1] [X_ap_1_t;U_ap_1_t]];
X_ap = [X_ap_1;X_ap_2];
U_ap = [U_ap_1;U_ap_2];
U_ap((abs(U_ap)<1e-9)) = 0;
X_ap((abs(X_ap)<1e-9)) = 0;
X_ap_simulink = [X_ap(1:8);X_ap(10:18);X_ap(20)];
X_init_all = [X_init_1;X_init_2];
X_init = [X_init_all(1:8);X_init_all(10:18);X_init_all(20)];
W_linear_simulink = [10;0;0;50];

%% DGL Flugzeug 1
% symbolische nicht-lineare DGL mit psi
plane_selector = 1;
assignin('base','plane_selector',plane_selector)
symbolic_equations;
% Zustands-DGL ohne psi
f = [dvA;dalpha;dbeta;dp;dq;dr;dphi;dtheta;dh];

% Ausgangsgleichung
h = [vA beta phi h];

% 1. Linearisierung durch bilden der Jacoby-Matrizen
A_sym = jacobian(f, x_red_9);   % A = d f(x,u) / dx
B_sym = jacobian(f, u_stell);   % B = d f(x,u) / du
C_sym = jacobian(h, x_red_9);   % C = d y(x,u) / dx
D_sym = jacobian(h, u_stell);   % D = d y(x,u) / du

% 2. Einsetzten der Anfangswerte
A1 = double(subs(A_sym, [x10,u_stell], [X_ap_1; U_ap_1]'));
B1 = double(subs(B_sym, [x10,u_stell], [X_ap_1; U_ap_1]'));
C1 = double(subs(C_sym, [x10,u_stell], [X_ap_1; U_ap_1]'));
D1 = double(subs(D_sym, [x10,u_stell], [X_ap_1; U_ap_1]'));

%% DGL Flugzeug 2
% symbolische nicht-lineare DGL mit psi
plane_selector = 2;
assignin('base','plane_selector',plane_selector)
symbolic_equations;
% Zustands-DGL ohne psi
f = [dvA;dalpha;dbeta;dp;dq;dr;dphi;dtheta;dh];

% Ausgangsgleichung
h = [vA beta phi h];

% 1. Linearisierung durch bilden der Jacoby-Matrizen
A_sym = jacobian(f, x_red_9);   % A = d f(x,u) / dx
B_sym = jacobian(f, u_stell);   % B = d f(x,u) / du
C_sym = jacobian(h, x_red_9);   % C = d y(x,u) / dx
D_sym = jacobian(h, u_stell);   % D = d y(x,u) / du

A2 = double(subs(A_sym, [x10,u_stell], [X_ap_2; U_ap_2]'));
B2 = double(subs(B_sym, [x10,u_stell], [X_ap_2; U_ap_2]'));
C2 = double(subs(C_sym, [x10,u_stell], [X_ap_2; U_ap_2]'));
D2 = double(subs(D_sym, [x10,u_stell], [X_ap_2; U_ap_2]'));

%% System ein Flugzeug
% % Steuerbarkeit des einzelnen Flugzeugs
% rank(ctrb(A1,B1));
% % Beobachtbarkeit des einzelnen Flugzeugs
% rank(obsv(A1,C1));
% 
% sys1 = ss(A1, B1, C1, D1);
% K = lqr(sys1, eye(9), eye(4));
% sys1_cl = ss(A1-B1*K, B1, C1, D1);
% 
% anz_io = size(C1,1);
% delta_k = zeros(1, anz_io);
% 
% % Berechnung der Differenzordnungen der Ausgänge
% % Nutzen Sie dabei eine for-Schleife, um alle Ausgänge zu durchlaufen
% % und eine while-Schleife zur Berechnung der jeweiligen Matrizen ci*(A^j)*B
% for k = 1:anz_io     % alle Ausgänge nacheinander
%   j=1;
%   while C1(k,:)*A1^(j-1)*B1 == zeros(1,anz_io)
%     j=j+1;
%   end
%   delta_k(k)=j;      % Differenzordnung des Ausgangs k
% end
% 
% % Differenzordnung des Gesamtsystems
% delta = sum(delta_k);

%% Zwei Flugzeug Modell
[A,B,C,n] = defineABC(A1,A2,B1,B2);
W_ap = C*X_ap_simulink;
% Steuerbarkeit
rank(ctrb(A,B));
% Beobachtbarkeit 
rank(obsv(A,C));

sys_2plane = ss(A, B, C, zeros(8,8));

anz_io = size(C,1);
delta_k = zeros(1, anz_io);

% Berechnung der Differenzordnungen der Ausgänge
% Nutzen Sie dabei eine for-Schleife, um alle Ausgänge zu durchlaufen
% und eine while-Schleife zur Berechnung der jeweiligen Matrizen ci*(A^j)*B
for k = 1:anz_io     % alle Ausgänge nacheinander
  j=1;
  while C(k,:)*A^(j-1)*B == zeros(1,anz_io)
    j=j+1;
  end
  delta_k(k)=j;      % Differenzordnung des Ausgangs k
end

% Differenzordnung des Gesamtsystems
delta = sum(delta_k);

%Saturations
eta_max = 10*pi/180; %Elevator
eta_min = - 25*pi/180; 
sigmaf_max = 20*pi/180; %Throttl
sigmaf_min = 0.5*pi/180;
xi_max = 25*pi/180; %Airlon
xi_min = - xi_max;
zita_max = 30*pi/180; %Rudder
zita_min = - zita_max;
% %Contolable/ Obsarvable
Ms = ctrb(A,B);
Mb = obsv(A,C);
if rank(Ms) ~= min(size(Ms,1),size(Ms,2))
    disp('System is not Kalman Controllable')
else
    disp('System is Kalman Controllable')
end
if rank(Mb) ~= min(size(Mb,1),size(Mb,2))
    disp('System is not Kalman Obsarvable')
else
    disp('System is Kalman Obsarvable')
end

eigenvalues = eig(A);

%Hautus
for i = 1:n
    eig_i = eigenvalues(i);
    if rank([eig_i*eye(n,n)-A;C]) ~=n
        
        disp(['Eigenvalue ',num2str(eig_i),' is not obsarvable ']);
    end
end
%% Normieren
 if system_norm == true
    [A,B,C] = normieren(A,B,C,eta_max,sigmaf_max,xi_max,zita_max);
 end
  
  %% Transfer Function Open Loop
   sys_ol = ss(A,B, C,zeros(8,8));
  
  %%Riccatti
  Q = eye(n,n);
  Q(1,1) = 1;
  Q(7,7) = 1;
  Q(8,8) = 1;
  Q(10,10) = 1;
  Q(16,16) =1;
  Q(17,17) = 1;
  
  R = eye(8,8);
  R(1,1) = 3;
  R(2,2) = 15;
  R(5,5) = 3;
  R(6,6) = 15;
  K = lqr(sys_ol,Q,R);
  K((abs(K)<10^-9)) = 0;
  Ak = A -B*K;
  F = -inv(C*(Ak\B));
  ew_ricati = eig(Ak);
  sys_ricati = ss(Ak,B*F,C,zeros(8,8));
  %step(sys_ricati)
%   Gamma = getgamma(A,B,C);
%   gamma_sum = sum(Gamma);
  ew_strecke = pole(sys_ol);
  
  %% Coupling Control (manual) 
  C_tilde = zeros(8,n);
  C_tilde(1,1) = 1; %vA
  C_tilde(2,3) = 1; %beta
  C_tilde(3,7) = 1; % phi
  C_tilde(4,9) = 1;%höhe
  C_tilde(5,1) = 1;
  C_tilde(5,10) = -1;
  C_tilde(6,3) = 1;
  C_tilde(6,12) = -1;
  C_tilde(7,7) = 1;
  C_tilde(7,16) = -1;
  C_tilde(8,9) = 1;
  C_tilde(8,18) = -1;
  l = 4; %coupling conditions
  opt_modus = 2; %0-conditional number 1- Values o K_coupling 2-control variable
  t = 0:0.1:20;
  W = zeros(size(C,1),length(t)); %Erzeugung Testführungsgröße
  for i = 1:length(t)
    if t(i)>= 10
        W(1:l,i) = W_linear_simulink;
    end
  end
  C1_tilde = C_tilde(1:l,:);
  C2_tilde = C_tilde(l+1:end,1:end);
  struct_cond.sys_ol = sys_ol ;
  struct_cond.C_tilde = C_tilde;
  struct_cond.ew_ricati = ew_ricati;
  struct_cond.l =l;
  struct_cond.opt_modus = opt_modus;
  struct_cond.X_ap = X_ap_simulink;
  struct_cond.W = W;
  struct_cond.t = t;
  assignin('base','struct_cond',struct_cond)
  m = 9; %Erfahrungswert
  FM = [-2.86064019651863,0.332405211765897,-0.0602917280558532,-0.0617805881064860;0.766558012117925,0.553216489267562,4.64418121686554,1.86524676077734;-0.00472974148553560,-1.00920705239542,-0.0998973467852818,-0.0113927345820094;-2.21880926456817,0.758138678089461,-0.720021293241349,2.82651684572960;0.000514407872634988,2.77829547874530,1.98688547498463,0.671836654483795;0.594678023819006,0.0476361765571765,3.87574729274094,3.84380463919648;-5.74306822335620,-0.106717506098773,-1.60139950602460,-0.209044870732595;1.80263054764665,-7.16601419920122,-0.672379611543736,0.650587693856959;-0.484095028279146,-0.103800999994330,-0.387132993284330,1.13495884954856];
  P = [2.26519543357233,0.299638542702110,-1.52196836203740,-0.614352473622596,-0.386832641866730,-1.46890810350677,-0.946450610335802,-1.30750261602475,0.846500664558696,2.76812303334222,0.113387383320868,-1.81723551410034,-0.764409438691380,1.17539509969254,0.458167304241593,1.44185529943168,1.69247814529552,-0.115215488305958;-0.0478903304637657,-0.197218455541783,0.621011307663875,0.241695211355152,0.421615541369374,0.176938356086965,-0.438499143610695,0.794142715421655,0.110154836255540,-0.495347149392365,-0.229807147664015,-0.521730614435447,0.410137543311993,0.397740426136556,-0.264144136031506,0.235526729771707,0.475728558388262,1.82200005530299;-1.55186406853224,-0.146422432942845,-1.50752258512374,0.549349147991792,1.08770585526875,3.46625991439170,0.343220672265526,-0.197263789445607,-1.16110152841623,0.468765524421863,-1.46173284001932,0.161435878047334,-0.789932749060492,1.95124871677449,-0.291985147351653,1.02137655041072,0.368530218417696,-2.02150665806026;0.444083312031626,-0.103070869778914,-1.67940213835796,0.467634317562715,-2.24933189844400,-0.214626724445032,-0.0584033819465111,0.649152816254485,-0.397535702561930,-0.657296887156008,-2.88233613993966,-1.06181087486791,0.161637129369329,-0.904656364205178,-0.386798143591353,-0.139830532942445,2.19200072423306,-0.636977460723231;-0.911818876809533,-2.79898194583242,0.788976200730389,0.191457003009039,1.80448013956929,0.486282563419722,2.53497468773507,-0.831474272159670,0.254294311040492,-1.71695724227432,-0.0474676075293698,0.450493739221112,1.97790399736349,-0.682152191566547,-0.553992932992651,-0.0815511030592480,1.53568510486071,-0.188803224160108;0.0494346475387195,0.393275176045361,-0.654282360534659,-0.229804955766629,-0.632060379431905,0.330885119922978,0.438645437760202,0.895954060001157,1.20778943607426,1.47051910410640,-0.462465950361891,-0.272798073223806,0.795266427309762,2.01829157670990,-0.943727928429579,1.25824675195836,0.907260215322908,0.148275041944675;1.07804587481937,0.990247152651875,1.24490301031691,-0.579216839923128,1.31645820719660,1.26790208092984,0.437517717998504,-1.81348642042825,-1.03354161269991,0.694137265600360,-0.576645894751125,-0.101454350734858,1.03736673316936,0.454343178361193,-0.0598481826488825,1.46853400199602,0.402259582355281,-1.78388004396413;0.308178759981126,-1.29763291067202,-1.29226441786498,0.480481177166826,1.55159205849066,1.09051420554461,-0.837675995187059,1.56668345394797,1.29514675428588,-0.510696986674642,-0.845968359502791,-1.42910594792867,2.36032996494344,-0.0421041798190231,-0.574684770106140,0.0329408560392328,-0.538382000967076,0.529067547951978];  
  P = [0.415988080616651,1.03849258185742,-1.01073115472517,-1.31481970059030,0.120384837653475,0.420029137155139,0.173121084058509,0.171396847589562,0.497769042375642,-0.781348057247054,0.536097411916586,-0.588337917341691,-0.0716097821925113,-0.378914524062899,0.277444401951355,1.39173310703490,-0.198381395750642,-0.560289062393102;0.349376297243175,-0.846141845156233,-0.212278344339006,0.0299390054131363,-0.988849764343024,0.400067085881244,-0.505141247626052,-0.0620718773168118,2.78778001870232,0.317371307254752,-1.53567188540987,-0.587905305569685,-0.934480646846289,-0.539444763683435,0.639116368857597,-0.0626974832237525,0.405396675105673,0.215124053636166;0.350313749321416,-0.172877447606138,-0.323791608562617,0.841783015293467,1.19655433547442,0.0949714655773644,-1.19227996400362,1.19845184328051,0.727201170565875,1.41440102205271,-0.202186132157768,0.851778579258681,0.162225756500345,-0.916099553355324,-0.0809323400710618,0.451304642577778,-1.42647361104207,0.939901327728260;-0.731288852919853,-1.20770241812063,1.93342346292895,0.398394291096688,-0.592042772021250,0.495757223926199,0.646306749650361,0.801350169258235,-0.772635307674583,0.402989423382010,-0.490152618276616,-1.87255560659702,-0.267258820955742,0.650070453739257,0.540468531495199,-0.361164462285152,-0.730401995313320,0.0933873583763721;0.327578664663301,-0.296741098520879,-0.568086168912471,-0.765394940253181,-0.469170594952540,1.09775099295256,-0.353194783718310,1.05283630297179,0.836130591746429,0.936594705940270,0.391761655477843,-0.208980932750443,-0.404979782620017,-0.771902394276751,-1.26861535190033,-1.02590080851419,1.15166237759788,-1.14787628514039;-0.515812919642444,-3.22608820019684,-0.248207610917150,-1.63046020456354,0.885118161385579,0.969887408332757,0.0463822530213178,-0.748544501338267,-1.12759682012419,-1.61327599666252,0.410816805586779,0.270238098165097,-0.715000420831507,0.538033161242675,1.11474485354879,-3.08556043382492,0.600064276306777,0.304814066882898;-0.897766281744310,-1.08433379542259,-1.55623217083982,1.45967844810189,-1.38338568823538,-0.568229711557669,-0.791886345572069,-0.935869705436611,-1.42361276928973,0.659909544976483,0.403904004375042,-0.651721625322274,0.0625672689296785,0.979774737578385,-0.993797284981060,0.623818506657955,-1.27889229646621,-1.16590112141059;-1.20474761237321,-1.42212341766077,-0.473180455692613,2.04850376981035,-1.95393766201862,0.809443054033074,-1.54890854446480,-1.26853937584031,0.717013070379425,2.10814778229781,-0.368920243532603,0.479691705372684,-1.88056780196154,-0.156260147067125,-1.82742930013045,-0.288421479368364,-2.22979935591542,-0.959392230272667];
  struct_cond.P = P;
  %[P,f0] = fminsearch('cost_coupling_control',P,optimset('TolX',100,'MaxFunEvals',100,'MaxIter',100)); 
  %[FM,f0] = fminsearch('cost_filter',FM,optimset('TolX',10^-5,'MaxFunEvals',2000,'MaxIter',2000)); 
  [K_coupling, F_coupling] = coupling_control_scratch2(sys_ol,C_tilde,ew_ricati,l,P,FM);
  max(max(abs(K_coupling)))
  sum(sum(abs(K_coupling)))
  sys_cl = ss(A-B*K_coupling,B*F_coupling,C_tilde,zeros(8,8));
