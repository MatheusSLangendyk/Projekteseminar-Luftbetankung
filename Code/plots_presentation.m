load('.\Messungen\step_v.mat')
step_v = out;
clear out;
load('.\Messungen\step_phi.mat')
step_phi = out;
clear out;
load('.\Messungen\step_theta.mat')
step_theta = out;
clear out;
load('.\Messungen\step_theta_ohne_stellgrbesch.mat')
step_theta_ohne_stell = out;
clear out;
load('.\Messungen\step_h.mat')
step_h = out;
clear out;
X_AP = [150;0;-11.0225402979539;0;0;0;0;-0.0733517623176560;0;5000;150;0;-11.0065445940443;0;0;0;0;-0.0732456962006296;0;5010];
U_AP = [-0.0870457297521801;0.247747504909435;0;0;-0.0871233228705516;0.247511353508586;0;0];
theta_1_ap = X_AP(8);
h_1_ap = X_AP(10);
delta_theta_ap = X_AP(8)-X_AP(18);
delta_h_ap = X_AP(10)-X_AP(20);
eta_1_ap = U_AP(1);
eta_2_ap = U_AP(5);
xi_1_ap = U_AP(3);
xi_2_ap = U_AP(7);
zita_1_ap = U_AP(4);
zita_2_ap = U_AP(8);
sigmaf_1_ap = U_AP(2);
sigmaf_2_ap = U_AP(6);

%% y1 um AP nur lineares Modell
f_step_lin = figure;
f_step_lin.Renderer = 'painters';
subplot(4,4,1)
plot(step_v.step_linear.Time, step_v.step_linear.Data(:,1), ':k', ...
    step_v.outputs_linear.Time, step_v.outputs_linear.Data(:,1), '--r')
legend('step', 'linear', 'Location', 'southeast')
ylim([0 1.2]);
ylabel('to v1 [m/s]');
title('\rm from w_{v1}')
grid on;
subplot(4,4,5)
plot(step_v.outputs_linear.Time, step_v.outputs_linear.Data(:,2), '--r');
ylim(1e-3*[-0.5 0.5]);
ylabel('to \Phi1 [rad]');
grid on;
subplot(4,4,9)
plot(step_v.outputs_linear.Time, step_v.outputs_linear.Data(:,3)-theta_1_ap, '--r');
ylim(1e-3*[-0.5 0.5]);
ylabel('to \Theta1 [rad]');
grid on;
subplot(4,4,13)
plot(step_v.outputs_linear.Time, step_v.outputs_linear.Data(:,4)-h_1_ap, '--r');
ylim(1e-3*[-0.5 0.5]);
ylabel('to h1 [h]');
xlabel('Time [s]');
grid on;

subplot(4,4,2)
plot(step_phi.outputs_linear.Time, step_phi.outputs_linear.Data(:,1), '--r');
ylim(1e-3*[-0.5 0.5]);
title('\rm from w_{\Phi1}')
grid on;
subplot(4,4,6)
plot(step_phi.step_linear.Time, step_phi.step_linear.Data(:,2), ':k', ...
    step_phi.outputs_linear.Time, step_phi.outputs_linear.Data(:,2), '--r');
ylim([-0.05 0.3]);
grid on;
subplot(4,4,10)
plot(step_phi.outputs_linear.Time, step_phi.outputs_linear.Data(:,3)-theta_1_ap, '--r');
ylim(1e-3*[-0.5 0.5]);
grid on;
subplot(4,4,14)
plot(step_phi.outputs_linear.Time, step_phi.outputs_linear.Data(:,4)-h_1_ap, '--r');
ylim(1e-3*[-0.5 0.5]);
xlabel('Time [s]');
grid on;

subplot(4,4,3)
plot(step_theta_ohne_stell.outputs_linear.Time, step_theta_ohne_stell.outputs_linear.Data(:,1), '--r');
ylim(1e-3*[-0.5 0.5]);
title('\rm from w_{\Theta1}')
grid on;
subplot(4,4,7)
plot(step_theta_ohne_stell.outputs_linear.Time, step_theta_ohne_stell.outputs_linear.Data(:,2), '--r');
ylim(1e-3*[-0.5 0.5]);
grid on;
subplot(4,4,11)
plot(step_theta_ohne_stell.step_linear.Time, step_theta_ohne_stell.step_linear.Data(:,3), ':k', ...
    step_theta_ohne_stell.outputs_linear.Time, step_theta_ohne_stell.outputs_linear.Data(:,3)-theta_1_ap, '--r');
ylim([-0.05 0.3]);
grid on;
subplot(4,4,15)
plot(step_theta_ohne_stell.outputs_linear.Time, step_theta_ohne_stell.outputs_linear.Data(:,4)-h_1_ap, '--r');
ylim([-20 5]);
xlabel('Time [s]');
grid on;

subplot(4,4,4)
plot(step_h.outputs_linear.Time, step_h.outputs_linear.Data(:,1), '--r');
ylim(1e-3*[-0.5 0.5]);
title('\rm from w_{h1}')
grid on;
subplot(4,4,8)
plot(step_h.outputs_linear.Time, step_h.outputs_linear.Data(:,2), '--r');
ylim(1e-3*[-0.5 0.5]);
grid on;
subplot(4,4,12)
plot(step_h.outputs_linear.Time, step_h.outputs_linear.Data(:,3)-theta_1_ap, '--r');
ylim(1e-1*[-0.2 0.2]);
grid on;
subplot(4,4,16)
plot(step_h.step_linear.Time, step_h.step_linear.Data(:,4), ':k', ...
    step_h.outputs_linear.Time, step_h.outputs_linear.Data(:,4)-h_1_ap, '--r');
ylim([-2 10]);
xlabel('Time [s]');
grid on;

set(gcf, 'Position',[383 42 850 960/1.5]);

%% delta y um AP nur lineares Modell
f_deltay_lin = figure;
f_deltay_lin.Renderer = 'painters';

subplot(4,4,1)
plot(step_v.outputs_linear.Time, step_v.outputs_linear.Data(:,5), '--r');
ylim(1e-3*[-0.5 0.5]);
ylabel('to \Delta v [m/s]');
title('\rm from w_{v1}')
grid on;
subplot(4,4,5)
plot(step_v.outputs_linear.Time, step_v.outputs_linear.Data(:,6), '--r');
ylim(1e-3*[-0.5 0.5]);
ylabel('to \Delta \Phi [rad]');
grid on;
subplot(4,4,9)
plot(step_v.outputs_linear.Time, step_v.outputs_linear.Data(:,7)-delta_theta_ap, '--r');
ylim(1e-3*[-0.5 0.5]);
ylabel('to \Delta \Theta [rad]');
grid on;
subplot(4,4,13)
plot(step_v.outputs_linear.Time, step_v.outputs_linear.Data(:,8)-delta_h_ap, '--r');
ylim(1e-3*[-0.5 0.5]);
ylabel('to \Delta h [m]');
xlabel('Time [s]');
grid on;

subplot(4,4,2)
plot(step_phi.outputs_linear.Time, step_phi.outputs_linear.Data(:,5), '--r');
ylim(1e-3*[-0.5 0.5]);
title('\rm from w_{\Phi1}')
grid on;
subplot(4,4,6)
plot(step_phi.outputs_linear.Time, step_phi.outputs_linear.Data(:,6), '--r');
ylim(1e-3*[-0.5 0.5]);
grid on;
subplot(4,4,10)
plot(step_phi.outputs_linear.Time, step_phi.outputs_linear.Data(:,7)-delta_theta_ap, '--r');
ylim(1e-3*[-0.5 0.5]);
grid on;
subplot(4,4,14)
plot(step_phi.outputs_linear.Time, step_phi.outputs_linear.Data(:,8)-delta_h_ap, '--r');
ylim(1e-3*[-0.5 0.5]);
xlabel('Time [s]');
grid on;

subplot(4,4,3)
plot(step_theta_ohne_stell.outputs_linear.Time, step_theta_ohne_stell.outputs_linear.Data(:,5), '--r');
ylim(1e-3*[-0.5 0.5]);
title('\rm from w_{\Theta1}')
grid on;
subplot(4,4,7)
plot(step_theta_ohne_stell.outputs_linear.Time, step_theta_ohne_stell.outputs_linear.Data(:,6), '--r');
ylim(1e-3*[-0.5 0.5]);
grid on;
subplot(4,4,11)
plot(step_theta_ohne_stell.outputs_linear.Time, step_theta_ohne_stell.outputs_linear.Data(:,7)-delta_theta_ap, '--r');
ylim(1e-3*[-0.5 0.5]);
grid on;
subplot(4,4,15)
plot(step_theta_ohne_stell.outputs_linear.Time, step_theta_ohne_stell.outputs_linear.Data(:,8)-delta_h_ap, '--r');
ylim(1e-3*[-0.5 0.5]);
grid on;
xlabel('Time [s]');

subplot(4,4,4)
plot(step_h.outputs_linear.Time, step_h.outputs_linear.Data(:,5), '--r');
ylim(1e-3*[-0.5 0.5]);
title('\rm from w_{h1}')
grid on;
subplot(4,4,8)
plot(step_h.outputs_linear.Time, step_h.outputs_linear.Data(:,6), '--r');
ylim(1e-3*[-0.5 0.5]);
grid on;
subplot(4,4,12)
plot(step_h.outputs_linear.Time, step_h.outputs_linear.Data(:,7)-delta_theta_ap, '--r');
ylim(1e-3*[-0.5 0.5]);
grid on;
subplot(4,4,16)
plot(step_h.outputs_linear.Time, step_h.outputs_linear.Data(:,8)-delta_h_ap, '--r');
ylim(1e-3*[-0.5 0.5]);
xlabel('Time [s]');
grid on;
set(gcf, 'Position',[383 42 850 960/1.5]);
