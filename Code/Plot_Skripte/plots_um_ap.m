load('C:\Users\Markus\Documents\Uni\WISE2021\Projektseminar\PS_Luftbetankung\PS_Luftbetankung_newRepo\Projekteseminar-Luftbetankung\Code\Messungen\step_v.mat')
step_v = out;
clear out;
load('C:\Users\Markus\Documents\Uni\WISE2021\Projektseminar\PS_Luftbetankung\PS_Luftbetankung_newRepo\Projekteseminar-Luftbetankung\Code\Messungen\step_phi.mat')
step_phi = out;
clear out;
load('C:\Users\Markus\Documents\Uni\WISE2021\Projektseminar\PS_Luftbetankung\PS_Luftbetankung_newRepo\Projekteseminar-Luftbetankung\Code\Messungen\step_theta.mat')
step_theta = out;
clear out;
load('C:\Users\Markus\Documents\Uni\WISE2021\Projektseminar\PS_Luftbetankung\PS_Luftbetankung_newRepo\Projekteseminar-Luftbetankung\Code\Messungen\step_theta_ohne_stellgrbesch.mat')
step_theta_ohne_stell = out;
clear out;
load('C:\Users\Markus\Documents\Uni\WISE2021\Projektseminar\PS_Luftbetankung\PS_Luftbetankung_newRepo\Projekteseminar-Luftbetankung\Code\Messungen\step_h.mat')
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

%% plot y1 and coupled output plus u ohne stellgrößenbeschränkung nur lineares system
f_step_lin = figure;
f_step_lin.Renderer = 'painters';
subplot(8,4,1)
plot(step_v.step_linear.Time, step_v.step_linear.Data(:,1), ':k', ...
    step_v.outputs_linear.Time, step_v.outputs_linear.Data(:,1), '--r')
legend('step', 'linear', 'Location', 'southeast')
ylim([0 1.2]);
ylabel('to v1 [m/s]');
title('\rm from w_{v1}')
grid on;
subplot(8,4,5)
plot(step_v.outputs_linear.Time, step_v.outputs_linear.Data(:,2), '--r');
ylim(1e-3*[-0.5 0.5]);
ylabel('to \Phi1 [rad]');
grid on;
subplot(8,4,9)
plot(step_v.outputs_linear.Time, step_v.outputs_linear.Data(:,3)-theta_1_ap, '--r');
ylim(1e-3*[-0.5 0.5]);
ylabel('to \Theta1 [rad]');
grid on;
subplot(8,4,13)
plot(step_v.outputs_linear.Time, step_v.outputs_linear.Data(:,4)-h_1_ap, '--r');
ylim(1e-3*[-0.5 0.5]);
ylabel('to h1 [h]');
grid on;
subplot(8,4,17)
plot(step_v.outputs_linear.Time, step_v.outputs_linear.Data(:,5), '--r');
ylim(1e-3*[-0.5 0.5]);
ylabel('to \Delta v [m/s]');
grid on;
subplot(8,4,21)
plot(step_v.outputs_linear.Time, step_v.outputs_linear.Data(:,6), '--r');
ylim(1e-3*[-0.5 0.5]);
ylabel('to \Delta \Phi [rad]');
grid on;
subplot(8,4,25)
plot(step_v.outputs_linear.Time, step_v.outputs_linear.Data(:,7)-delta_theta_ap, '--r');
ylim(1e-3*[-0.5 0.5]);
ylabel('to \Delta \Theta [rad]');
grid on;
subplot(8,4,29)
plot(step_v.outputs_linear.Time, step_v.outputs_linear.Data(:,8)-delta_h_ap, '--r');
ylim(1e-3*[-0.5 0.5]);
ylabel('to \Delta h [m]');
xlabel('Time [s]');
grid on;

subplot(8,4,2)
plot(step_phi.outputs_linear.Time, step_phi.outputs_linear.Data(:,1), '--r');
ylim(1e-3*[-0.5 0.5]);
title('\rm from w_{\Phi1}')
grid on;
subplot(8,4,6)
plot(step_phi.step_linear.Time, step_phi.step_linear.Data(:,2), ':k', ...
    step_phi.outputs_linear.Time, step_phi.outputs_linear.Data(:,2), '--r');
ylim([-0.05 0.3]);
grid on;
subplot(8,4,10)
plot(step_phi.outputs_linear.Time, step_phi.outputs_linear.Data(:,3)-theta_1_ap, '--r');
ylim(1e-3*[-0.5 0.5]);
grid on;
subplot(8,4,14)
plot(step_phi.outputs_linear.Time, step_phi.outputs_linear.Data(:,4)-h_1_ap, '--r');
ylim(1e-3*[-0.5 0.5]);
grid on;
subplot(8,4,18)
plot(step_phi.outputs_linear.Time, step_phi.outputs_linear.Data(:,5), '--r');
ylim(1e-3*[-0.5 0.5]);
grid on;
subplot(8,4,22)
plot(step_phi.outputs_linear.Time, step_phi.outputs_linear.Data(:,6), '--r');
ylim(1e-3*[-0.5 0.5]);
grid on;
subplot(8,4,26)
plot(step_phi.outputs_linear.Time, step_phi.outputs_linear.Data(:,7)-delta_theta_ap, '--r');
ylim(1e-3*[-0.5 0.5]);
grid on;
subplot(8,4,30)
plot(step_phi.outputs_linear.Time, step_phi.outputs_linear.Data(:,8)-delta_h_ap, '--r');
ylim(1e-3*[-0.5 0.5]);
xlabel('Time [s]');
grid on;

subplot(8,4,3)
plot(step_theta_ohne_stell.outputs_linear.Time, step_theta_ohne_stell.outputs_linear.Data(:,1), '--r');
ylim(1e-3*[-0.5 0.5]);
title('\rm from w_{\Theta1}')
grid on;
subplot(8,4,7)
plot(step_theta_ohne_stell.outputs_linear.Time, step_theta_ohne_stell.outputs_linear.Data(:,2), '--r');
ylim(1e-3*[-0.5 0.5]);
grid on;
subplot(8,4,11)
plot(step_theta_ohne_stell.step_linear.Time, step_theta_ohne_stell.step_linear.Data(:,3), ':k', ...
    step_theta_ohne_stell.outputs_linear.Time, step_theta_ohne_stell.outputs_linear.Data(:,3)-theta_1_ap, '--r');
ylim([-0.05 0.3]);
grid on;
subplot(8,4,15)
plot(step_theta_ohne_stell.outputs_linear.Time, step_theta_ohne_stell.outputs_linear.Data(:,4)-h_1_ap, '--r');
ylim([-20 5]);
grid on;
subplot(8,4,19)
plot(step_theta_ohne_stell.outputs_linear.Time, step_theta_ohne_stell.outputs_linear.Data(:,5), '--r');
ylim(1e-3*[-0.5 0.5]);
grid on;
subplot(8,4,23)
plot(step_theta_ohne_stell.outputs_linear.Time, step_theta_ohne_stell.outputs_linear.Data(:,6), '--r');
ylim(1e-3*[-0.5 0.5]);
grid on;
subplot(8,4,27)
plot(step_theta_ohne_stell.outputs_linear.Time, step_theta_ohne_stell.outputs_linear.Data(:,7)-delta_theta_ap, '--r');
ylim(1e-3*[-0.5 0.5]);
grid on;
subplot(8,4,31)
plot(step_theta_ohne_stell.outputs_linear.Time, step_theta_ohne_stell.outputs_linear.Data(:,8)-delta_h_ap, '--r');
ylim(1e-3*[-0.5 0.5]);
grid on;
xlabel('Time [s]');

subplot(8,4,4)
plot(step_h.outputs_linear.Time, step_h.outputs_linear.Data(:,1), '--r');
ylim(1e-3*[-0.5 0.5]);
title('\rm from w_{h1}')
grid on;
subplot(8,4,8)
plot(step_h.outputs_linear.Time, step_h.outputs_linear.Data(:,2), '--r');
ylim(1e-3*[-0.5 0.5]);
grid on;
subplot(8,4,12)
plot(step_h.outputs_linear.Time, step_h.outputs_linear.Data(:,3)-theta_1_ap, '--r');
ylim(1e-1*[-0.2 0.2]);
grid on;
subplot(8,4,16)
plot(step_h.step_linear.Time, step_h.step_linear.Data(:,4), ':k', ...
    step_h.outputs_linear.Time, step_h.outputs_linear.Data(:,4)-h_1_ap, '--r');
ylim([-2 10]);
grid on;
subplot(8,4,20)
plot(step_h.outputs_linear.Time, step_h.outputs_linear.Data(:,5), '--r');
ylim(1e-3*[-0.5 0.5]);
grid on;
subplot(8,4,24)
plot(step_h.outputs_linear.Time, step_h.outputs_linear.Data(:,6), '--r');
ylim(1e-3*[-0.5 0.5]);
grid on;
subplot(8,4,28)
plot(step_h.outputs_linear.Time, step_h.outputs_linear.Data(:,7)-delta_theta_ap, '--r');
ylim(1e-3*[-0.5 0.5]);
grid on;
subplot(8,4,32)
plot(step_h.outputs_linear.Time, step_h.outputs_linear.Data(:,8)-delta_h_ap, '--r');
ylim(1e-3*[-0.5 0.5]);
xlabel('Time [s]');
grid on;
set(gcf, 'Position',[383 42 850 960]);

f_stell_lin = figure;
f_stell_lin.Renderer = 'painters';
subplot(4,4,1)
plot(step_v.U_ist_linear.Time, step_v.U_ist_linear.Data(:,1)-eta_1_ap, '--r', ...
    step_v.U_ist_linear.Time, step_v.U_ist_linear.Data(:,5)-eta_2_ap, '-.b')
legend('linear plane1', 'linear plane2', 'Location', 'northeast')
ylim(1e-3*[-0.5 0.5]);
xlim([0 1.5]);
ylabel('to \eta [rad]');
title('\rm from w_{v1}')
grid on;
subplot(4,4,5)
plot(step_v.U_ist_linear.Time, step_v.U_ist_linear.Data(:,3)-xi_1_ap, '--r', ...
    step_v.U_ist_linear.Time, step_v.U_ist_linear.Data(:,7)-xi_2_ap, '-.b');
ylim([-0.05 0.2]);
xlim([0 1.5]);
ylabel('to \xi [rad]');
grid on;
subplot(4,4,9)
plot(step_v.U_ist_linear.Time, step_v.U_ist_linear.Data(:,4)-zita_1_ap, '--r', ...
    step_v.U_ist_linear.Time, step_v.U_ist_linear.Data(:,8)-zita_2_ap, '-.b');
ylim([-0.05 0.5]);
xlim([0 1.5]);
ylabel('to \zeta [rad]');
grid on;
subplot(4,4,13)
plot(step_v.U_ist_linear.Time, step_v.U_ist_linear.Data(:,2)-sigmaf_1_ap, '--r', ...
    step_v.U_ist_linear.Time, step_v.U_ist_linear.Data(:,6)-sigmaf_2_ap, '-.b');
ylim(1e-3*[-0.5 0.5]);
xlim([0 1.5]);
ylabel('to sigma_f');
grid on;
xlabel('Time [s]');

subplot(4,4,2)
plot(step_phi.U_ist_linear.Time, step_phi.U_ist_linear.Data(:,1)-eta_1_ap, '--r', ...
    step_phi.U_ist_linear.Time, step_phi.U_ist_linear.Data(:,5)-eta_2_ap, '-.b');
ylim(1e-3*[-0.5 0.5]);
xlim([0 1.5]);
title('\rm from w_{\Phi1}')
grid on;
subplot(4,4,6)
plot(step_phi.U_ist_linear.Time, step_phi.U_ist_linear.Data(:,3)-xi_1_ap, '--r', ...
    step_phi.U_ist_linear.Time, step_phi.U_ist_linear.Data(:,7)-xi_2_ap, '-.b');
ylim([-0.2 0.1]);
xlim([0 1.5]);
grid on;
subplot(4,4,10)
plot(step_phi.U_ist_linear.Time, step_phi.U_ist_linear.Data(:,4)-zita_1_ap, '--r', ...
    step_phi.U_ist_linear.Time, step_phi.U_ist_linear.Data(:,8)-zita_2_ap, '-.b');
ylim([-0.1 0.1]);
xlim([0 1.5]);
grid on;
subplot(4,4,14)
plot(step_phi.U_ist_linear.Time, step_phi.U_ist_linear.Data(:,2)-sigmaf_1_ap, '--r', ...
    step_phi.U_ist_linear.Time, step_phi.U_ist_linear.Data(:,6)-sigmaf_2_ap, '-.b');
ylim(1e-3*[-0.5 0.5]);
xlim([0 1.5]);
grid on;
xlabel('Time [s]');

subplot(4,4,3)
plot(step_theta_ohne_stell.U_ist_linear.Time, step_theta_ohne_stell.U_ist_linear.Data(:,1)-eta_1_ap, '--r', ...
    step_theta_ohne_stell.U_ist_linear.Time, step_theta_ohne_stell.U_ist_linear.Data(:,5)-eta_2_ap, '-.b');
ylim([-40 10]);
xlim([0 1.5]);
title('\rm from w_{\Theta1}')
grid on;
subplot(4,4,7)
plot(step_theta_ohne_stell.U_ist_linear.Time, step_theta_ohne_stell.U_ist_linear.Data(:,3)-xi_1_ap, '--r', ...
    step_theta_ohne_stell.U_ist_linear.Time, step_theta_ohne_stell.U_ist_linear.Data(:,7)-xi_2_ap, '-.b');
ylim(1e-3*[-0.5 0.5]);
xlim([0 1.5]);
grid on;
subplot(4,4,11)
plot(step_theta_ohne_stell.U_ist_linear.Time, step_theta_ohne_stell.U_ist_linear.Data(:,4)-zita_1_ap, '--r', ...
    step_theta_ohne_stell.U_ist_linear.Time, step_theta_ohne_stell.U_ist_linear.Data(:,8)-zita_2_ap, '-.b');
ylim(1e-3*[-0.5 0.5]);
xlim([0 1.5]);
grid on;
subplot(4,4,15)
plot(step_theta_ohne_stell.U_ist_linear.Time, step_theta_ohne_stell.U_ist_linear.Data(:,2)-sigmaf_1_ap, '--r', ...
    step_theta_ohne_stell.U_ist_linear.Time, step_theta_ohne_stell.U_ist_linear.Data(:,6)-sigmaf_2_ap, '-.b');
ylim([-650 50]);
xlim([0 1.5]);
grid on;
xlabel('Time [s]');

subplot(4,4,4)
plot(step_h.U_ist_linear.Time, step_h.U_ist_linear.Data(:,1)-eta_1_ap, '--r', ...
    step_h.U_ist_linear.Time, step_h.U_ist_linear.Data(:,5)-eta_2_ap, '-.b');
ylim(1e-3*[-5 3]);
xlim([0 1.5]);
title('\rm from w_{h1}')
grid on;
subplot(4,4,8)
plot(step_h.U_ist_linear.Time, step_h.U_ist_linear.Data(:,3)-xi_1_ap, '--r', ...
    step_h.U_ist_linear.Time, step_h.U_ist_linear.Data(:,7)-xi_2_ap, '-.b');
ylim(1e-3*[-0.5 0.5]);
xlim([0 1.5]);
grid on;
subplot(4,4,12)
plot(step_h.U_ist_linear.Time, step_h.U_ist_linear.Data(:,4)-zita_1_ap, '--r', ...
    step_h.U_ist_linear.Time, step_h.U_ist_linear.Data(:,8)-zita_2_ap, '-.b');
ylim(1e-3*[-0.5 0.5]);
xlim([0 1.5]);
grid on;
subplot(4,4,16)
plot(step_h.U_ist_linear.Time, step_h.U_ist_linear.Data(:,2)-sigmaf_1_ap, '--r', ...
    step_h.U_ist_linear.Time, step_h.U_ist_linear.Data(:,6)-sigmaf_2_ap, '-.b');
ylim([-0.1 0.3]);
xlim([0 1.5]);
grid on;
xlabel('Time [s]');
set(gcf, 'Position',[383 42 850 960/1.5]);


%% plot nichtlinear und linear mit stellgrößenbeschr in einem und stellgrößen dazu
f_both = figure;
f_both.Renderer = 'painters';

subplot(8,4,1)
plot(step_v.step_linear.Time, step_v.step_linear.Data(:,1), ':k', ...
    step_v.outputs_linear.Time, step_v.outputs_linear.Data(:,1), '--r', ...
    step_v.outputs_nonlinear.Time, step_v.outputs_nonlinear.Data(:,1), '-.b');
legend('step', 'linear', 'nonlinear', 'Location', 'southeast')
ylim([0 1.2]);
ylabel('to v1 [m/s]');
title('\rm from w_{v1}')
grid on;
subplot(8,4,5)
plot(step_v.step_linear.Time, step_v.step_linear.Data(:,2), ':k', ...
    step_v.outputs_linear.Time, step_v.outputs_linear.Data(:,2), '--r', ...
    step_v.outputs_nonlinear.Time, step_v.outputs_nonlinear.Data(:,2), '-.b');
ylim(1e-3*[-0.5 0.5]);
ylabel('to \Phi1 [rad]');
grid on;
subplot(8,4,9)
plot(step_v.outputs_linear.Time, step_v.outputs_linear.Data(:,3)-theta_1_ap, '--r', ...
    step_v.outputs_nonlinear.Time, step_v.outputs_nonlinear.Data(:,3)-theta_1_ap, '-.b');
ylim(1e-3*[-0.5 0.5]);
ylabel('to \Theta1 [rad]');
grid on;
subplot(8,4,13)
plot(step_v.outputs_linear.Time, step_v.outputs_linear.Data(:,4)-h_1_ap, '--r', ...
    step_v.outputs_nonlinear.Time, step_v.outputs_nonlinear.Data(:,4)-h_1_ap, '-.b');
ylim(1e-2*[-0.5 0.5]);
ylabel('to h1 [m]');
grid on;
subplot(8,4,17)
plot(step_v.outputs_linear.Time, step_v.outputs_linear.Data(:,5), '--r', ...
    step_v.outputs_nonlinear.Time, step_v.outputs_nonlinear.Data(:,5), '-.b');
ylim(1e-3*[-0.5 0.5]);
ylabel('to \Delta v [m/s]');
grid on;
subplot(8,4,21)
plot(step_v.outputs_linear.Time, step_v.outputs_linear.Data(:,6), '--r', ...
    step_v.outputs_nonlinear.Time, step_v.outputs_nonlinear.Data(:,6), '-.b');
ylim(1e-3*[-0.5 0.5]);
ylabel('to \Delta \Phi [rad]');
grid on;
subplot(8,4,25)
plot(step_v.outputs_linear.Time, step_v.outputs_linear.Data(:,7)-delta_theta_ap, '--r', ...
    step_v.outputs_nonlinear.Time, step_v.outputs_nonlinear.Data(:,7)-delta_theta_ap, '-.b');
ylim(1e-3*[-0.5 0.5]);
ylabel('to \Delta \Theta [rad]');
grid on;
subplot(8,4,29)
plot(step_v.outputs_linear.Time, step_v.outputs_linear.Data(:,8)-delta_h_ap, '--r', ...
    step_v.outputs_nonlinear.Time, step_v.outputs_nonlinear.Data(:,8)-delta_h_ap, '-.b');
ylim(1e-3*[-0.5 0.5]);
ylabel('to \Delta h [m]');
xlabel('Time [s]');
grid on;

subplot(8,4,2)
plot(step_phi.outputs_linear.Time, step_phi.outputs_linear.Data(:,1), '--r', ...
    step_phi.outputs_nonlinear.Time, step_phi.outputs_nonlinear.Data(:,1), '-.b');
ylim(1e-1*[-0.5 0.5]);
title('\rm from w_{\Phi1}')
grid on;
subplot(8,4,6)
plot(step_phi.step_linear.Time, step_phi.step_linear.Data(:,2), ':k', ...
    step_phi.outputs_linear.Time, step_phi.outputs_linear.Data(:,2), '--r', ...
    step_phi.outputs_nonlinear.Time, step_phi.outputs_nonlinear.Data(:,2), '-.b');
ylim([-0.05 0.3]);
grid on;
subplot(8,4,10)
plot(step_phi.outputs_linear.Time, step_phi.outputs_linear.Data(:,3)-theta_1_ap, '--r', ...
    step_phi.outputs_nonlinear.Time, step_phi.outputs_nonlinear.Data(:,3)-theta_1_ap, '-.b');
ylim(1e-2*[-0.5 0.5]);
grid on;
subplot(8,4,14)
plot(step_phi.outputs_linear.Time, step_phi.outputs_linear.Data(:,4)-h_1_ap, '--r', ...
    step_phi.outputs_nonlinear.Time, step_phi.outputs_nonlinear.Data(:,4)-h_1_ap, '-.b');
ylim([-5 5]);
grid on;
subplot(8,4,18)
plot(step_phi.outputs_linear.Time, step_phi.outputs_linear.Data(:,5), '--r', ...
    step_phi.outputs_nonlinear.Time, step_phi.outputs_nonlinear.Data(:,5), '-.b');
ylim(1e-3*[-0.5 0.5]);
grid on;
subplot(8,4,22)
plot(step_phi.outputs_linear.Time, step_phi.outputs_linear.Data(:,6), '--r', ...
    step_phi.outputs_nonlinear.Time, step_phi.outputs_nonlinear.Data(:,6), '-.b');
ylim(1e-3*[-0.5 0.5]);
grid on;
subplot(8,4,26)
plot(step_phi.outputs_linear.Time, step_phi.outputs_linear.Data(:,7)-delta_theta_ap, '--r', ...
    step_phi.outputs_nonlinear.Time, step_phi.outputs_nonlinear.Data(:,7)-delta_theta_ap, '-.b');
ylim(1e-3*[-0.5 0.5]);
grid on;
subplot(8,4,30)
plot(step_phi.outputs_linear.Time, step_phi.outputs_linear.Data(:,8)-delta_h_ap, '--r', ...
    step_phi.outputs_nonlinear.Time, step_phi.outputs_nonlinear.Data(:,8)-delta_h_ap, '-.b');
ylim(1e-2*[-0.5 0.5]);
xlabel('Time [s]');
grid on;

subplot(8,4,3)
plot(step_theta.outputs_linear.Time, step_theta.outputs_linear.Data(:,1), '--r', ...
    step_theta.outputs_nonlinear.Time, step_theta.outputs_nonlinear.Data(:,1), '-.b');
ylim([-15 30]);
title('\rm from w_{\Theta1}')
grid on;
subplot(8,4,7)
plot(step_theta.outputs_linear.Time, step_theta.outputs_linear.Data(:,2), '--r', ...
    step_theta.outputs_nonlinear.Time, step_theta.outputs_nonlinear.Data(:,2), '-.b');
ylim([-1 5]);
grid on;
subplot(8,4,11)
plot(step_theta.step_linear.Time, step_theta.step_linear.Data(:,3), ':k', ...
    step_theta.outputs_linear.Time, step_theta.outputs_linear.Data(:,3)-theta_1_ap, '--r', ...
    step_theta.outputs_nonlinear.Time, step_theta.outputs_nonlinear.Data(:,3)-theta_1_ap, '-.b');
ylim([-2 3]);
grid on;
subplot(8,4,15)
plot(step_theta.outputs_linear.Time, step_theta.outputs_linear.Data(:,4)-h_1_ap, '--r', ...
    step_theta.outputs_nonlinear.Time, step_theta.outputs_nonlinear.Data(:,4)-h_1_ap, '-.b');
ylim([-1000 4000]);
grid on;
subplot(8,4,19)
plot(step_theta.outputs_linear.Time, step_theta.outputs_linear.Data(:,5), '--r', ...
    step_theta.outputs_nonlinear.Time, step_theta.outputs_nonlinear.Data(:,5), '-.b');
ylim([-0.5 0.5]);
grid on;
subplot(8,4,23)
plot(step_theta.outputs_linear.Time, step_theta.outputs_linear.Data(:,6), '--r', ...
    step_theta.outputs_nonlinear.Time, step_theta.outputs_nonlinear.Data(:,6), '-.b');
ylim([-5 5]);
grid on;
subplot(8,4,27)
plot(step_theta.outputs_linear.Time, step_theta.outputs_linear.Data(:,7)-delta_theta_ap, '--r', ...
    step_theta.outputs_nonlinear.Time, step_theta.outputs_nonlinear.Data(:,7)-delta_theta_ap, '-.b');
ylim([-0.5 0.5]);
grid on;
subplot(8,4,31)
plot(step_theta.outputs_linear.Time, step_theta.outputs_linear.Data(:,8)-delta_h_ap, '--r', ...
    step_theta.outputs_nonlinear.Time, step_theta.outputs_nonlinear.Data(:,8)-delta_h_ap, '-.b');
ylim([-15 35]);
xlabel('Time [s]');
grid on;

subplot(8,4,4)
plot(step_h.outputs_linear.Time, step_h.outputs_linear.Data(:,1), '--r', ...
    step_h.outputs_nonlinear.Time, step_h.outputs_nonlinear.Data(:,1), '-.b');
ylim(1e-3*[-0.5 0.5]);
title('\rm from w_{h1}')
grid on;
subplot(8,4,8)
plot(step_h.outputs_linear.Time, step_h.outputs_linear.Data(:,2), '--r', ...
    step_h.outputs_nonlinear.Time, step_h.outputs_nonlinear.Data(:,2), '-.b');
ylim(1e-3*[-0.5 0.5]);
grid on;
subplot(8,4,12)
plot(step_h.outputs_linear.Time, step_h.outputs_linear.Data(:,3)-theta_1_ap, '--r', ...
    step_h.outputs_nonlinear.Time, step_h.outputs_nonlinear.Data(:,3)-theta_1_ap, '-.b');
ylim(1e-1*[-0.5 0.5]);
grid on;
subplot(8,4,16)
plot(step_h.step_linear.Time, step_h.step_linear.Data(:,4), ':k', ...
    step_h.outputs_linear.Time, step_h.outputs_linear.Data(:,4)-h_1_ap, '--r', ...
    step_h.outputs_nonlinear.Time, step_h.outputs_nonlinear.Data(:,4)-h_1_ap, '-.b');
ylim([-2 10]);
grid on;
subplot(8,4,20)
plot(step_h.outputs_linear.Time, step_h.outputs_linear.Data(:,5), '--r', ...
    step_h.outputs_nonlinear.Time, step_h.outputs_nonlinear.Data(:,5), '-.b');
ylim(1e-3*[-0.5 0.5]);
grid on;
subplot(8,4,24)
plot(step_h.outputs_linear.Time, step_h.outputs_linear.Data(:,6), '--r', ...
    step_h.outputs_nonlinear.Time, step_h.outputs_nonlinear.Data(:,6), '-.b');
ylim(1e-3*[-0.5 0.5]);
grid on;
subplot(8,4,28)
plot(step_h.outputs_linear.Time, step_h.outputs_linear.Data(:,7)-delta_theta_ap, '--r', ...
    step_h.outputs_nonlinear.Time, step_h.outputs_nonlinear.Data(:,7)-delta_theta_ap, '-.b');
ylim(1e-3*[-0.5 0.5]);
grid on;
subplot(8,4,32)
plot(step_h.outputs_linear.Time, step_h.outputs_linear.Data(:,8)-delta_h_ap, '--r', ...
    step_h.outputs_nonlinear.Time, step_h.outputs_nonlinear.Data(:,8)-delta_h_ap, '-.b');
ylim(1e-3*[-0.5 0.5]);
xlabel('Time [s]');
grid on;
set(gcf, 'Position',[383 42 850 960]);


f_stell_both = figure;
f_stell_both.Renderer = 'painters';
subplot(4,4,1)
plot(step_v.U_ist_linear.Time, step_v.U_ist_linear.Data(:,1)-eta_1_ap, '--r', ...
    step_v.U_ist_linear.Time, step_v.U_ist_linear.Data(:,5)-eta_2_ap, '--b', ...
    step_v.U_ist_nonlinear.Time, step_v.U_ist_nonlinear.Data(:,1)-eta_1_ap, '-.r', ...
    step_v.U_ist_nonlinear.Time, step_v.U_ist_nonlinear.Data(:,5)-eta_2_ap, '-.b');
legend('linear plane1', 'linear plane2', 'nonlinear plane1', 'nonlinear plane2', 'Location', 'northeast')
ylim(1e-3*[-3 0.5]);
xlim([0 1.5]);
ylabel('to \eta');
title('\rm from w_{v1}')
grid on;
subplot(4,4,5)
plot(step_v.U_ist_linear.Time, step_v.U_ist_linear.Data(:,3), '--r', ...
    step_v.U_ist_linear.Time, step_v.U_ist_linear.Data(:,7), '--b', ...
    step_v.U_ist_nonlinear.Time, step_v.U_ist_nonlinear.Data(:,3), '-.r', ...
    step_v.U_ist_nonlinear.Time, step_v.U_ist_nonlinear.Data(:,7), '-.b');
ylim([-0.05 0.2]);
xlim([0 1.5]);
ylabel('to \xi');
grid on;
subplot(4,4,9)
plot(step_v.U_ist_linear.Time, step_v.U_ist_linear.Data(:,4), '--r', ...
    step_v.U_ist_linear.Time, step_v.U_ist_linear.Data(:,8), '--b', ...
    step_v.U_ist_nonlinear.Time, step_v.U_ist_nonlinear.Data(:,4), '-.r', ...
    step_v.U_ist_nonlinear.Time, step_v.U_ist_nonlinear.Data(:,8), '-.b');
ylim([-0.05 0.5]);
xlim([0 1.5]);
ylabel('to \zeta');
grid on;
subplot(4,4,13)
plot(step_v.U_ist_linear.Time, step_v.U_ist_linear.Data(:,2)-sigmaf_1_ap, '--r', ...
    step_v.U_ist_linear.Time, step_v.U_ist_linear.Data(:,6)-sigmaf_2_ap, '--b', ...
    step_v.U_ist_nonlinear.Time, step_v.U_ist_nonlinear.Data(:,2)-sigmaf_1_ap, '-.r', ...
    step_v.U_ist_nonlinear.Time, step_v.U_ist_nonlinear.Data(:,6)-sigmaf_2_ap, '-.b');
ylim(1e-3*[-0.5 0.5]);
xlim([0 1.5]);
ylabel('to sigma_f');
grid on;

subplot(4,4,2)
plot(step_phi.U_ist_linear.Time, step_phi.U_ist_linear.Data(:,1)-eta_1_ap, '--r', ...
    step_phi.U_ist_linear.Time, step_phi.U_ist_linear.Data(:,5)-eta_2_ap, '--b', ...
    step_phi.U_ist_nonlinear.Time, step_phi.U_ist_nonlinear.Data(:,1)-eta_1_ap, '-.r', ...
    step_phi.U_ist_nonlinear.Time, step_phi.U_ist_nonlinear.Data(:,5)-eta_2_ap, '-.b');
ylim(1e-2*[-0.5 0.5]);
xlim([0 5]);
title('\rm from w_{\Phi1}')
grid on;
subplot(4,4,6)
plot(step_phi.U_ist_linear.Time, step_phi.U_ist_linear.Data(:,3), '--r', ...
    step_phi.U_ist_linear.Time, step_phi.U_ist_linear.Data(:,7), '--b', ...
    step_phi.U_ist_nonlinear.Time, step_phi.U_ist_nonlinear.Data(:,3), '-.r', ...
    step_phi.U_ist_nonlinear.Time, step_phi.U_ist_nonlinear.Data(:,7), '-.b');
ylim([-0.2 0.1]);
xlim([0 5]);
grid on;
subplot(4,4,10)
plot(step_phi.U_ist_linear.Time, step_phi.U_ist_linear.Data(:,4), '--r', ...
    step_phi.U_ist_linear.Time, step_phi.U_ist_linear.Data(:,8), '--b', ...
    step_phi.U_ist_nonlinear.Time, step_phi.U_ist_nonlinear.Data(:,4), '-.r', ...
    step_phi.U_ist_nonlinear.Time, step_phi.U_ist_nonlinear.Data(:,8), '-.b');
ylim([-0.1 0.1]);
xlim([0 5]);
grid on;
subplot(4,4,14)
plot(step_phi.U_ist_linear.Time, step_phi.U_ist_linear.Data(:,2)-sigmaf_1_ap, '--r', ...
    step_phi.U_ist_linear.Time, step_phi.U_ist_linear.Data(:,6)-sigmaf_2_ap, '--b', ...
    step_phi.U_ist_nonlinear.Time, step_phi.U_ist_nonlinear.Data(:,2)-sigmaf_1_ap, '-.r', ...
    step_phi.U_ist_nonlinear.Time, step_phi.U_ist_nonlinear.Data(:,6)-sigmaf_2_ap, '-.b');
ylim(1e-2*[-0.5 0.5]);
xlim([0 5]);
grid on;

subplot(4,4,3)
plot(step_theta.U_ist_linear.Time, step_theta.U_ist_linear.Data(:,1)-eta_1_ap, '--r', ...
    step_theta.U_ist_linear.Time, step_theta.U_ist_linear.Data(:,5)-eta_2_ap, '--b', ...
    step_theta.U_ist_nonlinear.Time, step_theta.U_ist_nonlinear.Data(:,1)-eta_1_ap, '-.r', ...
    step_theta.U_ist_nonlinear.Time, step_theta.U_ist_nonlinear.Data(:,5)-eta_2_ap, '-.b');
ylim([-0.5 0.4]);
xlim([0 20]);
title('\rm from w_{\Theta1}')
grid on;
subplot(4,4,7)
plot(step_theta.U_ist_linear.Time, step_theta.U_ist_linear.Data(:,3), '--r', ...
    step_theta.U_ist_linear.Time, step_theta.U_ist_linear.Data(:,7), '--b', ...
    step_theta.U_ist_nonlinear.Time, step_theta.U_ist_nonlinear.Data(:,3), '-.r', ...
    step_theta.U_ist_nonlinear.Time, step_theta.U_ist_nonlinear.Data(:,7), '-.b');
ylim(1*[-0.6 0.6]);
xlim([0 20]);
grid on;
subplot(4,4,11)
plot(step_theta.U_ist_linear.Time, step_theta.U_ist_linear.Data(:,4), '--r', ...
    step_theta.U_ist_linear.Time, step_theta.U_ist_linear.Data(:,8), '--b', ...
    step_theta.U_ist_nonlinear.Time, step_theta.U_ist_nonlinear.Data(:,4), '-.r', ...
    step_theta.U_ist_nonlinear.Time, step_theta.U_ist_nonlinear.Data(:,8), '-.b');
ylim(1*[-0.6 0.2]);
xlim([0 20]);
grid on;
subplot(4,4,15)
plot(step_theta.U_ist_linear.Time, step_theta.U_ist_linear.Data(:,2)-sigmaf_1_ap, '--r', ...
    step_theta.U_ist_linear.Time, step_theta.U_ist_linear.Data(:,6)-sigmaf_2_ap, '--b', ...
    step_theta.U_ist_nonlinear.Time, step_theta.U_ist_nonlinear.Data(:,2)-sigmaf_1_ap, '-.r', ...
    step_theta.U_ist_nonlinear.Time, step_theta.U_ist_nonlinear.Data(:,6)-sigmaf_2_ap, '-.b');
ylim([-0.3 0.2]);
xlim([0 20]);
grid on;

subplot(4,4,4)
plot(step_h.U_ist_linear.Time, step_h.U_ist_linear.Data(:,1)-eta_1_ap, '--r', ...
    step_h.U_ist_linear.Time, step_h.U_ist_linear.Data(:,5)-eta_2_ap, '--b', ...
    step_h.U_ist_nonlinear.Time, step_h.U_ist_nonlinear.Data(:,1)-eta_1_ap, '-.r', ...
    step_h.U_ist_nonlinear.Time, step_h.U_ist_nonlinear.Data(:,5)-eta_2_ap, '-.b');
ylim(1e-2*[-1 0.5]);
xlim([0 5]);
title('\rm from w_{h1}')
grid on;
subplot(4,4,8)
plot(step_h.U_ist_linear.Time, step_h.U_ist_linear.Data(:,3), '--r', ...
    step_h.U_ist_linear.Time, step_h.U_ist_linear.Data(:,7), '--b', ...
    step_h.U_ist_nonlinear.Time, step_h.U_ist_nonlinear.Data(:,3), '-.r', ...
    step_h.U_ist_nonlinear.Time, step_h.U_ist_nonlinear.Data(:,7), '-.b');
ylim(1e-2*[-0.5 0.5]);
xlim([0 5]);
grid on;
subplot(4,4,12)
plot(step_h.U_ist_linear.Time, step_h.U_ist_linear.Data(:,4), '--r', ...
    step_h.U_ist_linear.Time, step_h.U_ist_linear.Data(:,8), '--b', ...
    step_h.U_ist_nonlinear.Time, step_h.U_ist_nonlinear.Data(:,4), '-.r', ...
    step_h.U_ist_nonlinear.Time, step_h.U_ist_nonlinear.Data(:,8), '-.b');
ylim(1e-2*[-0.5 0.5]);
xlim([0 5]);
grid on;
subplot(4,4,16)
plot(step_h.U_ist_linear.Time, step_h.U_ist_linear.Data(:,2)-sigmaf_1_ap, '--r', ...
    step_h.U_ist_linear.Time, step_h.U_ist_linear.Data(:,6)-sigmaf_2_ap, '--b', ...
    step_h.U_ist_nonlinear.Time, step_h.U_ist_nonlinear.Data(:,2)-sigmaf_1_ap, '-.r', ...
    step_h.U_ist_nonlinear.Time, step_h.U_ist_nonlinear.Data(:,6)-sigmaf_2_ap, '-.b');
ylim(1e-1*[-0.3 1]);
xlim([0 5]);
grid on;
set(gcf, 'Position',[383 42 850 960/1.5]);

%% load full mass model (max 14000kg)
load('C:\Users\Markus\Documents\Uni\WISE2021\Projektseminar\PS_Luftbetankung\PS_Luftbetankung_newRepo\Projekteseminar-Luftbetankung\Code\Messungen\step_v_massmodel_1_50percent.mat')
step_v_m2 = out;
clear out;
load('C:\Users\Markus\Documents\Uni\WISE2021\Projektseminar\PS_Luftbetankung\PS_Luftbetankung_newRepo\Projekteseminar-Luftbetankung\Code\Messungen\step_phi_massmodel_1_50percent.mat')
step_phi_m2 = out;
clear out;
load('C:\Users\Markus\Documents\Uni\WISE2021\Projektseminar\PS_Luftbetankung\PS_Luftbetankung_newRepo\Projekteseminar-Luftbetankung\Code\Messungen\step_theta_massmodel_1_50percent.mat')
step_theta_m2 = out;
clear out;
load('C:\Users\Markus\Documents\Uni\WISE2021\Projektseminar\PS_Luftbetankung\PS_Luftbetankung_newRepo\Projekteseminar-Luftbetankung\Code\Messungen\step_h_massmodel_1_50percent.mat')
step_h_m2 = out;
clear out;

load('C:\Users\Markus\Documents\Uni\WISE2021\Projektseminar\PS_Luftbetankung\PS_Luftbetankung_newRepo\Projekteseminar-Luftbetankung\Code\Messungen\step_v_massmodel_2_50percent.mat')
step_v_m3 = out;
clear out;
load('C:\Users\Markus\Documents\Uni\WISE2021\Projektseminar\PS_Luftbetankung\PS_Luftbetankung_newRepo\Projekteseminar-Luftbetankung\Code\Messungen\step_phi_massmodel_2_50percent.mat')
step_phi_m3 = out;
clear out;
load('C:\Users\Markus\Documents\Uni\WISE2021\Projektseminar\PS_Luftbetankung\PS_Luftbetankung_newRepo\Projekteseminar-Luftbetankung\Code\Messungen\step_theta_massmodel_2_50percent.mat')
step_theta_m3 = out;
clear out;
load('C:\Users\Markus\Documents\Uni\WISE2021\Projektseminar\PS_Luftbetankung\PS_Luftbetankung_newRepo\Projekteseminar-Luftbetankung\Code\Messungen\step_h_massmodel_2_50percent.mat')
step_h_m3 = out;
clear out;

X_AP_m2 = [150;0;-9.30119255189732;0;0;0;0;-0.0619286599563016;0;5000;150;0;-11.0065445940443;0;0;0;0;-0.0732456962006296;0;5010];
U_AP_m2 = [-0.0953740994051672;0.225432849684937;0;0;-0.0871233228705516;0.247511353508586;0;0];
X_AP_m3 = [150;0;-11.0225402979539;0;0;0;0;-0.0733517623176560;0;5000;150;0;-9.28336222787026;0;0;0;0;-0.0618102455588889;0;5010];
U_AP_m3 = [-0.0870457297521801;0.247747504909435;0;0;-0.0954601345403630;0.225231005473444;0;0];
theta_1_m2 = X_AP_m2(8);
theta_1_m3 = X_AP_m3(8);
h_1_m2 = X_AP_m2(10);
h_1_m3 = X_AP_m3(10);
delta_theta_m2 = X_AP_m2(8)-X_AP_m2(18);
delta_theta_m3 = X_AP_m3(8)-X_AP_m3(18);
delta_h_m2 = X_AP_m2(10)-X_AP_m2(20);
delta_h_m3 = X_AP_m3(10)-X_AP_m3(20);
eta_1_m2 = U_AP_m2(1);
eta_2_m2 = U_AP_m2(5);
eta_1_m3 = U_AP_m3(1);
eta_2_m3 = U_AP_m3(5);
sigmaf_1_m2 = U_AP_m2(2);
sigmaf_2_m2 = U_AP_m2(6);
sigmaf_1_m3 = U_AP_m3(2);
sigmaf_2_m3 = U_AP_m3(6);

%% alle y für zwei massen modelle
f_all_out = figure;
f_all_out.Renderer = 'painters';
subplot(8,4,1)
plot(step_v_m2.step_linear.Time, step_v_m2.step_linear.Data(:,1), ':k', ...
    step_v_m2.outputs_linear.Time, step_v_m2.outputs_linear.Data(:,1), '--r', ...
    step_v_m3.outputs_linear.Time, step_v_m3.outputs_linear.Data(:,1), '--b', ...
    step_v_m2.outputs_nonlinear.Time, step_v_m2.outputs_nonlinear.Data(:,1), '-.r', ...
    step_v_m3.outputs_nonlinear.Time, step_v_m3.outputs_nonlinear.Data(:,1), '-.b');
legend('step', 'linear, Model 2', 'linear, Model 3', 'nonlinear, Model 2', 'nonlinear, Model 3', 'Location', 'southeast')
ylim([0 1.2]);
ylabel('to v1 [m/s]');
title('\rm from w_{v1}')
grid on;
subplot(8,4,5)
plot(step_v_m2.outputs_linear.Time, step_v_m2.outputs_linear.Data(:,2), '--r', ...
    step_v_m3.outputs_linear.Time, step_v_m3.outputs_linear.Data(:,2), '--b', ...
    step_v_m2.outputs_nonlinear.Time, step_v_m2.outputs_nonlinear.Data(:,2), '-.r', ...
    step_v_m3.outputs_nonlinear.Time, step_v_m3.outputs_nonlinear.Data(:,2), '-.b');
ylim(1e-2*[-0.2 0.2]);
ylabel('to \Phi1 [rad]');
grid on;
subplot(8,4,9)
plot(step_v_m2.outputs_linear.Time, step_v_m2.outputs_linear.Data(:,3)-theta_1_m2, '--r', ...
    step_v_m3.outputs_linear.Time, step_v_m3.outputs_linear.Data(:,3)-theta_1_m3, '--b', ...
    step_v_m2.outputs_nonlinear.Time, step_v_m2.outputs_nonlinear.Data(:,3)-theta_1_m2, '-.r', ...
    step_v_m3.outputs_nonlinear.Time, step_v_m3.outputs_nonlinear.Data(:,3)-theta_1_m3, '-.b');
ylim(1e-2*[-0.5 0.5]);
ylabel('to \Theta1 [rad]');
grid on;
subplot(8,4,13)
plot(step_v_m2.outputs_linear.Time, step_v_m2.outputs_linear.Data(:,4)-h_1_m2, '--r', ...
    step_v_m3.outputs_linear.Time, step_v_m3.outputs_linear.Data(:,4)-h_1_m3, '--b', ...
    step_v_m2.outputs_nonlinear.Time, step_v_m2.outputs_nonlinear.Data(:,4)-h_1_m2, '-.r', ...
    step_v_m3.outputs_nonlinear.Time, step_v_m3.outputs_nonlinear.Data(:,4)-h_1_m3, '-.b');
ylim([-3 2]);
ylabel('to h1 [m]');
grid on;
subplot(8,4,17)
plot(step_v_m2.outputs_linear.Time, step_v_m2.outputs_linear.Data(:,5), '--r', ...
    step_v_m3.outputs_linear.Time, step_v_m3.outputs_linear.Data(:,5), '--b', ...
    step_v_m2.outputs_nonlinear.Time, step_v_m2.outputs_nonlinear.Data(:,5), '-.r', ...
    step_v_m3.outputs_nonlinear.Time, step_v_m3.outputs_nonlinear.Data(:,5), '-.b');
ylim(1e-2*[-1 1]);
ylabel('to \Delta v [m/s]');
grid on;
subplot(8,4,21)
plot(step_v_m2.outputs_linear.Time, step_v_m2.outputs_linear.Data(:,6), '--r', ...
    step_v_m3.outputs_linear.Time, step_v_m3.outputs_linear.Data(:,6), '--b', ...
    step_v_m2.outputs_nonlinear.Time, step_v_m2.outputs_nonlinear.Data(:,6), '-.r', ...
    step_v_m3.outputs_nonlinear.Time, step_v_m3.outputs_nonlinear.Data(:,6), '-.b');
ylim(1e-2*[-0.5 0.5]);
ylabel('to \Delta \Phi [rad]');
grid on;
subplot(8,4,25)
plot(step_v_m2.outputs_linear.Time, step_v_m2.outputs_linear.Data(:,7)-delta_theta_m2, '--r', ...
    step_v_m3.outputs_linear.Time, step_v_m3.outputs_linear.Data(:,7)-delta_theta_m3, '--b', ...
    step_v_m2.outputs_nonlinear.Time, step_v_m2.outputs_nonlinear.Data(:,7)-delta_theta_m2, '-.r', ...
    step_v_m3.outputs_nonlinear.Time, step_v_m3.outputs_nonlinear.Data(:,7)-delta_theta_m3, '-.b');
ylim(1e-1*[-0.5 0.5]);
ylabel('to \Delta \Theta [rad]');
grid on;
subplot(8,4,29)
plot(step_v_m2.outputs_linear.Time, step_v_m2.outputs_linear.Data(:,8)-delta_h_m2, '--r', ...
    step_v_m3.outputs_linear.Time, step_v_m3.outputs_linear.Data(:,8)-delta_h_m3, '--b', ...
    step_v_m2.outputs_nonlinear.Time, step_v_m2.outputs_nonlinear.Data(:,8)-delta_h_m2, '-.r', ...
    step_v_m3.outputs_nonlinear.Time, step_v_m3.outputs_nonlinear.Data(:,8)-delta_h_m3, '-.b');
ylim(10*[-0.5 0.5]);
ylabel('to \Delta h [m]');
xlabel('Time [s]');
grid on;

subplot(8,4,2)
plot(step_phi_m2.outputs_linear.Time, step_phi_m2.outputs_linear.Data(:,1), '--r', ...
    step_phi_m3.outputs_linear.Time, step_phi_m3.outputs_linear.Data(:,1), '--b', ...
    step_phi_m2.outputs_nonlinear.Time, step_phi_m2.outputs_nonlinear.Data(:,1), '-.r', ...
    step_phi_m3.outputs_nonlinear.Time, step_phi_m3.outputs_nonlinear.Data(:,1), '-.b');
ylim([-0.05 0.15]);
title('\rm from w_{\Phi1}')
grid on;
subplot(8,4,6)
plot(step_phi_m2.step_linear.Time, step_phi_m2.step_linear.Data(:,2), ':k', ...
    step_phi_m2.outputs_linear.Time, step_phi_m2.outputs_linear.Data(:,2), '--r', ...
    step_phi_m3.outputs_linear.Time, step_phi_m3.outputs_linear.Data(:,2), '--b', ...
    step_phi_m2.outputs_nonlinear.Time, step_phi_m2.outputs_nonlinear.Data(:,2), '-.r', ...
    step_phi_m3.outputs_nonlinear.Time, step_phi_m3.outputs_nonlinear.Data(:,2), '-.b');
ylim([-0.05 0.3]);
grid on;
subplot(8,4,10)
plot(step_phi_m2.outputs_linear.Time, step_phi_m2.outputs_linear.Data(:,3)-theta_1_m2, '--r', ...
    step_phi_m3.outputs_linear.Time, step_phi_m3.outputs_linear.Data(:,3)-theta_1_m3, '--b', ...
    step_phi_m2.outputs_nonlinear.Time, step_phi_m2.outputs_nonlinear.Data(:,3)-theta_1_m2, '-.r', ...
    step_phi_m3.outputs_nonlinear.Time, step_phi_m3.outputs_nonlinear.Data(:,3)-theta_1_m3, '-.b');
ylim(1e-2*[-0.5 0.5]);
grid on;
subplot(8,4,14)
plot(step_phi_m2.outputs_linear.Time, step_phi_m2.outputs_linear.Data(:,4)-h_1_m2, '--r', ...
    step_phi_m3.outputs_linear.Time, step_phi_m3.outputs_linear.Data(:,4)-h_1_m3, '--b', ...
    step_phi_m2.outputs_nonlinear.Time, step_phi_m2.outputs_nonlinear.Data(:,4)-h_1_m2, '-.r', ...
    step_phi_m3.outputs_nonlinear.Time, step_phi_m3.outputs_nonlinear.Data(:,4)-h_1_m3, '-.b');
ylim([-10 5]);
grid on;
subplot(8,4,18)
plot(step_phi_m2.outputs_linear.Time, step_phi_m2.outputs_linear.Data(:,5), '--r', ...
    step_phi_m3.outputs_linear.Time, step_phi_m3.outputs_linear.Data(:,5), '--b', ...
    step_phi_m2.outputs_nonlinear.Time, step_phi_m2.outputs_nonlinear.Data(:,5), '-.r', ...
    step_phi_m3.outputs_nonlinear.Time, step_phi_m3.outputs_nonlinear.Data(:,5), '-.b');
ylim([-0.2 0.2]);
grid on;
subplot(8,4,22)
plot(step_phi_m2.outputs_linear.Time, step_phi_m2.outputs_linear.Data(:,6), '--r', ...
    step_phi_m3.outputs_linear.Time, step_phi_m3.outputs_linear.Data(:,6), '--b', ...
    step_phi_m2.outputs_nonlinear.Time, step_phi_m2.outputs_nonlinear.Data(:,6), '-.r', ...
    step_phi_m3.outputs_nonlinear.Time, step_phi_m3.outputs_nonlinear.Data(:,6), '-.b');
ylim(1e-1*[-0.5 0.5]);
grid on;
subplot(8,4,26)
plot(step_phi_m2.outputs_linear.Time, step_phi_m2.outputs_linear.Data(:,7)-delta_theta_m2, '--r', ...
    step_phi_m3.outputs_linear.Time, step_phi_m3.outputs_linear.Data(:,7)-delta_theta_m3, '--b', ...
    step_phi_m2.outputs_nonlinear.Time, step_phi_m2.outputs_nonlinear.Data(:,7)-delta_theta_m2, '-.r', ...
    step_phi_m3.outputs_nonlinear.Time, step_phi_m3.outputs_nonlinear.Data(:,7)-delta_theta_m3, '-.b');
ylim(1e-1*[-0.5 0.5]);
grid on;
subplot(8,4,30)
plot(step_phi_m2.outputs_linear.Time, step_phi_m2.outputs_linear.Data(:,8)-delta_h_m2, '--r', ...
    step_phi_m3.outputs_linear.Time, step_phi_m3.outputs_linear.Data(:,8)-delta_h_m3, '--b', ...
    step_phi_m2.outputs_nonlinear.Time, step_phi_m2.outputs_nonlinear.Data(:,8)-delta_h_m2, '-.r', ...
    step_phi_m3.outputs_nonlinear.Time, step_phi_m3.outputs_nonlinear.Data(:,8)-delta_h_m3, '-.b');
ylim([-5 5]);
xlabel('Time [s]');
grid on;

subplot(8,4,3)
plot(step_theta_m2.outputs_linear.Time, step_theta_m2.outputs_linear.Data(:,1), '--r', ...
    step_theta_m3.outputs_linear.Time, step_theta_m3.outputs_linear.Data(:,1), '--b', ...
    step_theta_m2.outputs_nonlinear.Time, step_theta_m2.outputs_nonlinear.Data(:,1), '-.r', ...
    step_theta_m3.outputs_nonlinear.Time, step_theta_m3.outputs_nonlinear.Data(:,1), '-.b');
ylim([-40 70]);
xlim([0 50]);
title('\rm from w_{\Theta1}')
grid on;
subplot(8,4,7)
plot(step_theta_m2.outputs_linear.Time, step_theta_m2.outputs_linear.Data(:,2), '--r', ...
    step_theta_m3.outputs_linear.Time, step_theta_m3.outputs_linear.Data(:,2), '--b', ...
    step_theta_m2.outputs_nonlinear.Time, step_theta_m2.outputs_nonlinear.Data(:,2), '-.r', ...
    step_theta_m3.outputs_nonlinear.Time, step_theta_m3.outputs_nonlinear.Data(:,2), '-.b');
ylim([-5 15]);
xlim([0 50]);
grid on;
subplot(8,4,11)
plot(step_theta_m2.step_linear.Time, step_theta_m2.step_linear.Data(:,3), ':k', ...
    step_theta_m2.outputs_linear.Time, step_theta_m2.outputs_linear.Data(:,3)-theta_1_m2, '--r', ...
    step_theta_m3.outputs_linear.Time, step_theta_m3.outputs_linear.Data(:,3)-theta_1_m3, '--b', ...
    step_theta_m2.outputs_nonlinear.Time, step_theta_m2.outputs_nonlinear.Data(:,3)-theta_1_m2, '-.r', ...
    step_theta_m3.outputs_nonlinear.Time, step_theta_m3.outputs_nonlinear.Data(:,3)-theta_1_m3, '-.b');
ylim([-2 3]);
xlim([0 50]);
grid on;
subplot(8,4,15)
plot(step_theta_m2.outputs_linear.Time, step_theta_m2.outputs_linear.Data(:,4)-h_1_m2, '--r', ...
    step_theta_m3.outputs_linear.Time, step_theta_m3.outputs_linear.Data(:,4)-h_1_m3, '--b', ...
    step_theta_m2.outputs_nonlinear.Time, step_theta_m2.outputs_nonlinear.Data(:,4)-h_1_m2, '-.r', ...
    step_theta_m3.outputs_nonlinear.Time, step_theta_m3.outputs_nonlinear.Data(:,4)-h_1_m3, '-.b');
ylim([-1000 5000]);
xlim([0 50]);
grid on;
subplot(8,4,19)
plot(step_theta_m2.outputs_linear.Time, step_theta_m2.outputs_linear.Data(:,5), '--r', ...
    step_theta_m3.outputs_linear.Time, step_theta_m3.outputs_linear.Data(:,5), '--b', ...
    step_theta_m2.outputs_nonlinear.Time, step_theta_m2.outputs_nonlinear.Data(:,5), '-.r', ...
    step_theta_m3.outputs_nonlinear.Time, step_theta_m3.outputs_nonlinear.Data(:,5), '-.b');
ylim([-80 80]);
xlim([0 50]);
grid on;
subplot(8,4,23)
plot(step_theta_m2.outputs_linear.Time, step_theta_m2.outputs_linear.Data(:,6), '--r', ...
    step_theta_m3.outputs_linear.Time, step_theta_m3.outputs_linear.Data(:,6), '--b', ...
    step_theta_m2.outputs_nonlinear.Time, step_theta_m2.outputs_nonlinear.Data(:,6), '-.r', ...
    step_theta_m3.outputs_nonlinear.Time, step_theta_m3.outputs_nonlinear.Data(:,6), '-.b');
ylim([-10 10]);
xlim([0 50]);
grid on;
subplot(8,4,27)
plot(step_theta_m2.outputs_linear.Time, step_theta_m2.outputs_linear.Data(:,7)-delta_theta_m2, '--r', ...
    step_theta_m3.outputs_linear.Time, step_theta_m3.outputs_linear.Data(:,7)-delta_theta_m3, '--b', ...
    step_theta_m2.outputs_nonlinear.Time, step_theta_m2.outputs_nonlinear.Data(:,7)-delta_theta_m2, '-.r', ...
    step_theta_m3.outputs_nonlinear.Time, step_theta_m3.outputs_nonlinear.Data(:,7)-delta_theta_m3, '-.b');
ylim([-3 3]);
xlim([0 50]);
grid on;
subplot(8,4,31)
plot(step_theta_m2.outputs_linear.Time, step_theta_m2.outputs_linear.Data(:,8)-delta_h_m2, '--r', ...
    step_theta_m3.outputs_linear.Time, step_theta_m3.outputs_linear.Data(:,8)-delta_h_m3, '--b', ...
    step_theta_m2.outputs_nonlinear.Time, step_theta_m2.outputs_nonlinear.Data(:,8)-delta_h_m2, '-.r', ...
    step_theta_m3.outputs_nonlinear.Time, step_theta_m3.outputs_nonlinear.Data(:,8)-delta_h_m3, '-.b');
ylim([-600 600]);
xlim([0 50]);
xlabel('Time [s]');
grid on;

subplot(8,4,4)
plot(step_h_m2.outputs_linear.Time, step_h_m2.outputs_linear.Data(:,1), '--r', ...
    step_h_m3.outputs_linear.Time, step_h_m3.outputs_linear.Data(:,1), '--b', ...
    step_h_m2.outputs_nonlinear.Time, step_h_m2.outputs_nonlinear.Data(:,1), '-.r', ...
    step_h_m3.outputs_nonlinear.Time, step_h_m3.outputs_nonlinear.Data(:,1), '-.b');
ylim(1e-3*[-0.5 0.5]);
title('\rm from w_{h1}')
grid on;
subplot(8,4,8)
plot(step_h_m2.outputs_linear.Time, step_h_m2.outputs_linear.Data(:,2), '--r', ...
    step_h_m3.outputs_linear.Time, step_h_m3.outputs_linear.Data(:,2), '--b', ...
    step_h_m2.outputs_nonlinear.Time, step_h_m2.outputs_nonlinear.Data(:,2), '-.r', ...
    step_h_m3.outputs_nonlinear.Time, step_h_m3.outputs_nonlinear.Data(:,2), '-.b');
ylim(1e-3*[-0.5 0.5]);
grid on;
subplot(8,4,12)
plot(step_h_m2.outputs_linear.Time, step_h_m2.outputs_linear.Data(:,3)-theta_1_m2, '--r', ...
    step_h_m3.outputs_linear.Time, step_h_m3.outputs_linear.Data(:,3)-theta_1_m3, '--b', ...
    step_h_m2.outputs_nonlinear.Time, step_h_m2.outputs_nonlinear.Data(:,3)-theta_1_m2, '-.r', ...
    step_h_m3.outputs_nonlinear.Time, step_h_m3.outputs_nonlinear.Data(:,3)-theta_1_m3, '-.b');
ylim(1e-1*[-0.5 0.5]);
grid on;
subplot(8,4,16)
plot(step_h_m2.step_linear.Time, step_h_m2.step_linear.Data(:,4), ':k', ...
    step_h_m2.outputs_linear.Time, step_h_m2.outputs_linear.Data(:,4)-h_1_m2, '--r', ...
    step_h_m3.outputs_linear.Time, step_h_m3.outputs_linear.Data(:,4)-h_1_m3, '--b', ...
    step_h_m2.outputs_nonlinear.Time, step_h_m2.outputs_nonlinear.Data(:,4)-h_1_m2, '-.r', ...
    step_h_m3.outputs_nonlinear.Time, step_h_m3.outputs_nonlinear.Data(:,4)-h_1_m3, '-.b');
ylim([-2 8]);
grid on;
subplot(8,4,20)
plot(step_h_m2.outputs_linear.Time, step_h_m2.outputs_linear.Data(:,5), '--r', ...
    step_h_m3.outputs_linear.Time, step_h_m3.outputs_linear.Data(:,5), '--b', ...
    step_h_m2.outputs_nonlinear.Time, step_h_m2.outputs_nonlinear.Data(:,5), '-.r', ...
    step_h_m3.outputs_nonlinear.Time, step_h_m3.outputs_nonlinear.Data(:,5), '-.b');
ylim(1e-3*[-0.5 0.5]);
grid on;
subplot(8,4,24)
plot(step_h_m2.outputs_linear.Time, step_h_m2.outputs_linear.Data(:,6), '--r', ...
    step_h_m3.outputs_linear.Time, step_h_m3.outputs_linear.Data(:,6), '--b', ...
    step_h_m2.outputs_nonlinear.Time, step_h_m2.outputs_nonlinear.Data(:,6), '-.r', ...
    step_h_m3.outputs_nonlinear.Time, step_h_m3.outputs_nonlinear.Data(:,6), '-.b');
ylim(1e-3*[-0.5 0.5]);
grid on;
subplot(8,4,28)
plot(step_h_m2.outputs_linear.Time, step_h_m2.outputs_linear.Data(:,7)-delta_theta_m2, '--r', ...
    step_h_m3.outputs_linear.Time, step_h_m3.outputs_linear.Data(:,7)-delta_theta_m3, '--b', ...
    step_h_m2.outputs_nonlinear.Time, step_h_m2.outputs_nonlinear.Data(:,7)-delta_theta_m2, '-.r', ...
    step_h_m3.outputs_nonlinear.Time, step_h_m3.outputs_nonlinear.Data(:,7)-delta_theta_m3, '-.b');
ylim(1e-1*[-0.5 0.5]);
grid on;
subplot(8,4,32)
plot(step_h_m2.outputs_linear.Time, step_h_m2.outputs_linear.Data(:,8)-delta_h_m2, '--r', ...
    step_h_m3.outputs_linear.Time, step_h_m3.outputs_linear.Data(:,8)-delta_h_m3, '--b', ...
    step_h_m2.outputs_nonlinear.Time, step_h_m2.outputs_nonlinear.Data(:,8)-delta_h_m2, '-.r', ...
    step_h_m3.outputs_nonlinear.Time, step_h_m3.outputs_nonlinear.Data(:,8)-delta_h_m3, '-.b');
ylim([-5 5]);
xlabel('Time [s]');
grid on;
set(gcf, 'Position',[383 42 850 960]);

%% plot differenz flugzeugpositionen
f_pos = figure;
subplot(1,4,1)
plot(step_v_m2.difference_position_nonlinear.Time, step_v_m2.difference_position_nonlinear.Data(:,1), '-.r', ...
    step_v_m2.difference_position_nonlinear.Time, step_v_m2.difference_position_nonlinear.Data(:,2), '-.b', ...
    step_v_m2.difference_position_nonlinear.Time, step_v_m2.difference_position_nonlinear.Data(:,3), '-.k', ...
    step_v_m3.difference_position_nonlinear.Time, step_v_m3.difference_position_nonlinear.Data(:,1), '--r', ...
    step_v_m3.difference_position_nonlinear.Time, step_v_m3.difference_position_nonlinear.Data(:,2), '--b', ...
    step_v_m3.difference_position_nonlinear.Time, step_v_m3.difference_position_nonlinear.Data(:,3), '--k');
legend('\Delta x Modell 2', '\Delta y Modell 2', '\Delta h Modell 2', ...
    '\Delta x Modell 3', '\Delta y Modell 3', '\Delta h Modell 3', 'Location', 'southeast')
ylim([-15 5]);
title('\rm from w_{v1}')
ylabel('Distance [m]');
xlabel('Time [s]');
pbaspect([1.5 2 1])
grid on;
subplot(1,4,2)
plot(step_phi_m2.difference_position_nonlinear.Time, step_phi_m2.difference_position_nonlinear.Data(:,1), '-.r', ...
    step_phi_m2.difference_position_nonlinear.Time, step_phi_m2.difference_position_nonlinear.Data(:,2), '-.b', ...
    step_phi_m2.difference_position_nonlinear.Time, step_phi_m2.difference_position_nonlinear.Data(:,3), '-.k', ...
    step_phi_m3.difference_position_nonlinear.Time, step_phi_m3.difference_position_nonlinear.Data(:,1), '--r', ...
    step_phi_m3.difference_position_nonlinear.Time, step_phi_m3.difference_position_nonlinear.Data(:,2), '--b', ...
    step_phi_m3.difference_position_nonlinear.Time, step_phi_m3.difference_position_nonlinear.Data(:,3), '--k');
% ylim([-15 5]);
title('\rm from w_{\Phi1}')
xlabel('Time [s]');
pbaspect([1.5 2 1])
grid on;
subplot(1,4,3)
plot(step_theta_m2.difference_position_nonlinear.Time, step_theta_m2.difference_position_nonlinear.Data(:,1), '-.r', ...
    step_theta_m2.difference_position_nonlinear.Time, step_theta_m2.difference_position_nonlinear.Data(:,2), '-.b', ...
    step_theta_m2.difference_position_nonlinear.Time, step_theta_m2.difference_position_nonlinear.Data(:,3), '-.k', ...
    step_theta_m3.difference_position_nonlinear.Time, step_theta_m3.difference_position_nonlinear.Data(:,1), '--r', ...
    step_theta_m3.difference_position_nonlinear.Time, step_theta_m3.difference_position_nonlinear.Data(:,2), '--b', ...
    step_theta_m3.difference_position_nonlinear.Time, step_theta_m3.difference_position_nonlinear.Data(:,3), '--k');
% ylim([-15 5]);
title('\rm from w_{\Theta1}')
xlabel('Time [s]');
pbaspect([1.5 2 1])
grid on;
subplot(1,4,4)
plot(step_h_m2.difference_position_nonlinear.Time, step_h_m2.difference_position_nonlinear.Data(:,1), '-.r', ...
    step_h_m2.difference_position_nonlinear.Time, step_h_m2.difference_position_nonlinear.Data(:,2), '-.b', ...
    step_h_m2.difference_position_nonlinear.Time, step_h_m2.difference_position_nonlinear.Data(:,3), '-.k', ...
    step_h_m3.difference_position_nonlinear.Time, step_h_m3.difference_position_nonlinear.Data(:,1), '--r', ...
    step_h_m3.difference_position_nonlinear.Time, step_h_m3.difference_position_nonlinear.Data(:,2), '--b', ...
    step_h_m3.difference_position_nonlinear.Time, step_h_m3.difference_position_nonlinear.Data(:,3), '--k');
% ylim([-15 5]);
title('\rm from w_{h1}')
xlabel('Time [s]');
pbaspect([1.5 2 1])
grid on;
