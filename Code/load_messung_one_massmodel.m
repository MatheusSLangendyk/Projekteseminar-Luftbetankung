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

eta_max = 10*pi/180; %Elevator
eta_min = - 25*pi/180; 
sigmaf_max = 20*pi/180; %Throttl
sigmaf_min = 0.5*pi/180;
xi_max = 25*pi/180; %Airlon
xi_min = - xi_max;
zita_max = 30*pi/180; %Rudder
zita_min = - zita_max;

%% plot coupled outputs y2 mit stellgrößenbeschränkung theta
f1 = figure;
subplot(4,4,1)
plot(step_v.outputs_linear.Time, step_v.outputs_linear.Data(:,5), '--r', ...
    step_v.outputs_nonlinear.Time, step_v.outputs_nonlinear.Data(:,5), '-.b');
legend('linear', 'nonlinear', 'Location', 'southeast')
ylim(1e-3*[-0.5 0.5]);
ylabel('to \Delta v');
title('\rm from w_{v1}')
grid on;
subplot(4,4,5)
plot(step_v.outputs_linear.Time, step_v.outputs_linear.Data(:,6), '--r', ...
    step_v.outputs_nonlinear.Time, step_v.outputs_nonlinear.Data(:,6), '-.b');
ylim(1e-3*[-0.5 0.5]);
ylabel('to \Delta \Phi');
grid on;
subplot(4,4,9)
plot(step_v.outputs_linear.Time, step_v.outputs_linear.Data(:,7), '--r', ...
    step_v.outputs_nonlinear.Time, step_v.outputs_nonlinear.Data(:,7), '-.b');
ylim(1e-3*[-0.5 0.5]);
ylabel('to \Delta \Theta');
grid on;
subplot(4,4,13)
plot(step_v.outputs_linear.Time, step_v.outputs_linear.Data(:,8), '--r', ...
    step_v.outputs_nonlinear.Time, step_v.outputs_nonlinear.Data(:,8), '-.b');
ylim([-11 -9]);
ylabel('to \Delta h');
xlabel('Time [s]');
grid on;

subplot(4,4,2)
plot(step_phi.outputs_linear.Time, step_phi.outputs_linear.Data(:,5), '--r', ...
    step_phi.outputs_nonlinear.Time, step_phi.outputs_nonlinear.Data(:,5), '-.b');
ylim(1e-3*[-0.5 0.5]);
title('\rm from w_{\Phi1}')
grid on;
subplot(4,4,6)
plot(step_phi.outputs_linear.Time, step_phi.outputs_linear.Data(:,6), '--r', ...
    step_phi.outputs_nonlinear.Time, step_phi.outputs_nonlinear.Data(:,6), '-.b');
ylim(1e-3*[-0.5 0.5]);
grid on;
subplot(4,4,10)
plot(step_phi.outputs_linear.Time, step_phi.outputs_linear.Data(:,7), '--r', ...
    step_phi.outputs_nonlinear.Time, step_phi.outputs_nonlinear.Data(:,7), '-.b');
ylim(1e-3*[-0.5 0.5]);
grid on;
subplot(4,4,14)
plot(step_phi.outputs_linear.Time, step_phi.outputs_linear.Data(:,8), '--r', ...
    step_phi.outputs_nonlinear.Time, step_phi.outputs_nonlinear.Data(:,8), '-.b');
ylim([-11 -9]);
xlabel('Time [s]');
grid on;

subplot(4,4,3)
plot(step_theta.outputs_linear.Time, step_theta.outputs_linear.Data(:,5), '--r', ...
    step_theta.outputs_nonlinear.Time, step_theta.outputs_nonlinear.Data(:,5), '-.b');
ylim([-0.5 0.5]);
title('\rm from w_{\Theta1}')
grid on;
subplot(4,4,7)
plot(step_theta.outputs_linear.Time, step_theta.outputs_linear.Data(:,6), '--r', ...
    step_theta.outputs_nonlinear.Time, step_theta.outputs_nonlinear.Data(:,6), '-.b');
ylim([-5 5]);
grid on;
subplot(4,4,11)
plot(step_theta.outputs_linear.Time, step_theta.outputs_linear.Data(:,7), '--r', ...
    step_theta.outputs_nonlinear.Time, step_theta.outputs_nonlinear.Data(:,7), '-.b');
ylim([-0.5 0.5]);
grid on;
subplot(4,4,15)
plot(step_theta.outputs_linear.Time, step_theta.outputs_linear.Data(:,8), '--r', ...
    step_theta.outputs_nonlinear.Time, step_theta.outputs_nonlinear.Data(:,8), '-.b');
ylim([-15 25]);
xlabel('Time [s]');
grid on;

subplot(4,4,4)
plot(step_h.outputs_linear.Time, step_h.outputs_linear.Data(:,5), '--r', ...
    step_h.outputs_nonlinear.Time, step_h.outputs_nonlinear.Data(:,5), '-.b');
ylim(1e-3*[-0.5 0.5]);
title('\rm from w_{h1}')
grid on;
subplot(4,4,8)
plot(step_h.outputs_linear.Time, step_h.outputs_linear.Data(:,6), '--r', ...
    step_h.outputs_nonlinear.Time, step_h.outputs_nonlinear.Data(:,6), '-.b');
ylim(1e-3*[-0.5 0.5]);
grid on;
subplot(4,4,12)
plot(step_h.outputs_linear.Time, step_h.outputs_linear.Data(:,7), '--r', ...
    step_h.outputs_nonlinear.Time, step_h.outputs_nonlinear.Data(:,7), '-.b');
ylim(1e-3*[-0.5 0.5]);
grid on;
subplot(4,4,16)
plot(step_h.outputs_linear.Time, step_h.outputs_linear.Data(:,8), '--r', ...
    step_h.outputs_nonlinear.Time, step_h.outputs_nonlinear.Data(:,8), '-.b');
ylim([-11 -9]);
xlabel('Time [s]');
grid on;
set(gcf, 'Position',[383 42 850 960/1.5]);

%% plot outputs y1 mit stellgrößenbeschränkung theta und alle in einem 
f2 = figure;
subplot(4,4,1)
plot(step_v.step_linear.Time, step_v.step_linear.Data(:,1), ':k', ...
    step_v.outputs_linear.Time, step_v.outputs_linear.Data(:,1), '--r', ...
    step_v.outputs_nonlinear.Time, step_v.outputs_nonlinear.Data(:,1), '-.b');
legend('step', 'linear', 'nonlinear', 'Location', 'southeast')
ylim([0 1.2]);
ylabel('to v1');
title('\rm from w_{v1}')
grid on;
subplot(4,4,5)
plot(step_v.step_linear.Time, step_v.step_linear.Data(:,2), ':k', ...
    step_v.outputs_linear.Time, step_v.outputs_linear.Data(:,2), '--r', ...
    step_v.outputs_nonlinear.Time, step_v.outputs_nonlinear.Data(:,2), '-.b');
ylim(1e-3*[-0.5 0.5]);
ylabel('to \Phi1');
grid on;
subplot(4,4,9)
plot(step_v.outputs_linear.Time, step_v.outputs_linear.Data(:,3), '--r', ...
    step_v.outputs_nonlinear.Time, step_v.outputs_nonlinear.Data(:,3), '-.b');
ylim([-0.1 0]);
ylabel('to \Theta1');
grid on;
subplot(4,4,13)
plot(step_v.step_linear.Time, step_v.step_linear.Data(:,4), ':k', ...
    step_v.outputs_linear.Time, step_v.outputs_linear.Data(:,4), '--r', ...
    step_v.outputs_nonlinear.Time, step_v.outputs_nonlinear.Data(:,4), '-.b');
ylim([4995 5005]);
ylabel('to h1');
xlabel('Time [s]');
grid on;

subplot(4,4,2)
plot(step_phi.step_linear.Time, step_phi.step_linear.Data(:,1), ':k', ...
    step_phi.outputs_linear.Time, step_phi.outputs_linear.Data(:,1), '--r', ...
    step_phi.outputs_nonlinear.Time, step_phi.outputs_nonlinear.Data(:,1), '-.b');
ylim(1e-1*[-0.5 0.5]);
title('\rm from w_{\Phi1}')
grid on;
subplot(4,4,6)
plot(step_phi.step_linear.Time, step_phi.step_linear.Data(:,2), ':k', ...
    step_phi.outputs_linear.Time, step_phi.outputs_linear.Data(:,2), '--r', ...
    step_phi.outputs_nonlinear.Time, step_phi.outputs_nonlinear.Data(:,2), '-.b');
ylim([-0.05 0.3]);
grid on;
subplot(4,4,10)
plot(step_phi.outputs_linear.Time, step_phi.outputs_linear.Data(:,3), '--r', ...
    step_phi.outputs_nonlinear.Time, step_phi.outputs_nonlinear.Data(:,3), '-.b');
ylim([-0.1 0]);
grid on;
subplot(4,4,14)
plot(step_phi.step_linear.Time, step_phi.step_linear.Data(:,4), ':k', ...
    step_phi.outputs_linear.Time, step_phi.outputs_linear.Data(:,4), '--r', ...
    step_phi.outputs_nonlinear.Time, step_phi.outputs_nonlinear.Data(:,4), '-.b');
ylim([4995 5005]);
xlabel('Time [s]');
grid on;

subplot(4,4,3)
plot(step_theta.step_linear.Time, step_theta.step_linear.Data(:,1), ':k', ...
    step_theta.outputs_linear.Time, step_theta.outputs_linear.Data(:,1), '--r', ...
    step_theta.outputs_nonlinear.Time, step_theta.outputs_nonlinear.Data(:,1), '-.b');
ylim([-15 30]);
title('\rm from w_{\Theta1}')
grid on;
subplot(4,4,7)
plot(step_theta.step_linear.Time, step_theta.step_linear.Data(:,2), ':k', ...
    step_theta.outputs_linear.Time, step_theta.outputs_linear.Data(:,2), '--r', ...
    step_theta.outputs_nonlinear.Time, step_theta.outputs_nonlinear.Data(:,2), '-.b');
ylim([-1 5]);
grid on;
subplot(4,4,11)
plot(step_theta.step_linear.Time, step_theta.step_linear.Data(:,3), ':k', ...
    step_theta.outputs_linear.Time, step_theta.outputs_linear.Data(:,3), '--r', ...
    step_theta.outputs_nonlinear.Time, step_theta.outputs_nonlinear.Data(:,3), '-.b');
ylim([-2 3]);
grid on;
subplot(4,4,15)
plot(step_theta.step_linear.Time, step_theta.step_linear.Data(:,4), ':k', ...
    step_theta.outputs_linear.Time, step_theta.outputs_linear.Data(:,4), '--r', ...
    step_theta.outputs_nonlinear.Time, step_theta.outputs_nonlinear.Data(:,4), '-.b');
ylim([4000 9000]);
xlabel('Time [s]');
grid on;

subplot(4,4,4)
plot(step_h.step_linear.Time, step_h.step_linear.Data(:,1), ':k', ...
    step_h.outputs_linear.Time, step_h.outputs_linear.Data(:,1), '--r', ...
    step_h.outputs_nonlinear.Time, step_h.outputs_nonlinear.Data(:,1), '-.b');
ylim(1e-3*[-0.5 0.5]);
title('\rm from w_{h1}')
grid on;
subplot(4,4,8)
plot(step_h.step_linear.Time, step_h.step_linear.Data(:,2), ':k', ...
    step_h.outputs_linear.Time, step_h.outputs_linear.Data(:,2), '--r', ...
    step_h.outputs_nonlinear.Time, step_h.outputs_nonlinear.Data(:,2), '-.b');
ylim(1e-3*[-0.5 0.5]);
grid on;
subplot(4,4,12)
plot(step_h.outputs_linear.Time, step_h.outputs_linear.Data(:,3), '--r', ...
    step_h.outputs_nonlinear.Time, step_h.outputs_nonlinear.Data(:,3), '-.b');
ylim([-0.1 0]);
grid on;
subplot(4,4,16)
plot(step_h.step_linear.Time, step_h.step_linear.Data(:,4)+5000, ':k', ...
    step_h.outputs_linear.Time, step_h.outputs_linear.Data(:,4), '--r', ...
    step_h.outputs_nonlinear.Time, step_h.outputs_nonlinear.Data(:,4), '-.b');
ylim([4995 5010]);
xlabel('Time [s]');
grid on;
set(gcf, 'Position',[383 42 850 960/1.5]);


% beide in einem Plot
f_both = figure;
subplot(8,4,1)
plot(step_v.step_linear.Time, step_v.step_linear.Data(:,1), ':k', ...
    step_v.outputs_linear.Time, step_v.outputs_linear.Data(:,1), '--r', ...
    step_v.outputs_nonlinear.Time, step_v.outputs_nonlinear.Data(:,1), '-.b');
legend('step', 'linear', 'nonlinear', 'Location', 'southeast')
ylim([0 1.2]);
ylabel('to v1');
title('\rm from w_{v1}')
grid on;
subplot(8,4,5)
plot(step_v.step_linear.Time, step_v.step_linear.Data(:,2), ':k', ...
    step_v.outputs_linear.Time, step_v.outputs_linear.Data(:,2), '--r', ...
    step_v.outputs_nonlinear.Time, step_v.outputs_nonlinear.Data(:,2), '-.b');
ylim(1e-3*[-0.5 0.5]);
ylabel('to \Phi1');
grid on;
subplot(8,4,9)
plot(step_v.outputs_linear.Time, step_v.outputs_linear.Data(:,3), '--r', ...
    step_v.outputs_nonlinear.Time, step_v.outputs_nonlinear.Data(:,3), '-.b');
ylim([-0.1 0]);
ylabel('to \Theta1');
grid on;
subplot(8,4,13)
plot(step_v.step_linear.Time, step_v.step_linear.Data(:,4), ':k', ...
    step_v.outputs_linear.Time, step_v.outputs_linear.Data(:,4), '--r', ...
    step_v.outputs_nonlinear.Time, step_v.outputs_nonlinear.Data(:,4), '-.b');
ylim([4995 5005]);
ylabel('to h1');
grid on;
subplot(8,4,17)
plot(step_v.outputs_linear.Time, step_v.outputs_linear.Data(:,5), '--r', ...
    step_v.outputs_nonlinear.Time, step_v.outputs_nonlinear.Data(:,5), '-.b');
ylim(1e-3*[-0.5 0.5]);
ylabel('to \Delta v');
grid on;
subplot(8,4,21)
plot(step_v.outputs_linear.Time, step_v.outputs_linear.Data(:,6), '--r', ...
    step_v.outputs_nonlinear.Time, step_v.outputs_nonlinear.Data(:,6), '-.b');
ylim(1e-3*[-0.5 0.5]);
ylabel('to \Delta \Phi');
grid on;
subplot(8,4,25)
plot(step_v.outputs_linear.Time, step_v.outputs_linear.Data(:,7), '--r', ...
    step_v.outputs_nonlinear.Time, step_v.outputs_nonlinear.Data(:,7), '-.b');
ylim(1e-3*[-0.5 0.5]);
ylabel('to \Delta \Theta');
grid on;
subplot(8,4,29)
plot(step_v.outputs_linear.Time, step_v.outputs_linear.Data(:,8), '--r', ...
    step_v.outputs_nonlinear.Time, step_v.outputs_nonlinear.Data(:,8), '-.b');
ylim([-11 -9]);
ylabel('to \Delta h');
xlabel('Time [s]');
grid on;

subplot(8,4,2)
plot(step_phi.step_linear.Time, step_phi.step_linear.Data(:,1), ':k', ...
    step_phi.outputs_linear.Time, step_phi.outputs_linear.Data(:,1), '--r', ...
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
plot(step_phi.outputs_linear.Time, step_phi.outputs_linear.Data(:,3), '--r', ...
    step_phi.outputs_nonlinear.Time, step_phi.outputs_nonlinear.Data(:,3), '-.b');
ylim([-0.1 0]);
grid on;
subplot(8,4,14)
plot(step_phi.step_linear.Time, step_phi.step_linear.Data(:,4), ':k', ...
    step_phi.outputs_linear.Time, step_phi.outputs_linear.Data(:,4), '--r', ...
    step_phi.outputs_nonlinear.Time, step_phi.outputs_nonlinear.Data(:,4), '-.b');
ylim([4995 5005]);
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
plot(step_phi.outputs_linear.Time, step_phi.outputs_linear.Data(:,7), '--r', ...
    step_phi.outputs_nonlinear.Time, step_phi.outputs_nonlinear.Data(:,7), '-.b');
ylim(1e-3*[-0.5 0.5]);
grid on;
subplot(8,4,30)
plot(step_phi.outputs_linear.Time, step_phi.outputs_linear.Data(:,8), '--r', ...
    step_phi.outputs_nonlinear.Time, step_phi.outputs_nonlinear.Data(:,8), '-.b');
ylim([-11 -9]);
xlabel('Time [s]');
grid on;

subplot(8,4,3)
plot(step_theta.step_linear.Time, step_theta.step_linear.Data(:,1), ':k', ...
    step_theta.outputs_linear.Time, step_theta.outputs_linear.Data(:,1), '--r', ...
    step_theta.outputs_nonlinear.Time, step_theta.outputs_nonlinear.Data(:,1), '-.b');
ylim([-15 30]);
title('\rm from w_{\Theta1}')
grid on;
subplot(8,4,7)
plot(step_theta.step_linear.Time, step_theta.step_linear.Data(:,2), ':k', ...
    step_theta.outputs_linear.Time, step_theta.outputs_linear.Data(:,2), '--r', ...
    step_theta.outputs_nonlinear.Time, step_theta.outputs_nonlinear.Data(:,2), '-.b');
ylim([-1 5]);
grid on;
subplot(8,4,11)
plot(step_theta.step_linear.Time, step_theta.step_linear.Data(:,3), ':k', ...
    step_theta.outputs_linear.Time, step_theta.outputs_linear.Data(:,3), '--r', ...
    step_theta.outputs_nonlinear.Time, step_theta.outputs_nonlinear.Data(:,3), '-.b');
ylim([-2 3]);
grid on;
subplot(8,4,15)
plot(step_theta.step_linear.Time, step_theta.step_linear.Data(:,4), ':k', ...
    step_theta.outputs_linear.Time, step_theta.outputs_linear.Data(:,4), '--r', ...
    step_theta.outputs_nonlinear.Time, step_theta.outputs_nonlinear.Data(:,4), '-.b');
ylim([4000 9000]);
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
plot(step_theta.outputs_linear.Time, step_theta.outputs_linear.Data(:,7), '--r', ...
    step_theta.outputs_nonlinear.Time, step_theta.outputs_nonlinear.Data(:,7), '-.b');
ylim([-0.5 0.5]);
grid on;
subplot(8,4,31)
plot(step_theta.outputs_linear.Time, step_theta.outputs_linear.Data(:,8), '--r', ...
    step_theta.outputs_nonlinear.Time, step_theta.outputs_nonlinear.Data(:,8), '-.b');
ylim([-15 25]);
xlabel('Time [s]');
grid on;

subplot(8,4,4)
plot(step_h.step_linear.Time, step_h.step_linear.Data(:,1), ':k', ...
    step_h.outputs_linear.Time, step_h.outputs_linear.Data(:,1), '--r', ...
    step_h.outputs_nonlinear.Time, step_h.outputs_nonlinear.Data(:,1), '-.b');
ylim(1e-3*[-0.5 0.5]);
title('\rm from w_{h1}')
grid on;
subplot(8,4,8)
plot(step_h.step_linear.Time, step_h.step_linear.Data(:,2), ':k', ...
    step_h.outputs_linear.Time, step_h.outputs_linear.Data(:,2), '--r', ...
    step_h.outputs_nonlinear.Time, step_h.outputs_nonlinear.Data(:,2), '-.b');
ylim(1e-3*[-0.5 0.5]);
grid on;
subplot(8,4,12)
plot(step_h.outputs_linear.Time, step_h.outputs_linear.Data(:,3), '--r', ...
    step_h.outputs_nonlinear.Time, step_h.outputs_nonlinear.Data(:,3), '-.b');
ylim([-0.1 0]);
grid on;
subplot(8,4,16)
plot(step_h.step_linear.Time, step_h.step_linear.Data(:,4)+5000, ':k', ...
    step_h.outputs_linear.Time, step_h.outputs_linear.Data(:,4), '--r', ...
    step_h.outputs_nonlinear.Time, step_h.outputs_nonlinear.Data(:,4), '-.b');
ylim([4995 5010]);
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
plot(step_h.outputs_linear.Time, step_h.outputs_linear.Data(:,7), '--r', ...
    step_h.outputs_nonlinear.Time, step_h.outputs_nonlinear.Data(:,7), '-.b');
ylim(1e-3*[-0.5 0.5]);
grid on;
subplot(8,4,32)
plot(step_h.outputs_linear.Time, step_h.outputs_linear.Data(:,8), '--r', ...
    step_h.outputs_nonlinear.Time, step_h.outputs_nonlinear.Data(:,8), '-.b');
ylim([-11 -9]);
xlabel('Time [s]');
grid on;
set(gcf, 'Position',[383 42 850 960]);

%% plot coupled outputs y2 ohne stellgrößenbeschränkung theta 
f3 = figure;
subplot(4,4,1)
plot(step_v.outputs_linear.Time, step_v.outputs_linear.Data(:,5), '--r', ...
    step_v.outputs_nonlinear.Time, step_v.outputs_nonlinear.Data(:,5), '-.b');
legend('linear', 'nonlinear', 'Location', 'southeast')
ylim(1e-3*[-0.5 0.5]);
ylabel('to \Delta v');
title('\rm from w_{v1}')
grid on;
subplot(4,4,5)
plot(step_v.outputs_linear.Time, step_v.outputs_linear.Data(:,6), '--r', ...
    step_v.outputs_nonlinear.Time, step_v.outputs_nonlinear.Data(:,6), '-.b');
ylim(1e-3*[-0.5 0.5]);
ylabel('to \Delta \Phi');
grid on;
subplot(4,4,9)
plot(step_v.outputs_linear.Time, step_v.outputs_linear.Data(:,7), '--r', ...
    step_v.outputs_nonlinear.Time, step_v.outputs_nonlinear.Data(:,7), '-.b');
ylim(1e-3*[-0.5 0.5]);
ylabel('to \Delta \Theta');
grid on;
subplot(4,4,13)
plot(step_v.outputs_linear.Time, step_v.outputs_linear.Data(:,8), '--r', ...
    step_v.outputs_nonlinear.Time, step_v.outputs_nonlinear.Data(:,8), '-.b');
ylim([-11 -9]);
ylabel('to \Delta h');
xlabel('Time [s]');
grid on;

subplot(4,4,2)
plot(step_phi.outputs_linear.Time, step_phi.outputs_linear.Data(:,5), '--r', ...
    step_phi.outputs_nonlinear.Time, step_phi.outputs_nonlinear.Data(:,5), '-.b');
ylim(1e-3*[-0.5 0.5]);
title('\rm from w_{\Phi1}')
grid on;
subplot(4,4,6)
plot(step_phi.outputs_linear.Time, step_phi.outputs_linear.Data(:,6), '--r', ...
    step_phi.outputs_nonlinear.Time, step_phi.outputs_nonlinear.Data(:,6), '-.b');
ylim(1e-3*[-0.5 0.5]);
grid on;
subplot(4,4,10)
plot(step_phi.outputs_linear.Time, step_phi.outputs_linear.Data(:,7), '--r', ...
    step_phi.outputs_nonlinear.Time, step_phi.outputs_nonlinear.Data(:,7), '-.b');
ylim(1e-3*[-0.5 0.5]);
grid on;
subplot(4,4,14)
plot(step_phi.outputs_linear.Time, step_phi.outputs_linear.Data(:,8), '--r', ...
    step_phi.outputs_nonlinear.Time, step_phi.outputs_nonlinear.Data(:,8), '-.b');
ylim([-11 -9]);
xlabel('Time [s]');
grid on;

subplot(4,4,3)
plot(step_theta_ohne_stell.outputs_linear.Time, step_theta_ohne_stell.outputs_linear.Data(:,5), '--r', ...
    step_theta_ohne_stell.outputs_nonlinear.Time, step_theta_ohne_stell.outputs_nonlinear.Data(:,5), '-.b');
ylim([-0.5 0.5]);
title('\rm from w_{\Theta1}')
grid on;
subplot(4,4,7)
plot(step_theta_ohne_stell.outputs_linear.Time, step_theta_ohne_stell.outputs_linear.Data(:,6), '--r', ...
    step_theta_ohne_stell.outputs_nonlinear.Time, step_theta_ohne_stell.outputs_nonlinear.Data(:,6), '-.b');
ylim([-1 4]);
grid on;
subplot(4,4,11)
plot(step_theta_ohne_stell.outputs_linear.Time, step_theta_ohne_stell.outputs_linear.Data(:,7), '--r', ...
    step_theta_ohne_stell.outputs_nonlinear.Time, step_theta_ohne_stell.outputs_nonlinear.Data(:,7), '-.b');
ylim([-0.5 0.5]);
grid on;
subplot(4,4,15)
plot(step_theta_ohne_stell.outputs_linear.Time, step_theta_ohne_stell.outputs_linear.Data(:,8), '--r', ...
    step_theta_ohne_stell.outputs_nonlinear.Time, step_theta_ohne_stell.outputs_nonlinear.Data(:,8), '-.b');
ylim([-11 -9]);
xlabel('Time [s]');
grid on;

subplot(4,4,4)
plot(step_h.outputs_linear.Time, step_h.outputs_linear.Data(:,5), '--r', ...
    step_h.outputs_nonlinear.Time, step_h.outputs_nonlinear.Data(:,5), '-.b');
ylim(1e-3*[-0.5 0.5]);
title('\rm from w_{h1}')
grid on;
subplot(4,4,8)
plot(step_h.outputs_linear.Time, step_h.outputs_linear.Data(:,6), '--r', ...
    step_h.outputs_nonlinear.Time, step_h.outputs_nonlinear.Data(:,6), '-.b');
ylim(1e-3*[-0.5 0.5]);
grid on;
subplot(4,4,12)
plot(step_h.outputs_linear.Time, step_h.outputs_linear.Data(:,7), '--r', ...
    step_h.outputs_nonlinear.Time, step_h.outputs_nonlinear.Data(:,7), '-.b');
ylim(1e-3*[-0.5 0.5]);
grid on;
subplot(4,4,16)
plot(step_h.outputs_linear.Time, step_h.outputs_linear.Data(:,8), '--r', ...
    step_h.outputs_nonlinear.Time, step_h.outputs_nonlinear.Data(:,8), '-.b');
ylim([-11 -9]);
xlabel('Time [s]');
grid on;
set(gcf, 'Position',[383 42 850 960/1.5]);

%% plot outputs y1 ohne stellgrößenbeschränkung theta
f4 = figure;
subplot(4,4,1)
plot(step_v.step_linear.Time, step_v.step_linear.Data(:,1), ':k', ...
    step_v.outputs_linear.Time, step_v.outputs_linear.Data(:,1), '--r', ...
    step_v.outputs_nonlinear.Time, step_v.outputs_nonlinear.Data(:,1), '-.b');
legend('step', 'linear', 'nonlinear', 'Location', 'southeast')
ylim([0 1.2]);
ylabel('to v1');
title('\rm from w_{v1}')
grid on;
subplot(4,4,5)
plot(step_v.step_linear.Time, step_v.step_linear.Data(:,2), ':k', ...
    step_v.outputs_linear.Time, step_v.outputs_linear.Data(:,2), '--r', ...
    step_v.outputs_nonlinear.Time, step_v.outputs_nonlinear.Data(:,2), '-.b');
ylim(1e-3*[-0.5 0.5]);
ylabel('to \Phi1');
grid on;
subplot(4,4,9)
plot(step_v.outputs_linear.Time, step_v.outputs_linear.Data(:,3), '--r', ...
    step_v.outputs_nonlinear.Time, step_v.outputs_nonlinear.Data(:,3), '-.b');
ylim([-0.1 0]);
ylabel('to \Theta1');
grid on;
subplot(4,4,13)
plot(step_v.step_linear.Time, step_v.step_linear.Data(:,4), ':k', ...
    step_v.outputs_linear.Time, step_v.outputs_linear.Data(:,4), '--r', ...
    step_v.outputs_nonlinear.Time, step_v.outputs_nonlinear.Data(:,4), '-.b');
ylim([4995 5005]);
ylabel('to h1');
xlabel('Time [s]');
grid on;

subplot(4,4,2)
plot(step_phi.step_linear.Time, step_phi.step_linear.Data(:,1), ':k', ...
    step_phi.outputs_linear.Time, step_phi.outputs_linear.Data(:,1), '--r', ...
    step_phi.outputs_nonlinear.Time, step_phi.outputs_nonlinear.Data(:,1), '-.b');
ylim(1e-1*[-0.5 0.5]);
title('\rm from w_{\Phi1}')
grid on;
subplot(4,4,6)
plot(step_phi.step_linear.Time, step_phi.step_linear.Data(:,2), ':k', ...
    step_phi.outputs_linear.Time, step_phi.outputs_linear.Data(:,2), '--r', ...
    step_phi.outputs_nonlinear.Time, step_phi.outputs_nonlinear.Data(:,2), '-.b');
ylim([-0.05 0.3]);
grid on;
subplot(4,4,10)
plot(step_phi.outputs_linear.Time, step_phi.outputs_linear.Data(:,3), '--r', ...
    step_phi.outputs_nonlinear.Time, step_phi.outputs_nonlinear.Data(:,3), '-.b');
ylim([-0.1 0]);
grid on;
subplot(4,4,14)
plot(step_phi.step_linear.Time, step_phi.step_linear.Data(:,4), ':k', ...
    step_phi.outputs_linear.Time, step_phi.outputs_linear.Data(:,4), '--r', ...
    step_phi.outputs_nonlinear.Time, step_phi.outputs_nonlinear.Data(:,4), '-.b');
ylim([4995 5005]);
xlabel('Time [s]');
grid on;

subplot(4,4,3)
plot(step_theta_ohne_stell.step_linear.Time, step_theta_ohne_stell.step_linear.Data(:,1), ':k', ...
    step_theta_ohne_stell.outputs_linear.Time, step_theta_ohne_stell.outputs_linear.Data(:,1), '--r', ...
    step_theta_ohne_stell.outputs_nonlinear.Time, step_theta_ohne_stell.outputs_nonlinear.Data(:,1), '-.b');
ylim([-15 30]);
title('\rm from w_{\Theta1}')
grid on;
subplot(4,4,7)
plot(step_theta_ohne_stell.step_linear.Time, step_theta_ohne_stell.step_linear.Data(:,2), ':k', ...
    step_theta_ohne_stell.outputs_linear.Time, step_theta_ohne_stell.outputs_linear.Data(:,2), '--r', ...
    step_theta_ohne_stell.outputs_nonlinear.Time, step_theta_ohne_stell.outputs_nonlinear.Data(:,2), '-.b');
ylim([-1 5]);
grid on;
subplot(4,4,11)
plot(step_theta_ohne_stell.step_linear.Time, step_theta_ohne_stell.step_linear.Data(:,3), ':k', ...
    step_theta_ohne_stell.outputs_linear.Time, step_theta_ohne_stell.outputs_linear.Data(:,3), '--r', ...
    step_theta_ohne_stell.outputs_nonlinear.Time, step_theta_ohne_stell.outputs_nonlinear.Data(:,3), '-.b');
ylim([-2 3]);
grid on;
subplot(4,4,15)
plot(step_theta_ohne_stell.step_linear.Time, step_theta_ohne_stell.step_linear.Data(:,4), ':k', ...
    step_theta_ohne_stell.outputs_linear.Time, step_theta_ohne_stell.outputs_linear.Data(:,4), '--r', ...
    step_theta_ohne_stell.outputs_nonlinear.Time, step_theta_ohne_stell.outputs_nonlinear.Data(:,4), '-.b');
ylim([4000 7000]);
grid on;
xlabel('Time [s]');

subplot(4,4,4)
plot(step_h.step_linear.Time, step_h.step_linear.Data(:,1), ':k', ...
    step_h.outputs_linear.Time, step_h.outputs_linear.Data(:,1), '--r', ...
    step_h.outputs_nonlinear.Time, step_h.outputs_nonlinear.Data(:,1), '-.b');
ylim(1e-3*[-0.5 0.5]);
title('\rm from w_{h1}')
grid on;
subplot(4,4,8)
plot(step_h.step_linear.Time, step_h.step_linear.Data(:,2), ':k', ...
    step_h.outputs_linear.Time, step_h.outputs_linear.Data(:,2), '--r', ...
    step_h.outputs_nonlinear.Time, step_h.outputs_nonlinear.Data(:,2), '-.b');
ylim(1e-3*[-0.5 0.5]);
grid on;
subplot(4,4,12)
plot(step_h.outputs_linear.Time, step_h.outputs_linear.Data(:,3), '--r', ...
    step_h.outputs_nonlinear.Time, step_h.outputs_nonlinear.Data(:,3), '-.b');
ylim([-0.1 0]);
grid on;
subplot(4,4,16)
plot(step_h.step_linear.Time, step_h.step_linear.Data(:,4)+5000, ':k', ...
    step_h.outputs_linear.Time, step_h.outputs_linear.Data(:,4), '--r', ...
    step_h.outputs_nonlinear.Time, step_h.outputs_nonlinear.Data(:,4), '-.b');
ylim([4995 5010]);
xlabel('Time [s]');
grid on;
set(gcf, 'Position',[383 42 850 960/1.5]);

%% plot stellgrößen
f5 = figure;
f5.Renderer = 'painters';
subplot(4,4,1)
plot(step_v.U_ist_linear.Time, step_v.U_ist_linear.Data(:,1), '--r', ...
    step_v.U_ist_linear.Time, step_v.U_ist_linear.Data(:,5), '--b', ...
    step_v.U_ist_nonlinear.Time, step_v.U_ist_nonlinear.Data(:,1), '-.r', ...
    step_v.U_ist_nonlinear.Time, step_v.U_ist_nonlinear.Data(:,5), '-.b', ...
    step_v.U_ist_linear.Time, ones(size(step_v.U_ist_linear.Time))*eta_min, ':k', ...
    step_v.U_ist_linear.Time, ones(size(step_v.U_ist_linear.Time))*eta_max, ':k');
ylim([-0.5 0.25]);
xlim([0 1.5]);
ylabel('to \eta [rad]');
title('\rm from w_{v1}')
grid on;
subplot(4,4,5)
plot(step_v.U_ist_linear.Time, step_v.U_ist_linear.Data(:,3), '--r', ...
    step_v.U_ist_linear.Time, step_v.U_ist_linear.Data(:,7), '--b', ...
    step_v.U_ist_nonlinear.Time, step_v.U_ist_nonlinear.Data(:,3), '-.r', ...
    step_v.U_ist_nonlinear.Time, step_v.U_ist_nonlinear.Data(:,7), '-.b', ...
    step_v.U_ist_linear.Time, ones(size(step_v.U_ist_linear.Time))*xi_min, ':k', ...
    step_v.U_ist_linear.Time, ones(size(step_v.U_ist_linear.Time))*xi_max, ':k');
ylim([-0.5 0.5]);
xlim([0 1.5]);
ylabel('to \xi [rad]');
grid on;
subplot(4,4,9)
plot(step_v.U_ist_linear.Time, step_v.U_ist_linear.Data(:,4), '--r', ...
    step_v.U_ist_linear.Time, step_v.U_ist_linear.Data(:,8), '--b', ...
    step_v.U_ist_nonlinear.Time, step_v.U_ist_nonlinear.Data(:,4), '-.r', ...
    step_v.U_ist_nonlinear.Time, step_v.U_ist_nonlinear.Data(:,8), '-.b', ...
    step_v.U_ist_linear.Time, ones(size(step_v.U_ist_linear.Time))*zita_min, ':k', ...
    step_v.U_ist_linear.Time, ones(size(step_v.U_ist_linear.Time))*zita_max, ':k');
ylim([-0.6 0.6]);
xlim([0 1.5]);
ylabel('to \zeta [rad]');
grid on;
subplot(4,4,13)
plot(step_v.U_ist_linear.Time, step_v.U_ist_linear.Data(:,2), '--r', ...
    step_v.U_ist_linear.Time, step_v.U_ist_linear.Data(:,6), '--b', ...
    step_v.U_ist_nonlinear.Time, step_v.U_ist_nonlinear.Data(:,2), '-.r', ...
    step_v.U_ist_nonlinear.Time, step_v.U_ist_nonlinear.Data(:,6), '-.b', ...
    step_v.U_ist_linear.Time, ones(size(step_v.U_ist_linear.Time))*sigmaf_min, ':k', ...
    step_v.U_ist_linear.Time, ones(size(step_v.U_ist_linear.Time))*sigmaf_max, ':k');
ylim([-0.05 0.4]);
xlim([0 1.5]);
ylabel('to \sigma_f');
grid on;

subplot(4,4,2)
plot(step_phi.U_ist_linear.Time, step_phi.U_ist_linear.Data(:,1), '--r', ...
    step_phi.U_ist_linear.Time, step_phi.U_ist_linear.Data(:,5), '--b', ...
    step_phi.U_ist_nonlinear.Time, step_phi.U_ist_nonlinear.Data(:,1), '-.r', ...
    step_phi.U_ist_nonlinear.Time, step_phi.U_ist_nonlinear.Data(:,5), '-.b', ...
    step_v.U_ist_linear.Time, ones(size(step_v.U_ist_linear.Time))*eta_min, ':k', ...
    step_v.U_ist_linear.Time, ones(size(step_v.U_ist_linear.Time))*eta_max, ':k');
ylim([-0.5 0.25]);
xlim([0 5]);
title('\rm from w_{\Phi1}')
grid on;
subplot(4,4,6)
plot(step_phi.U_ist_linear.Time, step_phi.U_ist_linear.Data(:,3), '--r', ...
    step_phi.U_ist_linear.Time, step_phi.U_ist_linear.Data(:,7), '--b', ...
    step_phi.U_ist_nonlinear.Time, step_phi.U_ist_nonlinear.Data(:,3), '-.r', ...
    step_phi.U_ist_nonlinear.Time, step_phi.U_ist_nonlinear.Data(:,7), '-.b', ...
    step_v.U_ist_linear.Time, ones(size(step_v.U_ist_linear.Time))*xi_min, ':k', ...
    step_v.U_ist_linear.Time, ones(size(step_v.U_ist_linear.Time))*xi_max, ':k');
ylim([-0.5 0.5]);
xlim([0 5]);
grid on;
subplot(4,4,10)
plot(step_phi.U_ist_linear.Time, step_phi.U_ist_linear.Data(:,4), '--r', ...
    step_phi.U_ist_linear.Time, step_phi.U_ist_linear.Data(:,8), '--b', ...
    step_phi.U_ist_nonlinear.Time, step_phi.U_ist_nonlinear.Data(:,4), '-.r', ...
    step_phi.U_ist_nonlinear.Time, step_phi.U_ist_nonlinear.Data(:,8), '-.b', ...
    step_v.U_ist_linear.Time, ones(size(step_v.U_ist_linear.Time))*zita_min, ':k', ...
    step_v.U_ist_linear.Time, ones(size(step_v.U_ist_linear.Time))*zita_max, ':k');
ylim([-0.6 0.6]);
xlim([0 5]);
grid on;
subplot(4,4,14)
plot(step_phi.U_ist_linear.Time, step_phi.U_ist_linear.Data(:,2), '--r', ...
    step_phi.U_ist_linear.Time, step_phi.U_ist_linear.Data(:,6), '--b', ...
    step_phi.U_ist_nonlinear.Time, step_phi.U_ist_nonlinear.Data(:,2), '-.r', ...
    step_phi.U_ist_nonlinear.Time, step_phi.U_ist_nonlinear.Data(:,6), '-.b', ...
    step_v.U_ist_linear.Time, ones(size(step_v.U_ist_linear.Time))*sigmaf_min, ':k', ...
    step_v.U_ist_linear.Time, ones(size(step_v.U_ist_linear.Time))*sigmaf_max, ':k');
ylim([-0.05 0.4]);
xlim([0 5]);
grid on;

subplot(4,4,3)
plot(step_theta.U_ist_linear.Time, step_theta.U_ist_linear.Data(:,1), '--r', ...
    step_theta.U_ist_linear.Time, step_theta.U_ist_linear.Data(:,5), '--b', ...
    step_theta.U_ist_nonlinear.Time, step_theta.U_ist_nonlinear.Data(:,1), '-.r', ...
    step_theta.U_ist_nonlinear.Time, step_theta.U_ist_nonlinear.Data(:,5), '-.b', ...
    step_v.U_ist_linear.Time, ones(size(step_v.U_ist_linear.Time))*eta_min, ':k', ...
    step_v.U_ist_linear.Time, ones(size(step_v.U_ist_linear.Time))*eta_max, ':k');
ylim([-0.5 0.25]);
xlim([0 5]);
title('\rm from w_{\Theta1}')
grid on;
subplot(4,4,7)
plot(step_theta.U_ist_linear.Time, step_theta.U_ist_linear.Data(:,3), '--r', ...
    step_theta.U_ist_linear.Time, step_theta.U_ist_linear.Data(:,7), '--b', ...
    step_theta.U_ist_nonlinear.Time, step_theta.U_ist_nonlinear.Data(:,3), '-.r', ...
    step_theta.U_ist_nonlinear.Time, step_theta.U_ist_nonlinear.Data(:,7), '-.b', ...
    step_v.U_ist_linear.Time, ones(size(step_v.U_ist_linear.Time))*xi_min, ':k', ...
    step_v.U_ist_linear.Time, ones(size(step_v.U_ist_linear.Time))*xi_max, ':k');
ylim([-0.5 0.5]);
xlim([0 5]);
grid on;
subplot(4,4,11)
plot(step_theta.U_ist_linear.Time, step_theta.U_ist_linear.Data(:,4), '--r', ...
    step_theta.U_ist_linear.Time, step_theta.U_ist_linear.Data(:,8), '--b', ...
    step_theta.U_ist_nonlinear.Time, step_theta.U_ist_nonlinear.Data(:,4), '-.r', ...
    step_theta.U_ist_nonlinear.Time, step_theta.U_ist_nonlinear.Data(:,8), '-.b', ...
    step_v.U_ist_linear.Time, ones(size(step_v.U_ist_linear.Time))*zita_min, ':k', ...
    step_v.U_ist_linear.Time, ones(size(step_v.U_ist_linear.Time))*zita_max, ':k');
ylim([-0.6 0.6]);
xlim([0 5]);
grid on;
subplot(4,4,15)
plot(step_theta.U_ist_linear.Time, step_theta.U_ist_linear.Data(:,2), '--r', ...
    step_theta.U_ist_linear.Time, step_theta.U_ist_linear.Data(:,6), '--b', ...
    step_theta.U_ist_nonlinear.Time, step_theta.U_ist_nonlinear.Data(:,2), '-.r', ...
    step_theta.U_ist_nonlinear.Time, step_theta.U_ist_nonlinear.Data(:,6), '-.b', ...
    step_v.U_ist_linear.Time, ones(size(step_v.U_ist_linear.Time))*sigmaf_min, ':k', ...
    step_v.U_ist_linear.Time, ones(size(step_v.U_ist_linear.Time))*sigmaf_max, ':k');
ylim([-0.05 0.4]);
xlim([0 5]);
grid on;

subplot(4,4,4)
plot(step_h.U_ist_linear.Time, step_h.U_ist_linear.Data(:,1), '--r', ...
    step_h.U_ist_linear.Time, step_h.U_ist_linear.Data(:,5), '--b', ...
    step_h.U_ist_nonlinear.Time, step_h.U_ist_nonlinear.Data(:,1), '-.r', ...
    step_h.U_ist_nonlinear.Time, step_h.U_ist_nonlinear.Data(:,5), '-.b', ...
    step_v.U_ist_linear.Time, ones(size(step_v.U_ist_linear.Time))*eta_min, ':k', ...
    step_v.U_ist_linear.Time, ones(size(step_v.U_ist_linear.Time))*eta_max, ':k');
ylim([-0.5 0.25]);
xlim([0 5]);
legend('linear plane1', 'linear plane2', 'nonlinear plane1', 'nonlinear plane2', 'Stellgrößenbeschränkung', 'Location', 'northeast')
title('\rm from w_{h1}')
grid on;
subplot(4,4,8)
plot(step_h.U_ist_linear.Time, step_h.U_ist_linear.Data(:,3), '--r', ...
    step_h.U_ist_linear.Time, step_h.U_ist_linear.Data(:,7), '--b', ...
    step_h.U_ist_nonlinear.Time, step_h.U_ist_nonlinear.Data(:,3), '-.r', ...
    step_h.U_ist_nonlinear.Time, step_h.U_ist_nonlinear.Data(:,7), '-.b', ...
    step_v.U_ist_linear.Time, ones(size(step_v.U_ist_linear.Time))*xi_min, ':k', ...
    step_v.U_ist_linear.Time, ones(size(step_v.U_ist_linear.Time))*xi_max, ':k');
ylim([-0.5 0.5]);
xlim([0 5]);
grid on;
subplot(4,4,12)
plot(step_h.U_ist_linear.Time, step_h.U_ist_linear.Data(:,4), '--r', ...
    step_h.U_ist_linear.Time, step_h.U_ist_linear.Data(:,8), '--b', ...
    step_h.U_ist_nonlinear.Time, step_h.U_ist_nonlinear.Data(:,4), '-.r', ...
    step_h.U_ist_nonlinear.Time, step_h.U_ist_nonlinear.Data(:,8), '-.b', ...
    step_v.U_ist_linear.Time, ones(size(step_v.U_ist_linear.Time))*zita_min, ':k', ...
    step_v.U_ist_linear.Time, ones(size(step_v.U_ist_linear.Time))*zita_max, ':k');
ylim([-0.6 0.6]);
xlim([0 5]);
grid on;
subplot(4,4,16)
plot(step_h.U_ist_linear.Time, step_h.U_ist_linear.Data(:,2), '--r', ...
    step_h.U_ist_linear.Time, step_h.U_ist_linear.Data(:,6), '--b', ...
    step_h.U_ist_nonlinear.Time, step_h.U_ist_nonlinear.Data(:,2), '-.r', ...
    step_h.U_ist_nonlinear.Time, step_h.U_ist_nonlinear.Data(:,6), '-.b', ...
    step_v.U_ist_linear.Time, ones(size(step_v.U_ist_linear.Time))*sigmaf_min, ':k', ...
    step_v.U_ist_linear.Time, ones(size(step_v.U_ist_linear.Time))*sigmaf_max, ':k');
ylim([-0.05 0.4]);
xlim([0 5]);
grid on;
set(gcf, 'Position',[383 42 850 960/1.5]);

%% plot y1 and coupled output plus u ohne stellgrößenbeschränkung nur lineares system
f6 = figure;
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
plot(step_v.outputs_linear.Time, step_v.outputs_linear.Data(:,3), '--r');
ylim([-0.1 0]);
ylabel('to \Theta1 [rad]');
grid on;
subplot(8,4,13)
plot(step_v.outputs_linear.Time, step_v.outputs_linear.Data(:,4), '--r');
ylim([4995 5005]);
ylabel('to h1 [m]');
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
plot(step_v.outputs_linear.Time, step_v.outputs_linear.Data(:,7), '--r');
ylim(1e-3*[-0.5 0.5]);
ylabel('to \Delta \Theta [rad]');
grid on;
subplot(8,4,29)
plot(step_v.outputs_linear.Time, step_v.outputs_linear.Data(:,8), '--r');
ylim([-11 -9]);
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
plot(step_phi.outputs_linear.Time, step_phi.outputs_linear.Data(:,3), '--r');
ylim([-0.1 0]);
grid on;
subplot(8,4,14)
plot(step_phi.outputs_linear.Time, step_phi.outputs_linear.Data(:,4), '--r');
ylim([4995 5005]);
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
plot(step_phi.outputs_linear.Time, step_phi.outputs_linear.Data(:,7), '--r');
ylim(1e-3*[-0.5 0.5]);
grid on;
subplot(8,4,30)
plot(step_phi.outputs_linear.Time, step_phi.outputs_linear.Data(:,8), '--r');
ylim([-11 -9]);
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
plot(step_theta_ohne_stell.step_linear.Time, step_theta_ohne_stell.step_linear.Data(:,3)-0.0734, ':k', ...
    step_theta_ohne_stell.outputs_linear.Time, step_theta_ohne_stell.outputs_linear.Data(:,3), '--r');
ylim([-0.1 0.2]);
grid on;
subplot(8,4,15)
plot(step_theta_ohne_stell.outputs_linear.Time, step_theta_ohne_stell.outputs_linear.Data(:,4), '--r');
ylim([4980 5005]);
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
plot(step_theta_ohne_stell.outputs_linear.Time, step_theta_ohne_stell.outputs_linear.Data(:,7), '--r');
ylim(1e-3*[-0.5 0.5]);
grid on;
subplot(8,4,31)
plot(step_theta_ohne_stell.outputs_linear.Time, step_theta_ohne_stell.outputs_linear.Data(:,8), '--r');
ylim([-11 -9]);
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
plot(step_h.outputs_linear.Time, step_h.outputs_linear.Data(:,3), '--r');
ylim([-0.1 0]);
grid on;
subplot(8,4,16)
plot(step_h.step_linear.Time, step_h.step_linear.Data(:,4)+5000, ':k', ...
    step_h.outputs_linear.Time, step_h.outputs_linear.Data(:,4), '--r');
ylim([4995 5010]);
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
plot(step_h.outputs_linear.Time, step_h.outputs_linear.Data(:,7), '--r');
ylim(1e-3*[-0.5 0.5]);
grid on;
subplot(8,4,32)
plot(step_h.outputs_linear.Time, step_h.outputs_linear.Data(:,8), '--r');
ylim([-11 -9]);
xlabel('Time [s]');
grid on;
set(gcf, 'Position',[383 42 850 960]);


f7 = figure;
f7.Renderer = 'painters';
subplot(4,4,1)
plot(step_v.U_ist_linear.Time, step_v.U_ist_linear.Data(:,1), '--r', ...
    step_v.U_ist_linear.Time, step_v.U_ist_linear.Data(:,5), '-.b')
legend('linear plane1', 'linear plane2', 'Location', 'northeast')
ylim([-0.15 0]);
xlim([0 1.5]);
ylabel('to \eta [rad]');
title('\rm from w_{v1}')
grid on;
subplot(4,4,5)
plot(step_v.U_ist_linear.Time, step_v.U_ist_linear.Data(:,3), '--r', ...
    step_v.U_ist_linear.Time, step_v.U_ist_linear.Data(:,7), '-.b');
ylim([-0.05 0.2]);
xlim([0 1.5]);
ylabel('to \xi [rad]');
grid on;
subplot(4,4,9)
plot(step_v.U_ist_linear.Time, step_v.U_ist_linear.Data(:,4), '--r', ...
    step_v.U_ist_linear.Time, step_v.U_ist_linear.Data(:,8), '-.b');
ylim([-0.05 0.5]);
xlim([0 1.5]);
ylabel('to \zeta [rad]');
grid on;
subplot(4,4,13)
plot(step_v.U_ist_linear.Time, step_v.U_ist_linear.Data(:,2), '--r', ...
    step_v.U_ist_linear.Time, step_v.U_ist_linear.Data(:,6), '-.b');
ylim([0 0.4]);
xlim([0 1.5]);
ylabel('to \sigma_f');
grid on;
xlabel('Time [s]');

subplot(4,4,2)
plot(step_phi.U_ist_linear.Time, step_phi.U_ist_linear.Data(:,1), '--r', ...
    step_phi.U_ist_linear.Time, step_phi.U_ist_linear.Data(:,5), '-.b');
ylim([-0.15 0]);
xlim([0 1.5]);
title('\rm from w_{\Phi1}')
grid on;
subplot(4,4,6)
plot(step_phi.U_ist_linear.Time, step_phi.U_ist_linear.Data(:,3), '--r', ...
    step_phi.U_ist_linear.Time, step_phi.U_ist_linear.Data(:,7), '-.b');
ylim([-0.2 0.1]);
xlim([0 1.5]);
grid on;
subplot(4,4,10)
plot(step_phi.U_ist_linear.Time, step_phi.U_ist_linear.Data(:,4), '--r', ...
    step_phi.U_ist_linear.Time, step_phi.U_ist_linear.Data(:,8), '-.b');
ylim([-0.1 0.1]);
xlim([0 1.5]);
grid on;
subplot(4,4,14)
plot(step_phi.U_ist_linear.Time, step_phi.U_ist_linear.Data(:,2), '--r', ...
    step_phi.U_ist_linear.Time, step_phi.U_ist_linear.Data(:,6), '-.b');
ylim([0 0.4]);
xlim([0 1.5]);
grid on;
xlabel('Time [s]');

subplot(4,4,3)
plot(step_theta_ohne_stell.U_ist_linear.Time, step_theta_ohne_stell.U_ist_linear.Data(:,1), '--r', ...
    step_theta_ohne_stell.U_ist_linear.Time, step_theta_ohne_stell.U_ist_linear.Data(:,5), '-.b');
ylim([-40 10]);
xlim([0 1.5]);
title('\rm from w_{\Theta1}')
grid on;
subplot(4,4,7)
plot(step_theta_ohne_stell.U_ist_linear.Time, step_theta_ohne_stell.U_ist_linear.Data(:,3), '--r', ...
    step_theta_ohne_stell.U_ist_linear.Time, step_theta_ohne_stell.U_ist_linear.Data(:,7), '-.b');
ylim([-0.2 0.1]);
xlim([0 1.5]);
grid on;
subplot(4,4,11)
plot(step_theta_ohne_stell.U_ist_linear.Time, step_theta_ohne_stell.U_ist_linear.Data(:,4), '--r', ...
    step_theta_ohne_stell.U_ist_linear.Time, step_theta_ohne_stell.U_ist_linear.Data(:,8), '-.b');
ylim([-0.1 0.1]);
xlim([0 1.5]);
grid on;
subplot(4,4,15)
plot(step_theta_ohne_stell.U_ist_linear.Time, step_theta_ohne_stell.U_ist_linear.Data(:,2), '--r', ...
    step_theta_ohne_stell.U_ist_linear.Time, step_theta_ohne_stell.U_ist_linear.Data(:,6), '-.b');
ylim([-650 10]);
xlim([0 1.5]);
grid on;
xlabel('Time [s]');

subplot(4,4,4)
plot(step_h.U_ist_linear.Time, step_h.U_ist_linear.Data(:,1), '--r', ...
    step_h.U_ist_linear.Time, step_h.U_ist_linear.Data(:,5), '-.b');
ylim([-1 1]);
xlim([0 1.5]);
title('\rm from w_{h1}')
grid on;
subplot(4,4,8)
plot(step_h.U_ist_linear.Time, step_h.U_ist_linear.Data(:,3), '--r', ...
    step_h.U_ist_linear.Time, step_h.U_ist_linear.Data(:,7), '-.b');
ylim([-0.2 0.1]);
xlim([0 1.5]);
grid on;
subplot(4,4,12)
plot(step_h.U_ist_linear.Time, step_h.U_ist_linear.Data(:,4), '--r', ...
    step_h.U_ist_linear.Time, step_h.U_ist_linear.Data(:,8), '-.b');
ylim([-0.1 0.1]);
xlim([0 1.5]);
grid on;
subplot(4,4,16)
plot(step_h.U_ist_linear.Time, step_h.U_ist_linear.Data(:,2), '--r', ...
    step_h.U_ist_linear.Time, step_h.U_ist_linear.Data(:,6), '-.b');
ylim([0 0.4]);
xlim([0 1.5]);
grid on;
xlabel('Time [s]');
set(gcf, 'Position',[383 42 850 960/1.5]);

%% plot differenz flugzeugpositionen
f8 = figure;
subplot(1,4,1)
plot(step_v.difference_position_nonlinear.Time, step_v.difference_position_nonlinear.Data(:,1), '-r', ...
    step_v.difference_position_nonlinear.Time, step_v.difference_position_nonlinear.Data(:,2), '-.b', ...
    step_v.difference_position_nonlinear.Time, step_v.difference_position_nonlinear.Data(:,3), '--k');
legend('\Delta x', '\Delta y', '\Delta h', 'Location', 'southeast')
ylim([-15 5]);
% xlim([0 1.5]);
title('\rm from w_{v1}')
ylabel('Distance [m]');
xlabel('Time [s]');
pbaspect([1.5 2 1])
grid on;
subplot(1,4,2)
plot(step_phi.difference_position_nonlinear.Time, step_phi.difference_position_nonlinear.Data(:,1), '-r', ...
    step_phi.difference_position_nonlinear.Time, step_phi.difference_position_nonlinear.Data(:,2), '-.b', ...
    step_phi.difference_position_nonlinear.Time, step_phi.difference_position_nonlinear.Data(:,3), '--k');
ylim([-15 5]);
% xlim([0 1.5]);
title('\rm from w_{\Phi1}')
xlabel('Time [s]');
pbaspect([1.5 2 1])
grid on;
subplot(1,4,3)
plot(step_theta.difference_position_nonlinear.Time, step_theta.difference_position_nonlinear.Data(:,1), '-r', ...
    step_theta.difference_position_nonlinear.Time, step_theta.difference_position_nonlinear.Data(:,2), '-.b', ...
    step_theta.difference_position_nonlinear.Time, step_theta.difference_position_nonlinear.Data(:,3), '--k');
ylim([-15 5]);
% xlim([0 1.5]);
title('\rm from w_{\Theta1}')
xlabel('Time [s]');
pbaspect([1.5 2 1])
grid on;
subplot(1,4,4)
plot(step_h.difference_position_nonlinear.Time, step_h.difference_position_nonlinear.Data(:,1), '-r', ...
    step_h.difference_position_nonlinear.Time, step_h.difference_position_nonlinear.Data(:,2), '-.b', ...
    step_h.difference_position_nonlinear.Time, step_h.difference_position_nonlinear.Data(:,3), '--k');
ylim([-15 5]);
% xlim([0 1.5]);
title('\rm from w_{h1}')
xlabel('Time [s]');
pbaspect([1.5 2 1])
grid on;
