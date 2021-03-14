%% load störungen
load('C:\Users\Markus\Documents\Uni\WISE2021\Projektseminar\PS_Luftbetankung\PS_Luftbetankung_newRepo\Projekteseminar-Luftbetankung\Code\Messungen\luftloch_1ms.mat')
luftloch = out;
clear out;
load('C:\Users\Markus\Documents\Uni\WISE2021\Projektseminar\PS_Luftbetankung\PS_Luftbetankung_newRepo\Projekteseminar-Luftbetankung\Code\Messungen\windboe.mat')
windboe = out;
clear out;

X_AP = [150;0;-11.0225402979539;0;0;0;0;-0.0733517623176560;0;5000;150;0;-11.0065445940443;0;0;0;0;-0.0732456962006296;0;5010];
U_AP = [-0.0870457297521801;0.247747504909435;0;0;-0.0871233228705516;0.247511353508586;0;0];
h_1_ap = X_AP(10);
h_2_ap = X_AP(20);
delta_h_ap = X_AP(10)-X_AP(20);
delta_theta_ap = X_AP(8)-X_AP(18);

%% plot h1 und h2 und coupled output plus für luftloch
f_luftloch = figure;
f_luftloch.Renderer = 'painters';
tiledlayout(2,2);
nexttile([1,2]);
plot(luftloch.luftloch.Time, luftloch.luftloch.Data(:,1)+h_1_ap, 'k:', ...
    luftloch.states_nonlinear.Time, luftloch.states_nonlinear.Data(:,9), '-r', ...
    luftloch.states_nonlinear.Time, luftloch.states_nonlinear.Data(:,18), '--b')
legend('Störung', 'h_1', 'h_2', 'Location', 'southeast')
ylim([4990 5020]);
ylabel('Höhe [m], Störung [m/s]');
xlabel('Time [s]');
grid on;

nexttile;
plot(luftloch.outputs_nonlinear.Time, luftloch.outputs_nonlinear.Data(:,5), 'k', ...
    luftloch.outputs_nonlinear.Time, luftloch.outputs_nonlinear.Data(:,6), '--b', ...
    luftloch.outputs_nonlinear.Time, luftloch.outputs_nonlinear.Data(:,7)-delta_theta_ap, '-.r')
legend('\Delta v', '\Delta \Phi', '\Delta \Theta', 'Location', 'northeast')
ylim([-0.01 0.01]);
ylabel('[m/s], [rad]');
xlabel('Time [s]');
grid on;

nexttile;
plot(luftloch.outputs_nonlinear.Time, luftloch.outputs_nonlinear.Data(:,8), 'k')
legend('\Delta h', 'Location', 'southeast')
ylim([-20 -9]);
ylabel('[m]');
xlabel('Time [s]');
grid on;
    
f_pos_luftloch = figure;
f_pos_luftloch.Renderer = 'painters';
plot(luftloch.difference_position_nonlinear.Time, luftloch.difference_position_nonlinear.Data(:,1), '-r', ...
    luftloch.difference_position_nonlinear.Time, luftloch.difference_position_nonlinear.Data(:,2), '--b', ...
    luftloch.difference_position_nonlinear.Time, luftloch.difference_position_nonlinear.Data(:,3), '-.k');
legend('\Delta x', '\Delta y', '\Delta h', 'Location', 'southeast')
ylim([-20 5]);
ylabel('Distance [m]');
xlabel('Time [s]');
pbaspect([1.5 2 1])
grid on;

%% plot outputs für wind
f_wind = figure;
f_wind.Renderer = 'painters';
plot(windboe.wind_v1.Time, windboe.wind_v1.Data(:,1), 'k:', ...
    windboe.wind_v1.Time, windboe.wind_v1.Data(:,2), '-r', ...
    windboe.wind_v1.Time, windboe.wind_v1.Data(:,3), '--b')
legend('u_w', 'v_w', 'w_w', 'Location', 'northeast')
ylim([-0.5 4]);
ylabel('wind speed [m/s]');
xlabel('Time [s]');
grid on;


f_output_wind = figure;
f_output_wind.Renderer = 'painters';
% tiledlayout(2,2);
nexttile([1,1]);
plot(windboe.outputs_nonlinear.Time, windboe.outputs_nonlinear.Data(:,4), 'k', ...
    windboe.states_nonlinear.Time, windboe.states_nonlinear.Data(:,18), '--b')
legend('h_1', 'h_2', 'Location', 'northeast')
ylim([4998 5012]);
ylabel('Height [m]');
xlabel('Time [s]');
grid on;

nexttile;
plot(windboe.outputs_nonlinear.Time, windboe.outputs_nonlinear.Data(:,1), 'k', ...
    windboe.outputs_nonlinear.Time, windboe.outputs_nonlinear.Data(:,2), '--b', ...
    windboe.outputs_nonlinear.Time, windboe.outputs_nonlinear.Data(:,3), '-.r')
legend('v_1', '\Phi_1', '\Theta_1', 'Location', 'northwest')
ylim([-0.25 0.25]);
ylabel('[m/s], [rad]');
xlabel('Time [s]');
grid on;

f_output_coupled_wind = figure;
f_output_coupled_wind.Renderer = 'painters';
nexttile;
plot(windboe.outputs_nonlinear.Time, windboe.outputs_nonlinear.Data(:,8), 'k')
legend('\Delta h', 'Location', 'southeast')
ylim([-12 -8]);
ylabel('[m]');
xlabel('Time [s]');
grid on;

nexttile;
plot(windboe.outputs_nonlinear.Time, windboe.outputs_nonlinear.Data(:,5), 'k', ...
    windboe.outputs_nonlinear.Time, windboe.outputs_nonlinear.Data(:,6), '--b', ...
    windboe.outputs_nonlinear.Time, windboe.outputs_nonlinear.Data(:,7), '-.r')
legend('\Delta v_1', '\Delta \Phi_1', '\Delta \Theta_1', 'Location', 'northwest')
ylim(0.01*[-0.25 0.25]);
ylabel('[m/s], [rad]');
xlabel('Time [s]');
grid on;


f_pos_wind = figure;
f_pos_wind.Renderer = 'painters';
plot(windboe.difference_position_nonlinear.Time, windboe.difference_position_nonlinear.Data(:,1), '-r', ...
    windboe.difference_position_nonlinear.Time, windboe.difference_position_nonlinear.Data(:,2), '--b', ...
    windboe.difference_position_nonlinear.Time, windboe.difference_position_nonlinear.Data(:,3), '-.k');
legend('\Delta x', '\Delta y', '\Delta h', 'Location', 'northeast')
ylim([-12 2]);
ylabel('Distance [m]');
xlabel('Time [s]');
pbaspect([1.5 2 1])
grid on;