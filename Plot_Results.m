%% ========================================================================
%  QUADCOPTER CASCADE PID -- STATIC RESULTS: FIGURES + METRICS
%
%  Replaces the real-time animation with publication-ready static plots
%  and prints figure-8 phase RMSE metrics to the Command Window.
%
%  Run AFTER the Simulink simulation:
%     1. run('Quad_Params.m')
%     2. open('Quad_Cascade_PID.slx')  ->  Run
%     3. run('plot_results_quad.m')
%
%  Requires in workspace: states_log, tout, Xd_traj, Yd_traj, Zd_traj,
%                         t_ref, t_sim, Z_hover, A_fig8, B_fig8,
%                         t_takeoff_duration, t_land_start
%
%  Outputs:
%     figures/simulation_results.png   (6-panel dashboard)
%     RMSE metrics printed to the Command Window
%% ========================================================================

clc;

%% ------------------------------------------------------------------------
%  CHECK DATA
%% ------------------------------------------------------------------------

if ~exist('states_log','var') || ~exist('tout','var')
    error('Run the Simulink simulation first: states_log and tout must exist.');
end
if ~exist('Xd_traj','var')
    error('Run Quad_Params.m first: Xd_traj must exist in the workspace.');
end

fprintf('Data found. Generating result figures...\n');

%% ------------------------------------------------------------------------
%  UNPACK STATES AND INTERPOLATE REFERENCE
%  states_log columns: [P Q R Phi Theta Psi U V W X Y Z]
%% ------------------------------------------------------------------------

Phi_real   = states_log(:,4);
Theta_real = states_log(:,5);
Psi_real   = states_log(:,6);
X_real     = states_log(:,10);
Y_real     = states_log(:,11);
Z_real     = states_log(:,12);
t_real     = tout;

Xd_i = interp1(t_ref, Xd_traj, t_real, 'linear', 'extrap');
Yd_i = interp1(t_ref, Yd_traj, t_real, 'linear', 'extrap');
Zd_i = interp1(t_ref, Zd_traj, t_real, 'linear', 'extrap');

eX = Xd_i - X_real;
eY = Yd_i - Y_real;
eZ = Zd_i - Z_real;
pos_err = sqrt(eX.^2 + eY.^2 + eZ.^2);

%% ------------------------------------------------------------------------
%  METRICS -- figure-8 phase only (takeoff and landing excluded)
%% ------------------------------------------------------------------------

idx = (t_real >= t_takeoff_duration) & (t_real <= t_land_start);

rmse_x = sqrt(mean(eX(idx).^2));
rmse_y = sqrt(mean(eY(idx).^2));
rmse_z = sqrt(mean(eZ(idx).^2));

norm_x = rmse_x / A_fig8  * 100;
norm_y = rmse_y / B_fig8  * 100;
norm_z = rmse_z / Z_hover * 100;

max_z_dev = max(abs(eZ(idx)));

fprintf('\n==============================================\n');
fprintf('  FIGURE-8 PHASE METRICS  (t = %.0f to %.0f s)\n', ...
        t_takeoff_duration, t_land_start);
fprintf('==============================================\n');
fprintf('  RMSE X : %.4f m   (%.1f%% of %.1f m amplitude)\n', rmse_x, norm_x, A_fig8);
fprintf('  RMSE Y : %.4f m   (%.1f%% of %.1f m amplitude)\n', rmse_y, norm_y, B_fig8);
fprintf('  RMSE Z : %.4f m   (%.2f%% of %.1f m altitude)\n',  rmse_z, norm_z, Z_hover);
fprintf('  Max altitude deviation : %.2f cm\n', max_z_dev * 100);
fprintf('==============================================\n\n');

%% ------------------------------------------------------------------------
%  6-PANEL RESULTS FIGURE
%% ------------------------------------------------------------------------

lw = 1.5;
fig = figure('Name','Cascade PID Quadcopter -- Simulation Results', ...
             'Color','w', 'Position',[50 50 1500 950]);
sgtitle('Cascade PID Quadcopter -- Simulation Results', ...
        'FontSize', 16, 'FontWeight', 'bold');

% --- (1) 3D trajectory --------------------------------------------------
subplot(2,3,1);
plot3(X_real, Y_real, Z_real, 'b-',  'LineWidth', lw); hold on;
plot3(Xd_i,   Yd_i,   Zd_i,   'r--', 'LineWidth', lw);
grid on; view(45, 28);
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
title('3D Trajectory');
legend('Actual','Reference','Location','best','FontSize',8);

% --- (2) XY plane -------------------------------------------------------
subplot(2,3,2);
plot(X_real, Y_real, 'b-',  'LineWidth', lw); hold on;
plot(Xd_i,   Yd_i,   'r--', 'LineWidth', lw);
grid on; axis equal;
xlabel('X (m)'); ylabel('Y (m)');
title('XY Plane');
legend('Actual','Reference','Location','best','FontSize',8);

% --- (3) Horizontal position tracking -----------------------------------
subplot(2,3,3);
plot(t_real, X_real, 'b-',  'LineWidth', lw); hold on;
plot(t_real, Xd_i,   'r--', 'LineWidth', lw);
plot(t_real, Y_real, 'g-',  'LineWidth', lw);
plot(t_real, Yd_i,   'm--', 'LineWidth', lw);
grid on;
xlabel('Time (s)'); ylabel('Position (m)');
title('Horizontal Position Tracking');
legend('X actual','X ref','Y actual','Y ref','Location','best','FontSize',8);

% --- (4) Altitude tracking ----------------------------------------------
subplot(2,3,4);
plot(t_real, Z_real, 'b-',  'LineWidth', lw); hold on;
plot(t_real, Zd_i,   'r--', 'LineWidth', lw);
grid on;
xlabel('Time (s)'); ylabel('Altitude (m)');
title('Altitude Tracking');
legend('Z actual','Z ref','Location','best','FontSize',8);

% --- (5) Position tracking error ----------------------------------------
subplot(2,3,5);
plot(t_real, pos_err, 'r-', 'LineWidth', lw);
grid on;
xlabel('Time (s)'); ylabel('Error (m)');
title('Position Tracking Error');

% --- (6) Attitude angles -------------------------------------------------
subplot(2,3,6);
plot(t_real, rad2deg(Phi_real),   'b-', 'LineWidth', lw); hold on;
plot(t_real, rad2deg(Theta_real), 'g-', 'LineWidth', lw);
plot(t_real, rad2deg(Psi_real),   'r-', 'LineWidth', lw);
grid on;
xlabel('Time (s)'); ylabel('Angle (deg)');
title('Attitude Angles');
legend('Roll (Phi)','Pitch (Theta)','Yaw (Psi)','Location','best','FontSize',8);

%% ------------------------------------------------------------------------
%  SAVE
%% ------------------------------------------------------------------------

fig_dir = fullfile(pwd, 'figures');
if ~exist(fig_dir, 'dir'); mkdir(fig_dir); end
out_png = fullfile(fig_dir, 'simulation_results.png');
try
    exportgraphics(fig, out_png, 'Resolution', 150);
catch
    print(fig, out_png, '-dpng', '-r150');   % fallback for path/renderer quirks
end
fprintf('Saved: %s\n', out_png);