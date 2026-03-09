clear; clc; close all;

% =========================================================
% ===================== EDITAR AQUI =======================
% =========================================================
data_dir   = 'C:\Users\algoi\OneDrive\Escritorio\Semestre 6\Fundamentacion\Nezih\4.1\ctc_withpert';
trial_name = 'CTC With Perturbation';
save_figs  = true;
out_dir    = 'C:\Users\algoi\OneDrive\Escritorio\Semestre 6\Fundamentacion\Nezih\4.1\ctc_withpert\results';
% =========================================================

if ~isfolder(data_dir)
    error("La carpeta de datos no existe: %s", data_dir);
end

if save_figs && ~isfolder(out_dir)
    mkdir(out_dir);
end

%% ===================== CARGA DE DATOS =====================
desired = readtable(fullfile(data_dir, 'desired_position.csv'));
actual  = readtable(fullfile(data_dir, 'actual_position.csv'));
errtbl  = readtable(fullfile(data_dir, 'position_error.csv'));
cmdspd  = readtable(fullfile(data_dir, 'commanded_speed.csv'));
ctrltw  = readtable(fullfile(data_dir, 'controller_delta_twist_cmds.csv'));
servtw  = readtable(fullfile(data_dir, 'servo_server_delta_twist_cmds.csv'));
jointst = readtable(fullfile(data_dir, 'joint_states.csv'));

hasPert = isfile(fullfile(data_dir, 'perturbation_enable.csv'));
if hasPert
    pert = readtable(fullfile(data_dir, 'perturbation_enable.csv'));
else
    pert = table();
end

hasPhase = isfile(fullfile(data_dir, 'task_phase.csv'));
if hasPhase
    phase = readtable(fullfile(data_dir, 'task_phase.csv'));
else
    phase = table();
end

%% ===================== METRICAS =====================
% Interpolar actual sobre tiempo desired
x_act_i = interp1(actual.t_sec, actual.x, desired.t_sec, 'linear', 'extrap');
y_act_i = interp1(actual.t_sec, actual.y, desired.t_sec, 'linear', 'extrap');
z_act_i = interp1(actual.t_sec, actual.z, desired.t_sec, 'linear', 'extrap');

ex = desired.x - x_act_i;
ey = desired.y - y_act_i;
ez = desired.z - z_act_i;

e_norm = sqrt(ex.^2 + ey.^2 + ez.^2);

rmse_x = sqrt(mean(ex.^2));
rmse_y = sqrt(mean(ey.^2));
rmse_z = sqrt(mean(ez.^2));
rmse_xyz = sqrt(mean(e_norm.^2));

max_err_x = max(abs(ex));
max_err_y = max(abs(ey));
max_err_z = max(abs(ez));
max_err_xyz = max(e_norm);

fprintf('\n===== %s =====\n', trial_name);
fprintf('RMSE X      = %.6f m\n', rmse_x);
fprintf('RMSE Y      = %.6f m\n', rmse_y);
fprintf('RMSE Z      = %.6f m\n', rmse_z);
fprintf('RMSE XYZ    = %.6f m\n', rmse_xyz);
fprintf('MAX ERR X   = %.6f m\n', max_err_x);
fprintf('MAX ERR Y   = %.6f m\n', max_err_y);
fprintf('MAX ERR Z   = %.6f m\n', max_err_z);
fprintf('MAX ERR XYZ = %.6f m\n', max_err_xyz);

% Guardar resumen
summaryTable = table( ...
    rmse_x, rmse_y, rmse_z, rmse_xyz, ...
    max_err_x, max_err_y, max_err_z, max_err_xyz);

if save_figs
    writetable(summaryTable, fullfile(out_dir, 'metrics_summary.csv'));
end

%% ===================== FIGURA 1: XYZ tracking =====================
f1 = figure('Name',[trial_name ' - Desired vs Actual XYZ'],'Color','w');

subplot(3,1,1);
plot(desired.t_sec, desired.x, '--', 'LineWidth', 1.4); hold on;
plot(actual.t_sec, actual.x, 'LineWidth', 1.2);
grid on; ylabel('X [m]');
legend('Desired','Actual','Location','best');
title([trial_name ' - X Tracking']);

subplot(3,1,2);
plot(desired.t_sec, desired.y, '--', 'LineWidth', 1.4); hold on;
plot(actual.t_sec, actual.y, 'LineWidth', 1.2);
grid on; ylabel('Y [m]');
legend('Desired','Actual','Location','best');
title([trial_name ' - Y Tracking']);

subplot(3,1,3);
plot(desired.t_sec, desired.z, '--', 'LineWidth', 1.4); hold on;
plot(actual.t_sec, actual.z, 'LineWidth', 1.2);
grid on; ylabel('Z [m]'); xlabel('Time [s]');
legend('Desired','Actual','Location','best');
title([trial_name ' - Z Tracking']);

%% ===================== FIGURA 2: Position error =====================
f2 = figure('Name',[trial_name ' - Position Error'],'Color','w');

subplot(4,1,1);
plot(errtbl.t_sec, errtbl.x, 'LineWidth', 1.2); grid on;
ylabel('e_x [m]');
title('Position Error X');

subplot(4,1,2);
plot(errtbl.t_sec, errtbl.y, 'LineWidth', 1.2); grid on;
ylabel('e_y [m]');
title('Position Error Y');

subplot(4,1,3);
plot(errtbl.t_sec, errtbl.z, 'LineWidth', 1.2); grid on;
ylabel('e_z [m]');
title('Position Error Z');

subplot(4,1,4);
plot(desired.t_sec, e_norm, 'LineWidth', 1.2); grid on;
ylabel('|e| [m]'); xlabel('Time [s]');
title('End-Effector Error Norm');

%% ===================== FIGURA 3: Commanded speed =====================
f3 = figure('Name',[trial_name ' - Commanded Speed'],'Color','w');

subplot(3,1,1);
plot(cmdspd.t_sec, cmdspd.x, 'LineWidth', 1.2); grid on;
ylabel('v_x [m/s]');
title('Commanded Speed X');

subplot(3,1,2);
plot(cmdspd.t_sec, cmdspd.y, 'LineWidth', 1.2); grid on;
ylabel('v_y [m/s]');
title('Commanded Speed Y');

subplot(3,1,3);
plot(cmdspd.t_sec, cmdspd.z, 'LineWidth', 1.2); grid on;
ylabel('v_z [m/s]'); xlabel('Time [s]');
title('Commanded Speed Z');

%% ===================== FIGURA 4: Controller vs Servo twist =====================
f4 = figure('Name',[trial_name ' - Controller vs Servo Twist'],'Color','w');

subplot(3,1,1);
plot(ctrltw.t_sec, ctrltw.lin_x, '--', 'LineWidth', 1.4); hold on;
plot(servtw.t_sec, servtw.lin_x, 'LineWidth', 1.2);
grid on; ylabel('v_x [m/s]');
legend('Controller','Servo','Location','best');
title('Twist Linear X');

subplot(3,1,2);
plot(ctrltw.t_sec, ctrltw.lin_y, '--', 'LineWidth', 1.4); hold on;
plot(servtw.t_sec, servtw.lin_y, 'LineWidth', 1.2);
grid on; ylabel('v_y [m/s]');
legend('Controller','Servo','Location','best');
title('Twist Linear Y');

subplot(3,1,3);
plot(ctrltw.t_sec, ctrltw.lin_z, '--', 'LineWidth', 1.4); hold on;
plot(servtw.t_sec, servtw.lin_z, 'LineWidth', 1.2);
grid on; ylabel('v_z [m/s]'); xlabel('Time [s]');
legend('Controller','Servo','Location','best');
title('Twist Linear Z');

%% ===================== FIGURA 5: Perturbation enable =====================
if hasPert
    f5 = figure('Name',[trial_name ' - Perturbation Enable'],'Color','w');
    stairs(pert.t_sec, pert.data, 'LineWidth', 1.4);
    ylim([-0.1 1.1]);
    grid on;
    xlabel('Time [s]');
    ylabel('Enable');
    title([trial_name ' - Perturbation Enable']);
end

%% ===================== FIGURA 6: Joint trajectories =====================
f6 = figure('Name',[trial_name ' - Joint Positions'],'Color','w');

jointNames = strings(1,6);
for i = 1:6
    jointNames(i) = string(jointst{1, sprintf('name_%d', i)});
end

for i = 1:6
    subplot(3,2,i);
    plot(jointst.t_sec, jointst{:, sprintf('pos_%d', i)}, 'LineWidth', 1.1);
    grid on;
    xlabel('Time [s]');
    ylabel(sprintf('q_%d [rad]', i));
    title(sprintf('Joint %d (%s)', i, jointNames(i)));
end

%% ===================== FIGURA 7: Joint velocities =====================
f7 = figure('Name',[trial_name ' - Joint Velocities'],'Color','w');

for i = 1:6
    subplot(3,2,i);
    plot(jointst.t_sec, jointst{:, sprintf('vel_%d', i)}, 'LineWidth', 1.1);
    grid on;
    xlabel('Time [s]');
    ylabel(sprintf('dq_%d [rad/s]', i));
    title(sprintf('Joint Velocity %d', i));
end

%% ===================== FIGURA 8: 3D path =====================
f8 = figure('Name',[trial_name ' - 3D Path'],'Color','w');
plot3(desired.x, desired.y, desired.z, '--', 'LineWidth', 1.4); hold on;
plot3(actual.x, actual.y, actual.z, 'LineWidth', 1.2);
grid on;
xlabel('X [m]');
ylabel('Y [m]');
zlabel('Z [m]');
legend('Desired','Actual','Location','best');
title([trial_name ' - 3D End-Effector Path']);
axis equal;

%% ===================== Guardar figuras =====================
if save_figs
    exportgraphics(f1, fullfile(out_dir, '01_xyz_tracking.png'), 'Resolution', 200);
    exportgraphics(f2, fullfile(out_dir, '02_position_error.png'), 'Resolution', 200);
    exportgraphics(f3, fullfile(out_dir, '03_commanded_speed.png'), 'Resolution', 200);
    exportgraphics(f4, fullfile(out_dir, '04_controller_vs_servo_twist.png'), 'Resolution', 200);
    if hasPert
        exportgraphics(f5, fullfile(out_dir, '05_perturbation_enable.png'), 'Resolution', 200);
    end
    exportgraphics(f6, fullfile(out_dir, '06_joint_positions.png'), 'Resolution', 200);
    exportgraphics(f7, fullfile(out_dir, '07_joint_velocities.png'), 'Resolution', 200);
    exportgraphics(f8, fullfile(out_dir, '08_3d_path.png'), 'Resolution', 200);
end
