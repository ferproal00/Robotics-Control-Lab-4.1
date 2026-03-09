clear; clc; close all;

% =========================================================
% ===================== EDITAR AQUI =======================
% =========================================================
dir_A = 'C:\Users\algoi\OneDrive\Escritorio\Semestre 6\Fundamentacion\Nezih\4.1\pd_withpert';
name_A = 'PD Pert';

dir_B = 'C:\Users\algoi\OneDrive\Escritorio\Semestre 6\Fundamentacion\Nezih\4.1\ctc_withpert';
name_B = 'CTC-like Pert';

save_figs = true;
out_dir = 'C:\Users\algoi\OneDrive\Escritorio\Semestre 6\Fundamentacion\Nezih\4.1\PD_pert_vs_CTC_pert';
% =========================================================

if save_figs && ~isfolder(out_dir)
    mkdir(out_dir);
end

A_des = readtable(fullfile(dir_A, 'desired_position.csv'));
A_act = readtable(fullfile(dir_A, 'actual_position.csv'));

B_des = readtable(fullfile(dir_B, 'desired_position.csv'));
B_act = readtable(fullfile(dir_B, 'actual_position.csv'));

% Interpolacion para RMSE
A_xi = interp1(A_act.t_sec, A_act.x, A_des.t_sec, 'linear', 'extrap');
A_yi = interp1(A_act.t_sec, A_act.y, A_des.t_sec, 'linear', 'extrap');
A_zi = interp1(A_act.t_sec, A_act.z, A_des.t_sec, 'linear', 'extrap');

B_xi = interp1(B_act.t_sec, B_act.x, B_des.t_sec, 'linear', 'extrap');
B_yi = interp1(B_act.t_sec, B_act.y, B_des.t_sec, 'linear', 'extrap');
B_zi = interp1(B_act.t_sec, B_act.z, B_des.t_sec, 'linear', 'extrap');

A_enorm = sqrt((A_des.x - A_xi).^2 + (A_des.y - A_yi).^2 + (A_des.z - A_zi).^2);
B_enorm = sqrt((B_des.x - B_xi).^2 + (B_des.y - B_yi).^2 + (B_des.z - B_zi).^2);

A_rmse = sqrt(mean(A_enorm.^2));
B_rmse = sqrt(mean(B_enorm.^2));

fprintf('\n===== Comparison =====\n');
fprintf('%s RMSE XYZ = %.6f m\n', name_A, A_rmse);
fprintf('%s RMSE XYZ = %.6f m\n', name_B, B_rmse);

f1 = figure('Name','Compare X','Color','w');
plot(A_des.t_sec, A_des.x, '--', 'LineWidth', 1.2); hold on;
plot(A_act.t_sec, A_act.x, 'LineWidth', 1.2);
plot(B_act.t_sec, B_act.x, 'LineWidth', 1.2);
grid on;
xlabel('Time [s]'); ylabel('X [m]');
legend('Desired', name_A, name_B, 'Location', 'best');
title('Comparison in X');

f2 = figure('Name','Compare Y','Color','w');
plot(A_des.t_sec, A_des.y, '--', 'LineWidth', 1.2); hold on;
plot(A_act.t_sec, A_act.y, 'LineWidth', 1.2);
plot(B_act.t_sec, B_act.y, 'LineWidth', 1.2);
grid on;
xlabel('Time [s]'); ylabel('Y [m]');
legend('Desired', name_A, name_B, 'Location', 'best');
title('Comparison in Y');

f3 = figure('Name','Compare Z','Color','w');
plot(A_des.t_sec, A_des.z, '--', 'LineWidth', 1.2); hold on;
plot(A_act.t_sec, A_act.z, 'LineWidth', 1.2);
plot(B_act.t_sec, B_act.z, 'LineWidth', 1.2);
grid on;
xlabel('Time [s]'); ylabel('Z [m]');
legend('Desired', name_A, name_B, 'Location', 'best');
title('Comparison in Z');

f4 = figure('Name','Compare Error Norm','Color','w');
plot(A_des.t_sec, A_enorm, 'LineWidth', 1.2); hold on;
plot(B_des.t_sec, B_enorm, 'LineWidth', 1.2);
grid on;
xlabel('Time [s]'); ylabel('|e| [m]');
legend(name_A, name_B, 'Location', 'best');
title('End-Effector Error Norm Comparison');

if save_figs
    exportgraphics(f1, fullfile(out_dir, 'compare_x.png'), 'Resolution', 200);
    exportgraphics(f2, fullfile(out_dir, 'compare_y.png'), 'Resolution', 200);
    exportgraphics(f3, fullfile(out_dir, 'compare_z.png'), 'Resolution', 200);
    exportgraphics(f4, fullfile(out_dir, 'compare_error_norm.png'), 'Resolution', 200);
end