clear; clc; close all;

% =========================================================
% ===================== EDITAR AQUI =======================
% =========================================================
dir_pd = 'C:\Users\algoi\OneDrive\Escritorio\Semestre 6\Fundamentacion\Nezih\4.1\pd_withpert';
dir_ctc = 'C:\Users\algoi\OneDrive\Escritorio\Semestre 6\Fundamentacion\Nezih\4.1\ctc_withpert';

name_pd  = 'PD con perturbación';
name_ctc = 'CTC-like con perturbación';

save_figs = true;
out_dir   = 'C:\Users\algoi\OneDrive\Escritorio\Semestre 6\Fundamentacion\Nezih\4.1\phase_results';
% =========================================================

if save_figs && ~isfolder(out_dir)
    mkdir(out_dir);
end

% Cargar datos
pd_js  = readtable(fullfile(dir_pd,  'joint_states.csv'));
ctc_js = readtable(fullfile(dir_ctc, 'joint_states.csv'));

% Joints seleccionados
joint_idx = [2, 3, 5];

% Confirmar nombres
joint_names_pd = strings(1,6);
joint_names_ctc = strings(1,6);
for i = 1:6
    joint_names_pd(i)  = string(pd_js{1,  sprintf('name_%d', i)});
    joint_names_ctc(i) = string(ctc_js{1, sprintf('name_%d', i)});
end

%% ===================== FIGURA 1: phase portraits overlay =====================
f1 = figure('Name','Phase Portraits PD vs CTC','Color','w');

for k = 1:length(joint_idx)
    j = joint_idx(k);

    q_pd  = pd_js{:,  sprintf('pos_%d', j)};
    dq_pd = pd_js{:,  sprintf('vel_%d', j)};

    q_ctc  = ctc_js{:, sprintf('pos_%d', j)};
    dq_ctc = ctc_js{:, sprintf('vel_%d', j)};

    subplot(1,3,k);
    plot(q_pd,  dq_pd,  'LineWidth', 1.2); hold on;
    plot(q_ctc, dq_ctc, 'LineWidth', 1.2);
    grid on;
    xlabel(sprintf('q_%d [rad]', j));
    ylabel(sprintf('dq_%d [rad/s]', j));
    title(sprintf('Joint %d', j));
    legend(name_pd, name_ctc, 'Location', 'best');
end

sgtitle('Diagramas de fase: PD vs CTC con perturbación');

%% ===================== FIGURA 2: phase portraits separados PD =====================
f2 = figure('Name','Phase Portraits PD','Color','w');

for k = 1:length(joint_idx)
    j = joint_idx(k);

    q_pd  = pd_js{:,  sprintf('pos_%d', j)};
    dq_pd = pd_js{:,  sprintf('vel_%d', j)};

    subplot(1,3,k);
    plot(q_pd, dq_pd, 'LineWidth', 1.2);
    grid on;
    xlabel(sprintf('q_%d [rad]', j));
    ylabel(sprintf('dq_%d [rad/s]', j));
    title(sprintf('PD - Joint %d', j));
end

sgtitle('Diagramas de fase del controlador PD con perturbación');

%% ===================== FIGURA 3: phase portraits separados CTC =====================
f3 = figure('Name','Phase Portraits CTC','Color','w');

for k = 1:length(joint_idx)
    j = joint_idx(k);

    q_ctc  = ctc_js{:, sprintf('pos_%d', j)};
    dq_ctc = ctc_js{:, sprintf('vel_%d', j)};

    subplot(1,3,k);
    plot(q_ctc, dq_ctc, 'LineWidth', 1.2);
    grid on;
    xlabel(sprintf('q_%d [rad]', j));
    ylabel(sprintf('dq_%d [rad/s]', j));
    title(sprintf('CTC-like - Joint %d', j));
end

sgtitle('Diagramas de fase del controlador CTC con perturbación');

%% ===================== Guardar =====================
if save_figs
    exportgraphics(f1, fullfile(out_dir, 'phase_portraits_overlay.png'), 'Resolution', 220);
    exportgraphics(f2, fullfile(out_dir, 'phase_portraits_pd.png'), 'Resolution', 220);
    exportgraphics(f3, fullfile(out_dir, 'phase_portraits_ctc.png'), 'Resolution', 220);
end