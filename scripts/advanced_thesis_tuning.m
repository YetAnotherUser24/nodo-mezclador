% advanced_thesis_tuning.m
% Implements Phase 2 and Phase 3 of the System ID Plan
clearvars -except force_model_idx; clc; close all;

%% 1. Load and Prepare Data
disp('Loading System ID Data...');
data = readtable('data/step_response.csv');
t = data.Time_s; t = t - t(1);
Ts = mean(diff(t));

if any(strcmp(data.Properties.VariableNames, 'RPM'))
    y = data.RPM * (pi / 30);
else
    y = data.Rad_s;
end
u = data.Compensated_Duty;

% Create iddata object
data_id = iddata(y, u, Ts, 'Name', 'Thruster Data');
data_id.InputName = 'Duty'; data_id.OutputName = 'Velocity';
data_id.InputUnit = '%'; data_id.OutputUnit = 'rad/s';
data_id.TimeUnit = 's';

%% 2. Phase 2: Plant Approximation (Compare Discrete Models & AIC)
disp('--------------------------------------------------');
disp('PHASE 2: Model Order Selection & Estimation');
disp('Estimating various models...');

% Justification for selected models:
% FOPDT: Theoretical baseline for thrusters (1st order + delay).
% ARX(1,1,1): Pure discrete 1st-order equivalent.
% ARX(2,2,1): 2nd-order discrete (checks if electrical inertia is significant).
% ARMAX(2,2,2,1): Adds colored noise modeling (can sometimes overfit).
% OE(2,2,1): Output Error model, completely separates noise from dynamics.

sys_fopdt = tfest(data_id, 1, 0, NaN);

% Extract the estimated physical delay and convert to discrete samples (nk)
physical_delay_s = sys_fopdt.IODelay;
nk = max(1, round(physical_delay_s / Ts));
fprintf('Detected physical dead-time: %.2f seconds (nk = %d samples)\n', physical_delay_s, nk);

sys_arx1 = arx(data_id, [1 1 nk]);
sys_arx2 = arx(data_id, [2 2 nk]);
sys_armax = armax(data_id, [2 2 2 nk]);
sys_oe = oe(data_id, [2 2 nk]);

models = {sys_fopdt, sys_arx1, sys_arx2, sys_armax, sys_oe};
names = {'FOPDT (Cont)', sprintf('ARX(1,1,%d)', nk), sprintf('ARX(2,2,%d)', nk), sprintf('ARMAX(2,2,2,%d)', nk), sprintf('OE(2,2,%d)', nk)};

fprintf('\n%-18s | %-10s | %-10s | %-10s\n', 'Model', 'Fit (%)', 'AIC', 'FPE');
fprintf(repmat('-', 1, 55)); fprintf('\n');

best_aic = inf;
best_model_idx = 1;

for i = 1:length(models)
    fit = models{i}.Report.Fit.FitPercent;
    aic_val = aic(models{i});
    fpe = models{i}.Report.Fit.FPE;
    fprintf('%-18s | %-10.2f | %-10.2f | %-10.2f\n', names{i}, fit, aic_val, fpe);
    
    if aic_val < best_aic && i > 1 % Only consider discrete
        best_aic = aic_val;
        best_model_idx = i;
    end
end

% Allow manual override
if exist('force_model_idx', 'var') && force_model_idx > 0 && force_model_idx <= length(models)
    disp(['--> FORCING MODEL OVERRIDE TO: ', names{force_model_idx}]);
    best_model_idx = force_model_idx;
end

disp('--------------------------------------------------');
best_model = models{best_model_idx};
fprintf('Selected Discrete Model: %s\n\n', names{best_model_idx});
disp('Polynomials / Transfer Function:');
present(best_model);

% Plot Visual Comparison of all models
f_comp = figure('Visible','off', 'Position', [100, 100, 1000, 600]);
plot(t, y, 'k', 'LineWidth', 2.0); hold on;
u_id = data_id(:,[],1);
plot(t, sim(sys_fopdt, u_id).OutputData, 'LineWidth', 1.5);
plot(t, sim(sys_arx1, u_id).OutputData, 'LineWidth', 1.5);
plot(t, sim(sys_arx2, u_id).OutputData, 'LineWidth', 1.5);
plot(t, sim(sys_armax, u_id).OutputData, 'LineWidth', 1.5, 'LineStyle', '--');
plot(t, sim(sys_oe, u_id).OutputData, 'LineWidth', 1.5);

title('Model Order Visual Comparison');
xlabel('Time (s)'); ylabel('Velocity (rad/s)');
legend(['Real Data', names], 'Location', 'best');
set(gca, 'FontSize', 12, 'FontWeight', 'bold'); grid on; grid minor;
saveas(f_comp, 'results/figures/pso/sysid_model_comparison.png');
disp('Saved visual comparison to results/figures/pso/sysid_model_comparison.png');

% Plot isolated fit for the BEST model
f_best_fit = figure('Visible','off', 'Position', [100, 100, 1000, 600]);
plot(t, y, 'k', 'LineWidth', 2.0); hold on;
plot(t, sim(best_model, u_id).OutputData, 'b--', 'LineWidth', 2.0);
title(['Advanced Model Fit: ', names{best_model_idx}, ' vs Real Data']);
xlabel('Time (s)'); ylabel('Velocity (rad/s)');
legend('Real Data', 'Model Prediction', 'Location', 'best');
set(gca, 'FontSize', 12, 'FontWeight', 'bold'); grid on; grid minor;
saveas(f_best_fit, 'results/figures/pso/sysid_model_fit.png');
disp('Saved isolated best model fit to results/figures/pso/sysid_model_fit.png');

%% 3. Phase 3: Cost-Function Optimization (ITAE with PSO)
disp('--------------------------------------------------');
disp('PHASE 3: Objective Functions & Metaheuristics Optimization');
disp('Using strictly DISCRETE control (no d2c conversion needed!)');

% 1. ITAE Cost Function Definition (Discrete Time)
    function J = itae_cost_discrete(K, P, Ts)
        C = pid(K(1), K(2), 0, 'Ts', Ts, 'IFormula', 'BackwardEuler');
        T = feedback(C * P, 1);
        t_sim = 0:Ts:5;
        try
            [y_sim, ~] = step(T, t_sim);
            e = 1 - y_sim;
            J = sum(t_sim(:) .* abs(e(:))) * Ts;
        catch
            J = 1e6;
        end
    end

% 2. Custom Multi-Objective Cost Function (Heavy Overshoot Penalty)
    function J = custom_cost_discrete(K, P, Ts)
        C = pid(K(1), K(2), 0, 'Ts', Ts, 'IFormula', 'BackwardEuler');
        T = feedback(C * P, 1);
        t_sim = 0:Ts:5;
        try
            si = stepinfo(T, 'SettlingTimeThreshold', 0.05);
            [y_sim, ~] = step(T, t_sim);
            
            OS = si.Overshoot;
            if isnan(OS), OS = 100; end
            
            % If it overshoots by more than 2%, massive penalty!
            if OS > 2.0
                penalty = 1000 * OS;
            else
                penalty = 0;
            end
            
            e = 1 - y_sim;
            ITAE = sum(t_sim(:) .* abs(e(:))) * Ts;
            
            J = ITAE + penalty;
        catch
            J = 1e6;
        end
    end

lb = [0.0001, 0.0001]; 
ub = [0.1, 0.1];
opts_pso = optimoptions('particleswarm', 'SwarmSize', 30, 'MaxIterations', 30, 'Display', 'off');
opts_fmin = optimset('Display','off', 'MaxIter', 200);
K_init = [0.001, 0.001];

disp('1. Running Nelder-Mead (fminsearch) with ITAE (Local Search)...');
K_fmin = fminsearch(@(K) itae_cost_discrete(K, best_model, Ts), K_init, opts_fmin);

disp('2. Running Particle Swarm (PSO) with ITAE (Global Search)...');
K_pso_itae = particleswarm(@(K) itae_cost_discrete(K, best_model, Ts), 2, lb, ub, opts_pso);

disp('3. Running Particle Swarm (PSO) with Custom Objective (Global Search)...');
K_pso_custom = particleswarm(@(K) custom_cost_discrete(K, best_model, Ts), 2, lb, ub, opts_pso);

% --- Simulate and Plot the Comparison ---
C_fmin = pid(K_fmin(1), K_fmin(2), 0, 'Ts', Ts, 'IFormula', 'BackwardEuler');
C_pso_itae = pid(K_pso_itae(1), K_pso_itae(2), 0, 'Ts', Ts, 'IFormula', 'BackwardEuler');
C_pso_custom = pid(K_pso_custom(1), K_pso_custom(2), 0, 'Ts', Ts, 'IFormula', 'BackwardEuler');

T_fmin = feedback(C_fmin * best_model, 1);
T_pso_itae = feedback(C_pso_itae * best_model, 1);
T_pso_custom = feedback(C_pso_custom * best_model, 1);

fig2 = figure('Name', 'Optimization Algorithm Comparison', 'Position', [150, 150, 800, 500], 'Visible', 'off');
t_sim = 0:Ts:5;
[y_fmin, ~] = step(T_fmin, t_sim);
[y_itae, ~] = step(T_pso_itae, t_sim);
[y_custom, ~] = step(T_pso_custom, t_sim);

plot(t_sim, y_fmin, 'r--', 'LineWidth', 1.5); hold on;
plot(t_sim, y_itae, 'b-', 'LineWidth', 2);
plot(t_sim, y_custom, 'g-.', 'LineWidth', 2);
yline(1, 'k--', 'Target');
title('Optimization Algorithm Comparison on ARMAX Model');
xlabel('Time (s)'); ylabel('Amplitude');
legend('fminsearch (ITAE)', 'PSO (ITAE)', 'PSO (Custom Overshoot Penalty)', 'Location', 'Southeast');
grid on;

if ~exist('results/figures/pso', 'dir'), mkdir('results/figures/pso'); end
saveas(fig2, 'results/figures/pso/optimization_comparison.png');
disp('Saved comparison plot to results/figures/pso/optimization_comparison.png');

% Use PSO ITAE as the official export (Standard Benchmark)
Kp_opt = K_pso_itae(1);
Ki_opt = K_pso_itae(2);
Kd_opt = 0;

fprintf('\n==================================================\n');
fprintf('OPTIMAL PI GAINS (ITAE PSO):\n');
fprintf('Kp = %g\n', Kp_opt);
fprintf('Ki = %g\n', Ki_opt);
fprintf('Kd = %g\n', Kd_opt);
fprintf('==================================================\n');

fid = fopen('results/models/pso_gains.txt', 'w');
fprintf(fid, '%g,%g,%g\n', Kp_opt, Ki_opt, Kd_opt);
fclose(fid);
disp('Saved gains to results/models/pso_gains.txt for easy copying!');

% Save the model and ITAE optimal controller for plot_validation
C_opt = C_pso_itae;
P = best_model;
sys_tf = sys_fopdt; % Keep FOPDT for reference
C = C_opt;
save('results/models/sysid_results.mat', 'sys_tf', 'C_opt', 'C', 'P');

disp('Script finished successfully. Run plot_validation.m to test it on hardware!');
