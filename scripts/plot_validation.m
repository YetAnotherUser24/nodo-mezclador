% plot_validation.m
% Validates closed-loop hardware test data against theoretical models.
% Can be parameterized by pre-defining variables before running.

clearvars -except csv_file mat_file out_img target_rpm; 
clc;

% Default parameters if not set via command line
if ~exist('csv_file', 'var')
    csv_file = 'data/closed_loop_test.csv';
end
if ~exist('mat_file', 'var')
    mat_file = 'results/models/sysid_results.mat';
end
if ~exist('out_img', 'var')
    out_img = 'results/figures/closed_loop_validation.png';
end

if ~isfile(csv_file)
    fprintf('Validation data not found: %s\n', csv_file);
    return;
end

disp(['Loading data from: ', csv_file]);

data = readtable(csv_file);
t = data.Time_s;
duty = data.Compensated_Duty;

if any(strcmp(data.Properties.VariableNames, 'RPM'))
    vel = data.RPM * (pi/30);
    vel_str = 'Velocity (rad/s)';
else
    vel = data.Rad_s;
    vel_str = 'Velocity (rad/s)';
end

% Auto-detect Setpoint if available in CSV
if any(strcmp(data.Properties.VariableNames, 'Setpoint_RPM'))
    target_rpm = max(data.Setpoint_RPM);
    fprintf('Auto-detected Setpoint: %.1f RPM\n', target_rpm);
elseif ~exist('target_rpm', 'var')
    target_rpm = 1500; % Fallback
end

target_vel = target_rpm * (pi/30);

f1 = figure('Visible', 'off', 'Position', [100, 100, 800, 800]);

% 1. Plot Controller Output
subplot(2,1,1);
plot(t, duty, 'b', 'LineWidth', 2.0);
title('Controller Output (Compensated Duty)');
ylabel('Duty'); grid on; grid minor;
set(gca, 'FontSize', 12, 'FontWeight', 'bold');

% 2. Plot Velocity and Simulator Data
subplot(2,1,2);
plot(t, vel, 'r', 'LineWidth', 2.0); hold on;
yline(target_vel, 'k--', 'LineWidth', 2.0);

leg_str = {'Measured', 'Target'};

% Load sysid data if available
sim_stats = ''; real_stats = ''; gains_str = '';
if isfile(mat_file)
    load(mat_file, 'C', 'P');
    % Create Closed-Loop Transfer Function
    T_closed = feedback(C*P, 1);
    
    % Extract gains
    gains_str = sprintf('Gains: Kp=%.5g, Ki=%.5g, Kd=%.5g', C.Kp, C.Ki, C.Kd);
    
    % Extract actual input trajectory if available, otherwise assume step at t=0
    if any(strcmp(data.Properties.VariableNames, 'Setpoint_RPM'))
        u_sim = data.Setpoint_RPM * (pi/30);
    else
        u_sim = target_vel * ones(size(t));
    end
    
    % Simulate Step Response (from t=0 to max(t))
    [vel_sim, t_sim] = lsim(T_closed, u_sim, t);
    
    plot(t_sim, vel_sim, 'b--', 'LineWidth', 2.0);
    leg_str{end+1} = 'Simulated';
    
    % Calculate Characteristics for Simulated
    s_sim = stepinfo(vel_sim, t, target_vel);
    sim_es = abs(target_vel - mean(vel_sim(end-20:end)));
    sim_stats = sprintf('Simulated -> OS: %.1f%%, ts: %.2fs, es: %.2f', s_sim.Overshoot, s_sim.SettlingTime, sim_es);
end

% Calculate Characteristics for Real
s_real = stepinfo(vel, t, target_vel);
real_es = abs(target_vel - mean(vel(end-20:end)));
real_stats = sprintf('Measured -> OS: %.1f%%, ts: %.2fs, es: %.2f', s_real.Overshoot, s_real.SettlingTime, real_es);

% Title array dynamically includes gains if available
title_array = {['Motor Velocity (', vel_str, ')']};
if ~isempty(gains_str), title_array{end+1} = gains_str; end
if ~isempty(sim_stats), title_array{end+1} = sim_stats; end
title_array{end+1} = real_stats;

title(title_array);
xlabel('Time (s)'); ylabel(vel_str);
legend(leg_str, 'Location', 'best'); grid on; grid minor;
set(gca, 'FontSize', 12, 'FontWeight', 'bold');

saveas(f1, out_img);
fprintf('Validation plot saved to %s\n', out_img);
disp(real_stats);
if ~isempty(sim_stats)
    disp(sim_stats);
end
