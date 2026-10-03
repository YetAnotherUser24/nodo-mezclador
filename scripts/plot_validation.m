% plot_validation.m
clear; clc;
data = readtable('closed_loop_test.csv');
t = data.Time_s;
duty = data.Compensated_Duty;

if any(strcmp(data.Properties.VariableNames, 'RPM'))
    vel = data.RPM;
    target_vel = 1500;
    vel_str = 'RPM';
else
    vel = data.Rad_s;
    target_vel = 1500 * (pi/30);
    vel_str = 'Velocity (rad/s)';
end

f1 = figure('Visible', 'off', 'Position', [100, 100, 800, 800]);

% 1. Plot Controller Output
subplot(2,1,1);
plot(t, duty, 'b', 'LineWidth', 1.5);
title('Controller Output (Compensated Duty)');
ylabel('Duty'); grid on;

% 2. Plot Velocity and Simulator Data
subplot(2,1,2);
plot(t, vel, 'r', 'LineWidth', 1.5); hold on;
yline(target_vel, 'k--', 'LineWidth', 1.5);

leg_str = {'Measured', 'Target'};

% Load sysid data if available
sim_stats = ''; real_stats = '';
if isfile('sysid_results.mat')
    load('sysid_results.mat', 'sys_tf', 'C', 'P');
    % Create Closed-Loop Transfer Function
    T_closed = feedback(C*P, 1);
    
    % Simulate Step Response (from t=0 to max(t))
    [vel_sim, t_sim] = lsim(T_closed, target_vel * ones(size(t)), t);
    
    plot(t_sim, vel_sim, 'b--', 'LineWidth', 1.5);
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

title({['Motor Velocity (', vel_str, ')'], sim_stats, real_stats});
xlabel('Time (s)'); ylabel(vel_str);
legend(leg_str, 'Location', 'best'); grid on;

saveas(f1, 'closed_loop_validation.png');
disp('Validation plot saved to closed_loop_validation.png');
disp(real_stats);
if ~isempty(sim_stats)
    disp(sim_stats);
end
