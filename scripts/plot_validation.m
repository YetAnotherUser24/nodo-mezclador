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

f1 = figure('Visible', 'off', 'Position', [100, 100, 800, 600]);
subplot(2,1,1);
plot(t, duty, 'b', 'LineWidth', 1.5);
title('Controller Output (Compensated Duty)');
ylabel('Duty'); grid on;

subplot(2,1,2);
plot(t, vel, 'r', 'LineWidth', 1.5); hold on;
yline(target_vel, 'k--', 'LineWidth', 1.5);
title(['Motor Velocity (', vel_str, ')']);
xlabel('Time (s)'); ylabel(vel_str);
legend('Measured', 'Target'); grid on;

saveas(f1, 'closed_loop_validation.png');
disp('Validation plot saved to closed_loop_validation.png');
