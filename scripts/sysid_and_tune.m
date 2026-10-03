% sysid_and_tune.m
% T-200 Thruster: System Identification and PID Tuning
clear; clc; close all;

%% 1. Load Data
disp('Loading step_response.csv...');
try
    data = readtable('step_response.csv');
catch
    error('Could not find step_response.csv. Did the Python logger run successfully?');
end

t = data.Time_s;
u = data.Compensated_Duty;
if any(strcmp(data.Properties.VariableNames, 'RPM'))
    y = data.RPM * (pi / 30); % Convert old CSVs to rad/s
else
    y = data.Rad_s;
end

% Ensure time starts at exactly 0
t = t - t(1);
Ts = mean(diff(t)); % Calculate average sample time (should be ~0.02s for 50Hz)
disp(['Average Sample Time: ', num2str(Ts), ' seconds (', num2str(1/Ts), ' Hz)']);

%% 2. System Identification
disp('Performing System Identification...');
% Create IDDATA object for MATLAB
data_id = iddata(y, u, Ts);

% Fit an ARX discrete model (2 poles, 1 zero, 1 sample delay)
sys_arx = arx(data_id, [2 1 1]);

% Fit a Continuous FOPDT model (1 pole, 0 zeros, auto-estimated delay)
% Note: Requires System Identification Toolbox
try
    sys_tf = tfest(data_id, 1, 0, NaN);
    
    disp('--------------------------------------------------');
    disp('Continuous Transfer Function (FOPDT) Estimated:');
    sys_tf
    disp('--------------------------------------------------');
    
    % Extract for control design
    P = tf(sys_tf.Numerator, sys_tf.Denominator, 'InputDelay', sys_tf.IODelay);
catch ME
    disp('tfest failed (possibly missing System ID Toolbox). Falling back to basic ARX.');
    P = d2c(tf(sys_arx.A, sys_arx.B, Ts)); % Convert discrete to continuous
end

%% 3. Controller Tuning
disp('Calculating Optimal PID Gains...');
% We use a PI or PID controller. Thrusters usually only need PI because 
% they are 1st order systems, but we'll ask MATLAB for a PIDF (PID with filter)
try
    opts = pidtuneOptions('DesignFocus', 'reference-tracking');
    [C, info] = pidtune(P, 'PIDF', opts);
    
    disp('==================================================');
    disp('OPTIMAL PID GAINS (MATLAB pidtune):');
    disp(['Kp = ', num2str(C.Kp, 6)]);
    disp(['Ki = ', num2str(C.Ki, 6)]);
    disp(['Kd = ', num2str(C.Kd, 6)]);
    disp('==================================================');
    disp('Controller Info:');
    disp(info);
    
    disp('Saving figures...');
    % 1. Raw Data Plot
    f1 = figure('Visible','off');
    subplot(2,1,1);
    plot(t, u, 'b', 'LineWidth', 1.5);
    title('Input: Commanded Duty'); ylabel('Duty'); grid on;
    subplot(2,1,2);
    plot(t, y, 'r', 'LineWidth', 1.5);
    title('Output: Motor Velocity'); xlabel('Time (s)'); ylabel('Velocity (rad/s)'); grid on;
    saveas(f1, 'sysid_raw_data.png');
    
    % 2. Model Fit Plot (using lsim to avoid GUI figure spawning issues)
    f2 = figure('Visible','off');
    [y_sim, t_sim] = lsim(sys_tf, u, t);
    plot(t, y, 'k', 'LineWidth', 1.5); hold on;
    plot(t_sim, y_sim, 'b--', 'LineWidth', 1.5);
    title('FOPDT Model Fit vs Real Data (rad/s)');
    xlabel('Time (s)'); ylabel('Velocity (rad/s)');
    legend('Real Data', 'Model Prediction', 'Location', 'best');
    grid on;
    saveas(f2, 'sysid_model_fit.png');
    
    % 3. Closed Loop Simulation Plot
    f3 = figure('Visible','off');
    T_closed = feedback(C*P, 1);
    step(T_closed);
    title('Theoretical Closed-Loop Step Response (with Optimal Gains)');
    grid on;
    saveas(f3, 'sysid_closed_loop.png');
    
    save('sysid_results.mat', 'sys_tf', 'C', 'P');


catch
    disp('Control System Toolbox missing for pidtune. Cannot calculate gains automatically.');
end

% Note: We use -batch so MATLAB will exit automatically when done.
disp('Script finished successfully.');
