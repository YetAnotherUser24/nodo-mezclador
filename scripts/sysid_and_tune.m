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
y = data.RPM;

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
    
catch
    disp('Control System Toolbox missing for pidtune. Cannot calculate gains automatically.');
end

% Note: We use -batch so MATLAB will exit automatically when done.
disp('Script finished successfully.');
