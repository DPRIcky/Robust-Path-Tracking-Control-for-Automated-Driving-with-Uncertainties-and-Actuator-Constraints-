% EGR 560 Project - Phase 1 Baseline: LQR Path Tracking
% Author: [Your Name]
% Date: %s
%
% TODO List for Steps 1-5:
% Step 1: Implement bicycle model dynamics (continuous/discrete) - DONE
% Step 2: Build reference generator + open-loop simulation harness - DONE
% Step 3: Design LQR controller for path tracking - DONE
% Step 4: Incorporate actuator constraints (saturation, rates) - DONE
% Step 5: Prepare for Phase 2: online disturbance-bound estimation

%% Initialization
clear; close all; clc;

% Add utils directory to MATLAB path for helper functions
scriptDir = fileparts(mfilename('fullpath'));
addpath(fullfile(scriptDir, '..', 'utils'));

%% Parameters and Sample Time
% Vehicle parameters (from CarSim)
m  = 1573;      % kg
Iz = 2031.4;    % kg-m^2
lf = 1.04;      % m
lr = 1.56;      % m
Cf = 52618;     % N/rad
Cr = 110185;    % N/rad
Vx = 10;        % m/s (longitudinal velocity)

% Sample time for discrete-time model
Ts = 0.02;      % s (50 Hz, chosen for adequate sampling of vehicle dynamics)
% Justification: Ts=0.02s provides sufficient resolution for lateral dynamics
% while keeping computational load manageable for real-time implementation.

%% Bicycle Model Lateral Dynamics (Continuous Time)
% States: x_lat = [v_y; r]  (lateral velocity, yaw rate)
% Input:  u_lat = delta      (steering angle)
%
% A_lat = [-(Cf+Cr)/(m*Vx),  -(Vx + (Cf*lf - Cr*lr)/(m*Vx));
%          -(Cf*lf - Cr*lr)/(Iz*Vx),  -(Cf*lf^2 + Cr*lr^2)/(Iz*Vx)];
% B_lat = [Cf/m; Cf*lf/Iz];

A_lat = [-(Cf+Cr)/(m*Vx),             -(Vx + (Cf*lf - Cr*lr)/(m*Vx));
         -(Cf*lf - Cr*lr)/(Iz*Vx),    -(Cf*lf^2 + Cr*lr^2)/(Iz*Vx)];
B_lat = [Cf/m; Cf*lf/Iz];

%% Augmented Tracking Model (Continuous Time)
% States: x = [e_y; e_psi; v_y; r]
%   e_y: lateral error
%   e_psi: heading error
%
% Inputs:
%   delta: steering angle
%   kappa_ref: reference curvature (1/R)
%
% Model: dx/dt = A_track*x + B_track*delta + E_track*kappa_ref
%
% Derivation:
%   dot(e_y)   = Vx*e_psi + v_y
%   dot(e_psi) = r - Vx*kappa_ref
%   dot(v_y), dot(r) from bicycle lateral dynamics above
%
% Therefore:
%   A_track = [0, Vx, 1, 0;
%              0, 0, 0, 1;
%              0, 0, A_lat(1,1), A_lat(1,2);
%              0, 0, A_lat(2,1), A_lat(2,2)];
%   B_track = [0; 0; B_lat(1); B_lat(2)];
%   E_track = [0; -Vx; 0; 0];

A_track = [0, Vx, 1, 0;
           0, 0, 0, 1;
           0, 0, A_lat(1,1), A_lat(1,2);
           0, 0, A_lat(2,1), A_lat(2,2)];
B_track = [0; 0; B_lat(1); B_lat(2)];
E_track = [0; -Vx; 0; 0];

%% Discretization (Zero-Order Hold)
% Discretize the combined system [B_track E_track] to get Ad, Bd, Ed
sysc = ss(A_track, [B_track E_track], eye(4), zeros(4,2));
sysd = c2d(sysc, Ts);
Ad = sysd.A;
Bd = sysd.B(:,1);   % from delta
Ed = sysd.B(:,2);   % from kappa_ref

Bplant = [Bd Ed];
Dplant = zeros(4,2);

%% Sanity Check
fprintf('=== Step 1: Bicycle Model Discretization Complete ===\n');
fprintf('Sample time Ts = %.3f s\n\n', Ts);

fprintf('Continuous-time matrices:\n');
fprintf('A_track (4x4):\n'); disp(A_track);
fprintf('B_track (4x1):\n'); disp(B_track);
fprintf('E_track (4x1):\n'); disp(E_track);

fprintf('\nDiscrete-time matrices:\n');
fprintf('Ad (4x4):\n'); disp(Ad);
fprintf('Bd (4x1):\n'); disp(Bd);
fprintf('Ed (4x1):\n'); disp(Ed);

fprintf('\nEigenvalues of Ad (magnitudes):\n');
lambda = eig(Ad);
disp(abs(lambda));
insideUnitCircle = all(abs(lambda) < 1);
if insideUnitCircle
    fprintf('All eigenvalues inside unit circle? Yes\n');
else
    fprintf('All eigenvalues inside unit circle? No\n');
end
fprintf('==================================================\n\n');

%% Step 2: Reference Generator and Open-Loop Simulation
fprintf('=== Step 2: Reference Generator and Open-Loop Simulation ===\n');

% Simulation parameters
Tsim = 10.0;           % total simulation time (s)
t = 0:Ts:Tsim;         % time vector
N = length(t);

% Scenario A: Lane change-like smooth curvature burst
kappa_peak = 0.02;     % peak curvature (1/m)
Tmaneuver  = 5.0;      % maneuver duration (s)
kappa_ref_A = generate_lane_change_kappa(t, kappa_peak, Tmaneuver);
kappaA_ts = [t(:) kappa_ref_A(:)];
% Scenario B: Constant-radius turn
kappa_const = 0.01;    % constant curvature (1/m)
Tconst      = 5.0;     % duration of constant curvature (s)
kappa_ref_B = generate_constant_turn_kappa(t, kappa_const, Tconst);
kappaB_ts = [t(:) kappa_ref_B(:)];
% Open-loop simulation (delta = 0 for all k)
% States: x = [e_y; e_psi; v_y; r]
% Initial condition: zero initial state
x0 = [0; 0; 0; 0];

% Preallocate state trajectories
x_A = zeros(4, N);
x_B = zeros(4, N);
x_A(:,1) = x0;
x_B(:,1) = x0;

% Simulation loop for scenario A
for k = 1:N-1
    x_A(:,k+1) = Ad * x_A(:,k) + Bd * 0 + Ed * kappa_ref_A(k);
end

% Simulation loop for scenario B
for k = 1:N-1
    x_B(:,k+1) = Ad * x_B(:,k) + Bd * 0 + Ed * kappa_ref_B(k);
end

% Extract errors
e_y_A   = x_A(1,:);
e_psi_A = x_A(2,:);
e_y_B   = x_B(1,:);
e_psi_B = x_B(2,:);

% Plot results - Scenario A
figure('Name', 'Step 2: Lane Change Scenario', 'NumberTitle', 'off');
subplot(3,1,1);
plot(t, kappa_ref_A, 'LineWidth', 1.5);
ylabel('\kappa_{ref} (1/m)');
title('Scenario A: Lane Change - Reference Curvature');
grid on;

subplot(3,1,2);
plot(t, e_y_A, 'LineWidth', 1.5);
ylabel('e_y (m)');
title('Lateral Error');
grid on;

subplot(3,1,3);
plot(t, e_psi_A, 'LineWidth', 1.5);
ylabel('e_\psi (rad)');
xlabel('Time (s)');
title('Heading Error');
grid on;

export_figure_clean(gcf, fullfile(scriptDir, '..', 'plots', 'scenario_A_lane_change.png'));

% Plot results - Scenario B
figure('Name', 'Step 2: Constant Turn Scenario', 'NumberTitle', 'off');
subplot(3,1,1);
plot(t, kappa_ref_B, 'LineWidth', 1.5);
ylabel('\kappa_{ref} (1/m)');
title('Scenario B: Constant Turn - Reference Curvature');
grid on;

subplot(3,1,2);
plot(t, e_y_B, 'LineWidth', 1.5);
ylabel('e_y (m)');
title('Lateral Error');
grid on;

subplot(3,1,3);
plot(t, e_psi_B, 'LineWidth', 1.5);
ylabel('e_\psi (rad)');
xlabel('Time (s)');
title('Heading Error');
grid on;

export_figure_clean(gcf, fullfile(scriptDir, '..', 'plots', 'scenario_B_constant_turn.png'));

% Sanity checks
fprintf('\n--- Step 2 Sanity Checks ---\n');
fprintf('Scenario A (Lane Change):\n');
fprintf('  kappa_peak = %.3f 1/m, Tmaneuver = %.1f s\n', kappa_peak, Tmaneuver);
fprintf('  kappa_ref starts at %.4f, ends at %.4f (should be near 0)\n', kappa_ref_A(1), kappa_ref_A(end));
fprintf('  Time vector length: %d, Ts = %.3f s, Tsim = %.1f s\n', N, Ts, Tsim);
fprintf('  State vector size: %d x %d\n', size(x_A,1), size(x_A,2));

fprintf('Scenario B (Constant Turn):\n');
fprintf('  kappa_const = %.3f 1/m, Tconst = %.1f s\n', kappa_const, Tconst);
fprintf('  kappa_ref during turn: %.4f (should be %.3f)\n', kappa_ref_B(find(t>=0 & t<=Tconst,1)), kappa_const);
fprintf('  kappa_ref starts at %.4f, ends at %.4f (should be 0 after turn)\n', kappa_ref_B(1), kappa_ref_B(end));
fprintf('  Time vector length: %d, Ts = %.3f s, Tsim = %.1f s\n', N, Ts, Tsim);
fprintf('  State vector size: %d x %d\n', size(x_B,1), size(x_B,2));
fprintf('============================\n\n');

fprintf('Step 2 complete. Plots saved to project/plots/\n');

%% Step 3: LQR Baseline Controller
fprintf('=== Step 3: LQR Baseline Controller ===\n');

% LQR design
Q = diag([1000, 300, 5, 5]);  % weights on [e_y, e_psi, v_y, r]
R = 1;                        % weight on delta
[K, ~, ~] = dlqr(Ad, Bd, Q, R);
fprintf('LQR gain K = [% .4f % .4f % .4f % .4f]\n', K);

% Closed-loop eigenvalues
cl_eig = eig(Ad - Bd*K);
fprintf('Closed-loop eigenvalues (Ad - Bd*K):\n');
disp(cl_eig);

% Feedforward gain: delta_ff = (lf+lr) * kappa_ref
wheelbase = lf + lr;

% Preallocate for closed-loop simulation
x_cl_A = zeros(4, N);
x_cl_B = zeros(4, N);
x_cl_A(:,1) = x0;
x_cl_B(:,1) = x0;
delta_cmd_A = zeros(1, N);
delta_cmd_B = zeros(1, N);

% Simulation loops for closed-loop
for k = 1:N-1
    % Scenario A
    delta_ff_A = wheelbase * kappa_ref_A(k);
    delta_cmd_A(k) = delta_ff_A - K * x_cl_A(:,k);
    x_cl_A(:,k+1) = Ad * x_cl_A(:,k) + Bd * delta_cmd_A(k) + Ed * kappa_ref_A(k);

    % Scenario B
    delta_ff_B = wheelbase * kappa_ref_B(k);
    delta_cmd_B(k) = delta_ff_B - K * x_cl_B(:,k);
    x_cl_B(:,k+1) = Ad * x_cl_B(:,k) + Bd * delta_cmd_B(k) + Ed * kappa_ref_B(k);
end

% Compute last control command (optional)
delta_cmd_A(N) = wheelbase * kappa_ref_A(N) - K * x_cl_A(:,N);
delta_cmd_B(N) = wheelbase * kappa_ref_B(N) - K * x_cl_B(:,N);

% Extract errors
e_y_cl_A   = x_cl_A(1,:);
e_psi_cl_A = x_cl_A(2,:);
e_y_cl_B   = x_cl_B(1,:);
e_psi_cl_B = x_cl_B(2,:);

% Compute metrics
e_y_rms_A = sqrt(mean(e_y_cl_A.^2));
e_y_max_A = max(abs(e_y_cl_A));
delta_max_A = max(abs(delta_cmd_A));

e_y_rms_B = sqrt(mean(e_y_cl_B.^2));
e_y_max_B = max(abs(e_y_cl_B));
delta_max_B = max(abs(delta_cmd_B));

fprintf('\n--- Scenario A (Lane Change) Metrics ---\n');
fprintf('  RMS(e_y) = %.4f m\n', e_y_rms_A);
fprintf('  Max|e_y| = %.4f m\n', e_y_max_A);
fprintf('  Max|delta| = %.4f rad (%.1f deg)\n', delta_max_A, delta_max_A*180/pi);

fprintf('--- Scenario B (Constant Turn) Metrics ---\n');
fprintf('  RMS(e_y) = %.4f m\n', e_y_rms_B);
fprintf('  Max|e_y| = %.4f m\n', e_y_max_B);
fprintf('  Max|delta| = %.4f rad (%.1f deg)\n', delta_max_B, delta_max_B*180/pi);

% Plot results for scenario A
figure('Name', 'Step 3: Lane Change LQR', 'NumberTitle', 'off');
subplot(4,1,1);
plot(t, kappa_ref_A, 'LineWidth', 1.5);
ylabel('\kappa_{ref} (1/m)');
title('Scenario A: Lane Change - Reference Curvature');
grid on;

subplot(4,1,2);
plot(t, e_y_cl_A, 'LineWidth', 1.5);
ylabel('e_y (m)');
title('Lateral Error');
grid on;

subplot(4,1,3);
plot(t, e_psi_cl_A, 'LineWidth', 1.5);
ylabel('e_\psi (rad)');
title('Heading Error');
grid on;

subplot(4,1,4);
plot(t, delta_cmd_A, 'LineWidth', 1.5);
ylabel('\delta (rad)');
xlabel('Time (s)');
title('Steering Command');
grid on;

export_figure_clean(gcf, fullfile(scriptDir, '..', 'plots', 'scenario_A_lane_change_LQR.png'));

% Plot results for scenario B
figure('Name', 'Step 3: Constant Turn LQR', 'NumberTitle', 'off');
subplot(4,1,1);
plot(t, kappa_ref_B, 'LineWidth', 1.5);
ylabel('\kappa_{ref} (1/m)');
title('Scenario B: Constant Turn - Reference Curvature');
grid on;

subplot(4,1,2);
plot(t, e_y_cl_B, 'LineWidth', 1.5);
ylabel('e_y (m)');
title('Lateral Error');
grid on;

subplot(4,1,3);
plot(t, e_psi_cl_B, 'LineWidth', 1.5);
ylabel('e_\psi (rad)');
title('Heading Error');
grid on;

subplot(4,1,4);
plot(t, delta_cmd_B, 'LineWidth', 1.5);
ylabel('\delta (rad)');
xlabel('Time (s)');
title('Steering Command');
grid on;

export_figure_clean(gcf, fullfile(scriptDir, '..', 'plots', 'scenario_B_constant_turn_LQR.png'));

fprintf('\nStep 3 complete. LQR plots saved to project/plots/\n');

%% Step 4: Actuator Constraints (steering angle + rate limits)
fprintf('=== Step 4: Actuator Constraints ===\n');

% Actuator constraints (editable parameters)
delta_max = 0.5;        % rad
delta_rate_max = 0.5;   % rad/s

% For unconstrained simulation, set limits to infinity
delta_max_uncon = inf;
delta_rate_max_uncon = inf;

%% Scenario A: Lane change
fprintf('\n--- Scenario A: Lane Change ---\n');

% Unconstrained
delta_prev = 0;
x_A_uncon = zeros(4, N);
delta_cmd_A_uncon = zeros(1, N);
delta_applied_A_uncon = zeros(1, N);
delta_rate_A_uncon = zeros(1, N);
x_A_uncon(:,1) = x0;

for k = 1:N-1
    delta_ff = wheelbase * kappa_ref_A(k);
    delta_cmd = delta_ff - K * x_A_uncon(:,k);
    [delta_applied_A_uncon(k), delta_rate_A_uncon(k)] = apply_actuator_limits( ...
        delta_cmd, delta_prev, Ts, delta_max_uncon, delta_rate_max_uncon);
    x_A_uncon(:,k+1) = Ad * x_A_uncon(:,k) + Bd * delta_applied_A_uncon(k) + Ed * kappa_ref_A(k);
    delta_prev = delta_applied_A_uncon(k);
    delta_cmd_A_uncon(k) = delta_cmd;
end
delta_cmd_A_uncon(N) = wheelbase * kappa_ref_A(N) - K * x_A_uncon(:,N);

% Constrained
delta_prev = 0;
x_A_con = zeros(4, N);
delta_cmd_A_con = zeros(1, N);
delta_applied_A_con = zeros(1, N);
delta_rate_A_con = zeros(1, N);
x_A_con(:,1) = x0;

for k = 1:N-1
    delta_ff = wheelbase * kappa_ref_A(k);
    delta_cmd = delta_ff - K * x_A_con(:,k);
    [delta_applied_A_con(k), delta_rate_A_con(k)] = apply_actuator_limits( ...
        delta_cmd, delta_prev, Ts, delta_max, delta_rate_max);
    x_A_con(:,k+1) = Ad * x_A_con(:,k) + Bd * delta_applied_A_con(k) + Ed * kappa_ref_A(k);
    delta_prev = delta_applied_A_con(k);
    delta_cmd_A_con(k) = delta_cmd;
end
delta_cmd_A_con(N) = wheelbase * kappa_ref_A(N) - K * x_A_con(:,N);

%% Scenario B: Constant turn
fprintf('\n--- Scenario B: Constant Turn ---\n');

% Unconstrained
delta_prev = 0;
x_B_uncon = zeros(4, N);
delta_cmd_B_uncon = zeros(1, N);
delta_applied_B_uncon = zeros(1, N);
delta_rate_B_uncon = zeros(1, N);
x_B_uncon(:,1) = x0;

for k = 1:N-1
    delta_ff = wheelbase * kappa_ref_B(k);
    delta_cmd = delta_ff - K * x_B_uncon(:,k);
    [delta_applied_B_uncon(k), delta_rate_B_uncon(k)] = apply_actuator_limits( ...
        delta_cmd, delta_prev, Ts, delta_max_uncon, delta_rate_max_uncon);
    x_B_uncon(:,k+1) = Ad * x_B_uncon(:,k) + Bd * delta_applied_B_uncon(k) + Ed * kappa_ref_B(k);
    delta_prev = delta_applied_B_uncon(k);
    delta_cmd_B_uncon(k) = delta_cmd;
end
delta_cmd_B_uncon(N) = wheelbase * kappa_ref_B(N) - K * x_B_uncon(:,N);

% Constrained
delta_prev = 0;
x_B_con = zeros(4, N);
delta_cmd_B_con = zeros(1, N);
delta_applied_B_con = zeros(1, N);
delta_rate_B_con = zeros(1, N);
x_B_con(:,1) = x0;

for k = 1:N-1
    delta_ff = wheelbase * kappa_ref_B(k);
    delta_cmd = delta_ff - K * x_B_con(:,k);
    [delta_applied_B_con(k), delta_rate_B_con(k)] = apply_actuator_limits( ...
        delta_cmd, delta_prev, Ts, delta_max, delta_rate_max);
    x_B_con(:,k+1) = Ad * x_B_con(:,k) + Bd * delta_applied_B_con(k) + Ed * kappa_ref_B(k);
    delta_prev = delta_applied_B_con(k);
    delta_cmd_B_con(k) = delta_cmd;
end
delta_cmd_B_con(N) = wheelbase * kappa_ref_B(N) - K * x_B_con(:,N);

%% Compute metrics for unconstrained and constrained
e_y_rms_A_uncon = sqrt(mean(x_A_uncon(1,:).^2));
e_y_max_A_uncon = max(abs(x_A_uncon(1,:)));
delta_max_A_uncon = max(abs(delta_cmd_A_uncon));
delta_rate_max_A_uncon = max(abs(delta_rate_A_uncon));

e_y_rms_A_con = sqrt(mean(x_A_con(1,:).^2));
e_y_max_A_con = max(abs(x_A_con(1,:)));
delta_max_A_con = max(abs(delta_applied_A_con));
delta_rate_max_A_con = max(abs(delta_rate_A_con));

rate_limited_A = abs(delta_rate_A_con) >= delta_rate_max - 1e-9;
angle_saturated_A = abs(delta_applied_A_con) >= delta_max - 1e-9;
perc_rate_limited_A = 100 * sum(rate_limited_A) / N;
perc_angle_saturated_A = 100 * sum(angle_saturated_A) / N;

e_y_rms_B_uncon = sqrt(mean(x_B_uncon(1,:).^2));
e_y_max_B_uncon = max(abs(x_B_uncon(1,:)));
delta_max_B_uncon = max(abs(delta_cmd_B_uncon));
delta_rate_max_B_uncon = max(abs(delta_rate_B_uncon));

e_y_rms_B_con = sqrt(mean(x_B_con(1,:).^2));
e_y_max_B_con = max(abs(x_B_con(1,:)));
delta_max_B_con = max(abs(delta_applied_B_con));
delta_rate_max_B_con = max(abs(delta_rate_B_con));

rate_limited_B = abs(delta_rate_B_con) >= delta_rate_max - 1e-9;
angle_saturated_B = abs(delta_applied_B_con) >= delta_max - 1e-9;
perc_rate_limited_B = 100 * sum(rate_limited_B) / N;
perc_angle_saturated_B = 100 * sum(angle_saturated_B) / N;

%% Print metrics
fprintf('\n=== Scenario A (Lane Change) Metrics ===\n');
fprintf('Unconstrained:\n');
fprintf('  RMS(e_y) = %.4f m, Max|e_y| = %.4f m\n', e_y_rms_A_uncon, e_y_max_A_uncon);
fprintf('  Max|delta| = %.4f rad (%.1f deg)\n', delta_max_A_uncon, delta_max_A_uncon*180/pi);
fprintf('  Max|delta_rate| = %.4f rad/s\n', delta_rate_max_A_uncon);

fprintf('Constrained:\n');
fprintf('  RMS(e_y) = %.4f m, Max|e_y| = %.4f m\n', e_y_rms_A_con, e_y_max_A_con);
fprintf('  Max|delta_applied| = %.4f rad (%.1f deg)\n', delta_max_A_con, delta_max_A_con*180/pi);
fprintf('  Max|delta_rate| = %.4f rad/s\n', delta_rate_max_A_con);
fprintf('  %% time rate-limited = %.3f%%\n', perc_rate_limited_A);
fprintf('  %% time angle-saturated = %.3f%%\n\n', perc_angle_saturated_A);

fprintf('=== Scenario B (Constant Turn) Metrics ===\n');
fprintf('Unconstrained:\n');
fprintf('  RMS(e_y) = %.4f m, Max|e_y| = %.4f m\n', e_y_rms_B_uncon, e_y_max_B_uncon);
fprintf('  Max|delta| = %.4f rad (%.1f deg)\n', delta_max_B_uncon, delta_max_B_uncon*180/pi);
fprintf('  Max|delta_rate| = %.4f rad/s\n', delta_rate_max_B_uncon);

fprintf('Constrained:\n');
fprintf('  RMS(e_y) = %.4f m, Max|e_y| = %.4f m\n', e_y_rms_B_con, e_y_max_B_con);
fprintf('  Max|delta_applied| = %.4f rad (%.1f deg)\n', delta_max_B_con, delta_max_B_con*180/pi);
fprintf('  Max|delta_rate| = %.4f rad/s\n', delta_rate_max_B_con);
fprintf('  %% time rate-limited = %.3f%%\n', perc_rate_limited_B);
fprintf('  %% time angle-saturated = %.3f%%\n\n', perc_angle_saturated_B);

%% Sensitivity tests (only if constraints do not bind)
if perc_rate_limited_A < 1 && perc_angle_saturated_A < 1
    fprintf('\n--- Sensitivity Test for Scenario A (Tight Actuator Limits) ---\n');
    delta_max_sens = 0.1;
    delta_rate_max_sens = 0.1;

    delta_prev = 0;
    x_A_sens = zeros(4, N);
    delta_cmd_A_sens = zeros(1, N);
    delta_applied_A_sens = zeros(1, N);
    delta_rate_A_sens = zeros(1, N);
    x_A_sens(:,1) = x0;

    for k = 1:N-1
        delta_ff = wheelbase * kappa_ref_A(k);
        delta_cmd = delta_ff - K * x_A_sens(:,k);
        [delta_applied_A_sens(k), delta_rate_A_sens(k)] = apply_actuator_limits( ...
            delta_cmd, delta_prev, Ts, delta_max_sens, delta_rate_max_sens);
        x_A_sens(:,k+1) = Ad * x_A_sens(:,k) + Bd * delta_applied_A_sens(k) + Ed * kappa_ref_A(k);
        delta_prev = delta_applied_A_sens(k);
        delta_cmd_A_sens(k) = delta_cmd;
    end

    e_y_rms_A_sens = sqrt(mean(x_A_sens(1,:).^2));
    e_y_max_A_sens = max(abs(x_A_sens(1,:)));
    delta_max_A_sens = max(abs(delta_applied_A_sens));
    delta_rate_max_A_sens = max(abs(delta_rate_A_sens));

    rate_limited_A_sens = abs(delta_rate_A_sens) >= delta_rate_max_sens - 1e-9;
    angle_saturated_A_sens = abs(delta_applied_A_sens) >= delta_max_sens - 1e-9;

    perc_rate_limited_A_sens = 100 * sum(rate_limited_A_sens) / N;
    perc_angle_saturated_A_sens = 100 * sum(angle_saturated_A_sens) / N;

    fprintf('  RMS(e_y) = %.4f m\n', e_y_rms_A_sens);
    fprintf('  Max|e_y| = %.4f m\n', e_y_max_A_sens);
    fprintf('  Max|delta_applied| = %.4f rad (%.1f deg)\n', delta_max_A_sens, delta_max_A_sens*180/pi);
    fprintf('  Max|delta_rate| = %.4f rad/s\n', delta_rate_max_A_sens);
    fprintf('  %% time rate-limited = %.3f%%\n', perc_rate_limited_A_sens);
    fprintf('  %% time angle-saturated = %.3f%%\n', perc_angle_saturated_A_sens);
end

if perc_rate_limited_B < 1 && perc_angle_saturated_B < 1
    fprintf('\n--- Sensitivity Test for Scenario B (Tight Actuator Limits) ---\n');
    delta_max_sens = 0.1;
    delta_rate_max_sens = 0.1;

    delta_prev = 0;
    x_B_sens = zeros(4, N);
    delta_cmd_B_sens = zeros(1, N);
    delta_applied_B_sens = zeros(1, N);
    delta_rate_B_sens = zeros(1, N);
    x_B_sens(:,1) = x0;

    for k = 1:N-1
        delta_ff = wheelbase * kappa_ref_B(k);
        delta_cmd = delta_ff - K * x_B_sens(:,k);
        [delta_applied_B_sens(k), delta_rate_B_sens(k)] = apply_actuator_limits( ...
            delta_cmd, delta_prev, Ts, delta_max_sens, delta_rate_max_sens);
        x_B_sens(:,k+1) = Ad * x_B_sens(:,k) + Bd * delta_applied_B_sens(k) + Ed * kappa_ref_B(k);
        delta_prev = delta_applied_B_sens(k);
        delta_cmd_B_sens(k) = delta_cmd;
    end

    e_y_rms_B_sens = sqrt(mean(x_B_sens(1,:).^2));
    e_y_max_B_sens = max(abs(x_B_sens(1,:)));
    delta_max_B_sens = max(abs(delta_applied_B_sens));
    delta_rate_max_B_sens = max(abs(delta_rate_B_sens));

    rate_limited_B_sens = abs(delta_rate_B_sens) >= delta_rate_max_sens - 1e-9;
    angle_saturated_B_sens = abs(delta_applied_B_sens) >= delta_max_sens - 1e-9;

    perc_rate_limited_B_sens = 100 * sum(rate_limited_B_sens) / N;
    perc_angle_saturated_B_sens = 100 * sum(angle_saturated_B_sens) / N;

    fprintf('  RMS(e_y) = %.4f m\n', e_y_rms_B_sens);
    fprintf('  Max|e_y| = %.4f m\n', e_y_max_B_sens);
    fprintf('  Max|delta_applied| = %.4f rad (%.1f deg)\n', delta_max_B_sens, delta_max_B_sens*180/pi);
    fprintf('  Max|delta_rate| = %.4f rad/s\n', delta_rate_max_B_sens);
    fprintf('  %% time rate-limited = %.3f%%\n', perc_rate_limited_B_sens);
    fprintf('  %% time angle-saturated = %.3f%%\n', perc_angle_saturated_B_sens);
end

%% Generate comparison plots
figure('Name', 'Step 4: Lane Change Comparison', 'NumberTitle', 'off');
subplot(4,1,1);
plot(t, kappa_ref_A, 'LineWidth', 1.5);
ylabel('\kappa_{ref} (1/m)');
title('Scenario A: Lane Change - Reference Curvature');
grid on;

subplot(4,1,2);
plot(t, x_A_uncon(1,:), 'LineWidth', 1.5, 'DisplayName', 'e_y unconstrained'); hold on;
plot(t, x_A_con(1,:), 'LineWidth', 1.5, 'DisplayName', 'e_y constrained');
ylabel('e_y (m)');
title('Lateral Error');
legend('Location', 'best');
grid on;

subplot(4,1,3);
plot(t, delta_cmd_A_uncon, 'LineWidth', 1.5, 'DisplayName', 'delta_cmd unconstrained'); hold on;
plot(t, delta_applied_A_con, 'LineWidth', 1.5, 'DisplayName', 'delta_applied constrained');
ylabel('\delta (rad)');
xlabel('Time (s)');
title('Steering Command');
legend('Location', 'best');
grid on;

subplot(4,1,4);
plot(t, delta_rate_A_uncon, 'LineWidth', 1.5, 'DisplayName', 'delta_rate unconstrained'); hold on;
plot(t, delta_rate_A_con, 'LineWidth', 1.5, 'DisplayName', 'delta_rate constrained');
ylabel('\delta_{rate} (rad/s)');
xlabel('Time (s)');
title('Steering Rate');
legend('Location', 'best');
grid on;

export_figure_clean(gcf, fullfile(scriptDir, '..', 'plots', 'scenario_A_lane_change_LQR_constraints.png'));

figure('Name', 'Step 4: Constant Turn Comparison', 'NumberTitle', 'off');
subplot(4,1,1);
plot(t, kappa_ref_B, 'LineWidth', 1.5);
ylabel('\kappa_{ref} (1/m)');
title('Scenario B: Constant Turn - Reference Curvature');
grid on;

subplot(4,1,2);
plot(t, x_B_uncon(1,:), 'LineWidth', 1.5, 'DisplayName', 'e_y unconstrained'); hold on;
plot(t, x_B_con(1,:), 'LineWidth', 1.5, 'DisplayName', 'e_y constrained');
ylabel('e_y (m)');
title('Lateral Error');
legend('Location', 'best');
grid on;

subplot(4,1,3);
plot(t, delta_cmd_B_uncon, 'LineWidth', 1.5, 'DisplayName', 'delta_cmd unconstrained'); hold on;
plot(t, delta_applied_B_con, 'LineWidth', 1.5, 'DisplayName', 'delta_applied constrained');
ylabel('\delta (rad)');
xlabel('Time (s)');
title('Steering Command');
legend('Location', 'best');
grid on;

subplot(4,1,4);
plot(t, delta_rate_B_uncon, 'LineWidth', 1.5, 'DisplayName', 'delta_rate unconstrained'); hold on;
plot(t, delta_rate_B_con, 'LineWidth', 1.5, 'DisplayName', 'delta_rate constrained');
ylabel('\delta_{rate} (rad/s)');
xlabel('Time (s)');
title('Steering Rate');
legend('Location', 'best');
grid on;

export_figure_clean(gcf, fullfile(scriptDir, '..', 'plots', 'scenario_B_constant_turn_LQR_constraints.png'));

fprintf('\nStep 4 complete. Comparison plots saved to project/plots/\n');

%% Step 5: Uncertainty + Crosswind Disturbance Experiments
fprintf('=== Step 5: Uncertainty + Crosswind Disturbance Experiments ===\n');

plotsDir = fullfile(scriptDir, '..', 'plots');
if ~exist(plotsDir, 'dir')
    mkdir(plotsDir);
end

%% A) Parameter uncertainty
fprintf('\n--- Step 5A: Parameter Uncertainty ---\n');

% Controller remains fixed at the nominal LQR gain K.
% The perturbed plant is used for simulation only.
use_secondary_uncertainty = false;
Cf_real = 0.7 * Cf;
Cr_real = 0.7 * Cr;
m_real = m;
Iz_real = Iz;

if use_secondary_uncertainty
    m_real = 1.2 * m;
    Iz_real = 1.1 * Iz;
end

[~, ~, ~, ~, ~, Ad_real, Bd_real, Ed_real] = build_tracking_model( ...
    m_real, Iz_real, lf, lr, Cf_real, Cr_real, Vx, Ts);

uncertainty_nominal_A = simulate_tracking_case( ...
    Ad, Bd, Ed, K, wheelbase, kappa_ref_A, x0, Ts, delta_max, delta_rate_max);
uncertainty_nominal_B = simulate_tracking_case( ...
    Ad, Bd, Ed, K, wheelbase, kappa_ref_B, x0, Ts, delta_max, delta_rate_max);
uncertainty_real_A = simulate_tracking_case( ...
    Ad_real, Bd_real, Ed_real, K, wheelbase, kappa_ref_A, x0, Ts, delta_max, delta_rate_max);
uncertainty_real_B = simulate_tracking_case( ...
    Ad_real, Bd_real, Ed_real, K, wheelbase, kappa_ref_B, x0, Ts, delta_max, delta_rate_max);

metrics_unc_nom_A = compute_tracking_metrics( ...
    uncertainty_nominal_A.x, uncertainty_nominal_A.delta_applied, uncertainty_nominal_A.delta_rate, ...
    delta_max, delta_rate_max);
metrics_unc_real_A = compute_tracking_metrics( ...
    uncertainty_real_A.x, uncertainty_real_A.delta_applied, uncertainty_real_A.delta_rate, ...
    delta_max, delta_rate_max);
metrics_unc_nom_B = compute_tracking_metrics( ...
    uncertainty_nominal_B.x, uncertainty_nominal_B.delta_applied, uncertainty_nominal_B.delta_rate, ...
    delta_max, delta_rate_max);
metrics_unc_real_B = compute_tracking_metrics( ...
    uncertainty_real_B.x, uncertainty_real_B.delta_applied, uncertainty_real_B.delta_rate, ...
    delta_max, delta_rate_max);

fprintf('Real plant parameters: Cf_real = %.1f N/rad, Cr_real = %.1f N/rad', Cf_real, Cr_real);
if use_secondary_uncertainty
    fprintf(', m_real = %.1f kg, Iz_real = %.1f kg-m^2\n', m_real, Iz_real);
else
    fprintf(' (secondary mass/inertia uncertainty disabled)\n');
end
print_metrics_table('Scenario A: Lane Change', {'Nominal', 'Uncertainty'}, ...
    [metrics_unc_nom_A, metrics_unc_real_A]);
print_metrics_table('Scenario B: Constant Turn', {'Nominal', 'Uncertainty'}, ...
    [metrics_unc_nom_B, metrics_unc_real_B]);

fig_unc_A = figure('Name', 'Step 5A: Lane Change Uncertainty', 'NumberTitle', 'off');
subplot(3,1,1);
plot(t, kappa_ref_A, 'LineWidth', 1.5);
ylabel('\kappa_{ref} (1/m)');
title('Scenario A: Lane Change - Parameter Uncertainty');
grid on;

subplot(3,1,2);
plot(t, uncertainty_nominal_A.x(1,:), 'LineWidth', 1.5, 'DisplayName', 'Nominal plant'); hold on;
plot(t, uncertainty_real_A.x(1,:), 'LineWidth', 1.5, 'DisplayName', 'Real plant');
ylabel('e_y (m)');
title('Lateral Error');
legend('Location', 'best');
grid on;

subplot(3,1,3);
plot(t, uncertainty_nominal_A.delta_applied, 'LineWidth', 1.5, 'DisplayName', 'Nominal \delta_{applied}'); hold on;
plot(t, uncertainty_real_A.delta_cmd, '--', 'LineWidth', 1.2, 'DisplayName', 'Real \delta_{cmd}');
plot(t, uncertainty_real_A.delta_applied, 'LineWidth', 1.5, 'DisplayName', 'Real \delta_{applied}');
ylabel('\delta (rad)');
xlabel('Time (s)');
title('Steering Comparison');
legend('Location', 'best');
grid on;

export_figure_clean(fig_unc_A, fullfile(plotsDir, 'scenario_A_lane_change_uncertainty.png'));

fig_unc_B = figure('Name', 'Step 5A: Constant Turn Uncertainty', 'NumberTitle', 'off');
subplot(3,1,1);
plot(t, kappa_ref_B, 'LineWidth', 1.5);
ylabel('\kappa_{ref} (1/m)');
title('Scenario B: Constant Turn - Parameter Uncertainty');
grid on;

subplot(3,1,2);
plot(t, uncertainty_nominal_B.x(1,:), 'LineWidth', 1.5, 'DisplayName', 'Nominal plant'); hold on;
plot(t, uncertainty_real_B.x(1,:), 'LineWidth', 1.5, 'DisplayName', 'Real plant');
ylabel('e_y (m)');
title('Lateral Error');
legend('Location', 'best');
grid on;

subplot(3,1,3);
plot(t, uncertainty_nominal_B.delta_applied, 'LineWidth', 1.5, 'DisplayName', 'Nominal \delta_{applied}'); hold on;
plot(t, uncertainty_real_B.delta_cmd, '--', 'LineWidth', 1.2, 'DisplayName', 'Real \delta_{cmd}');
plot(t, uncertainty_real_B.delta_applied, 'LineWidth', 1.5, 'DisplayName', 'Real \delta_{applied}');
ylabel('\delta (rad)');
xlabel('Time (s)');
title('Steering Comparison');
legend('Location', 'best');
grid on;

export_figure_clean(fig_unc_B, fullfile(plotsDir, 'scenario_B_constant_turn_uncertainty.png'));

fprintf('Uncertainty plots saved to project/plots/\n');

%% B) Crosswind disturbance
fprintf('\n--- Step 5B: Crosswind Disturbance ---\n');

F_wind = 500;         % N
wind_start = 2.0;     % s
wind_end = 3.0;       % s
d_wind = zeros(1, N);
d_wind(t >= wind_start & t <= wind_end) = F_wind;

% Discrete disturbance channel: lateral force [N] induces a step-wise
% change in v_y of Ts * F_wind / m over each sample.
Gd = [0; 0; Ts / m; 0];

wind_nominal_A = simulate_tracking_case( ...
    Ad, Bd, Ed, K, wheelbase, kappa_ref_A, x0, Ts, delta_max, delta_rate_max);
wind_nominal_B = simulate_tracking_case( ...
    Ad, Bd, Ed, K, wheelbase, kappa_ref_B, x0, Ts, delta_max, delta_rate_max);
wind_disturbed_A = simulate_tracking_case( ...
    Ad, Bd, Ed, K, wheelbase, kappa_ref_A, x0, Ts, delta_max, delta_rate_max, d_wind, Gd);
wind_disturbed_B = simulate_tracking_case( ...
    Ad, Bd, Ed, K, wheelbase, kappa_ref_B, x0, Ts, delta_max, delta_rate_max, d_wind, Gd);

metrics_wind_nom_A = compute_tracking_metrics( ...
    wind_nominal_A.x, wind_nominal_A.delta_applied, wind_nominal_A.delta_rate, ...
    delta_max, delta_rate_max);
metrics_wind_dist_A = compute_tracking_metrics( ...
    wind_disturbed_A.x, wind_disturbed_A.delta_applied, wind_disturbed_A.delta_rate, ...
    delta_max, delta_rate_max);
metrics_wind_nom_B = compute_tracking_metrics( ...
    wind_nominal_B.x, wind_nominal_B.delta_applied, wind_nominal_B.delta_rate, ...
    delta_max, delta_rate_max);
metrics_wind_dist_B = compute_tracking_metrics( ...
    wind_disturbed_B.x, wind_disturbed_B.delta_applied, wind_disturbed_B.delta_rate, ...
    delta_max, delta_rate_max);

fprintf('Wind pulse: %.1f N from t = %.1f s to t = %.1f s\n', F_wind, wind_start, wind_end);
fprintf('Using Gd = [0; 0; Ts/m; 0] = [%g; %g; %g; %g]\n', Gd(1), Gd(2), Gd(3), Gd(4));
print_metrics_table('Scenario A: Lane Change', {'No wind', 'Wind'}, ...
    [metrics_wind_nom_A, metrics_wind_dist_A]);
print_metrics_table('Scenario B: Constant Turn', {'No wind', 'Wind'}, ...
    [metrics_wind_nom_B, metrics_wind_dist_B]);

fig_wind_A = figure('Name', 'Step 5B: Lane Change Wind', 'NumberTitle', 'off');
subplot(4,1,1);
plot(t, kappa_ref_A, 'LineWidth', 1.5);
ylabel('\kappa_{ref} (1/m)');
title('Scenario A: Lane Change - Crosswind Disturbance');
grid on;

subplot(4,1,2);
plot(t, d_wind, 'LineWidth', 1.5);
ylabel('F_{wind} (N)');
title('Wind Profile');
grid on;

subplot(4,1,3);
plot(t, wind_nominal_A.x(1,:), 'LineWidth', 1.5, 'DisplayName', 'No wind'); hold on;
plot(t, wind_disturbed_A.x(1,:), 'LineWidth', 1.5, 'DisplayName', 'Wind');
ylabel('e_y (m)');
title('Lateral Error');
legend('Location', 'best');
grid on;

subplot(4,1,4);
plot(t, wind_nominal_A.delta_applied, 'LineWidth', 1.5, 'DisplayName', 'No wind'); hold on;
plot(t, wind_disturbed_A.delta_applied, 'LineWidth', 1.5, 'DisplayName', 'Wind');
ylabel('\delta_{applied} (rad)');
xlabel('Time (s)');
title('Applied Steering');
legend('Location', 'best');
grid on;

export_figure_clean(fig_wind_A, fullfile(plotsDir, 'scenario_A_lane_change_wind.png'));

fig_wind_B = figure('Name', 'Step 5B: Constant Turn Wind', 'NumberTitle', 'off');
subplot(4,1,1);
plot(t, kappa_ref_B, 'LineWidth', 1.5);
ylabel('\kappa_{ref} (1/m)');
title('Scenario B: Constant Turn - Crosswind Disturbance');
grid on;

subplot(4,1,2);
plot(t, d_wind, 'LineWidth', 1.5);
ylabel('F_{wind} (N)');
title('Wind Profile');
grid on;

subplot(4,1,3);
plot(t, wind_nominal_B.x(1,:), 'LineWidth', 1.5, 'DisplayName', 'No wind'); hold on;
plot(t, wind_disturbed_B.x(1,:), 'LineWidth', 1.5, 'DisplayName', 'Wind');
ylabel('e_y (m)');
title('Lateral Error');
legend('Location', 'best');
grid on;

subplot(4,1,4);
plot(t, wind_nominal_B.delta_applied, 'LineWidth', 1.5, 'DisplayName', 'No wind'); hold on;
plot(t, wind_disturbed_B.delta_applied, 'LineWidth', 1.5, 'DisplayName', 'Wind');
ylabel('\delta_{applied} (rad)');
xlabel('Time (s)');
title('Applied Steering');
legend('Location', 'best');
grid on;

export_figure_clean(fig_wind_B, fullfile(plotsDir, 'scenario_B_constant_turn_wind.png'));

fprintf('Wind disturbance plots saved to project/plots/\n');

%% Step 6: Option A Novelty - Online Disturbance-Bound Estimation
fprintf('\n=== Step 6: Option A Novelty - Online Disturbance-Bound Estimation ===\n');

%% Scenario selection and reusable trajectories
fprintf('\n--- Step 6: Scenario B Constant Turn ---\n');

% Reuse stored Step 5 trajectories and compare all cases against the same
% nominal predictor model (Ad, Bd, Ed).
x_nom = wind_nominal_B.x;
delta_nom_applied = wind_nominal_B.delta_applied;
x_unc = uncertainty_real_B.x;
delta_unc_applied = uncertainty_real_B.delta_applied;
x_wind = wind_disturbed_B.x;
delta_wind_applied = wind_disturbed_B.delta_applied;
kappa_ref_step6 = kappa_ref_B;

% Rolling window for online bound estimation (editable).
M_wmax = 50;   % samples, 1 second at Ts = 0.02 s
maneuver_time = 6.0;   % s
residual_noise_sigma = 5e-5;
residual_noise_seed = 560;
sigma_meas = 5e-5;

%% Residual and rolling bound estimation
rng(residual_noise_seed, 'twister');
[w_hat_nom, w_inf_nom, w_max_nom] = compute_disturbance_bound( ...
    x_nom, delta_nom_applied, kappa_ref_step6, Ad, Bd, Ed, M_wmax, residual_noise_sigma);
[w_hat_unc, w_inf_unc, w_max_unc] = compute_disturbance_bound( ...
    x_unc, delta_unc_applied, kappa_ref_step6, Ad, Bd, Ed, M_wmax, residual_noise_sigma);
[w_hat_wind, w_inf_wind, w_max_wind] = compute_disturbance_bound( ...
    x_wind, delta_wind_applied, kappa_ref_step6, Ad, Bd, Ed, M_wmax, residual_noise_sigma);

stats_nom = summarize_disturbance_metrics(w_inf_nom, w_max_nom, t, maneuver_time);
stats_unc = summarize_disturbance_metrics(w_inf_unc, w_max_unc, t, maneuver_time);
stats_wind = summarize_disturbance_metrics(w_inf_wind, w_max_wind, t, maneuver_time);

fprintf('Nominal predictor uses Ad, Bd, Ed from the baseline model for all cases.\n');
fprintf('Rolling window M = %d samples (%.2f s)\n', M_wmax, M_wmax * Ts);
fprintf('Residual predictor measurement noise sigma = %.2e, applied only to x_meas in Ad*x_meas + Bd*u + Ed*kappa.\n', ...
    residual_noise_sigma);
print_disturbance_summary('Scenario B: Constant Turn', ...
    {'Nominal', 'Uncertainty', 'Wind'}, [stats_nom, stats_unc, stats_wind]);

%% Novelty plots
fig_what_B = figure('Name', 'Step 6: Scenario B w_inf Overlay', 'NumberTitle', 'off');
subplot(2,1,1);
plot(t, kappa_ref_step6, 'LineWidth', 1.5);
ylabel('\kappa_{ref} (1/m)');
title('Scenario B: Constant Turn - Novelty Residual');
grid on;

subplot(2,1,2);
plot(t, w_inf_nom, 'LineWidth', 1.5, 'DisplayName', 'Nominal'); hold on;
plot(t, w_inf_unc, 'LineWidth', 1.5, 'DisplayName', 'Uncertainty');
plot(t, w_inf_wind, 'LineWidth', 1.5, 'DisplayName', 'Wind');
ylabel('w_{\infty}(k)');
xlabel('Time (s)');
title('Instantaneous Residual Bound');
legend('Location', 'best');
grid on;

export_figure_clean(fig_what_B, fullfile(plotsDir, 'scenario_B_what_overlay.png'));

fig_wmax_B = figure('Name', 'Step 6: Scenario B w_max Overlay', 'NumberTitle', 'off');
subplot(2,1,1);
plot(t, kappa_ref_step6, 'LineWidth', 1.5);
ylabel('\kappa_{ref} (1/m)');
title('Scenario B: Constant Turn - Rolling Disturbance Bound');
grid on;

subplot(2,1,2);
plot(t, w_max_nom, 'LineWidth', 1.5, 'DisplayName', 'Nominal'); hold on;
plot(t, w_max_unc, 'LineWidth', 1.5, 'DisplayName', 'Uncertainty');
plot(t, w_max_wind, 'LineWidth', 1.5, 'DisplayName', 'Wind');
ylabel('w_{max}(k)');
xlabel('Time (s)');
title('Rolling Maximum of w_{\infty}');
legend('Location', 'best');
grid on;

export_figure_clean(fig_wmax_B, fullfile(plotsDir, 'scenario_B_wmax_overlay.png'));

fprintf('Step 6 novelty plots saved to project/plots/\n');

function [A_lat, B_lat, A_track, B_track, E_track, Ad, Bd, Ed] = build_tracking_model( ...
    m, Iz, lf, lr, Cf, Cr, Vx, Ts)
%BUILD_TRACKING_MODEL Rebuild continuous and discrete tracking models.

    A_lat = [-(Cf+Cr)/(m*Vx),             -(Vx + (Cf*lf - Cr*lr)/(m*Vx));
             -(Cf*lf - Cr*lr)/(Iz*Vx),    -(Cf*lf^2 + Cr*lr^2)/(Iz*Vx)];
    B_lat = [Cf/m; Cf*lf/Iz];

    A_track = [0, Vx, 1, 0;
               0, 0, 0, 1;
               0, 0, A_lat(1,1), A_lat(1,2);
               0, 0, A_lat(2,1), A_lat(2,2)];
    B_track = [0; 0; B_lat(1); B_lat(2)];
    E_track = [0; -Vx; 0; 0];

    sysc = ss(A_track, [B_track E_track], eye(4), zeros(4,2));
    sysd = c2d(sysc, Ts);
    Ad = sysd.A;
    Bd = sysd.B(:,1);
    Ed = sysd.B(:,2);
end

function sim = simulate_tracking_case(Ad, Bd, Ed, K, wheelbase, kappa_ref, x0, Ts, ...
    delta_max, delta_rate_max, disturbance_profile, Gd)
%SIMULATE_TRACKING_CASE Run constrained closed-loop tracking simulation.

    N_local = numel(kappa_ref);
    x = zeros(4, N_local);
    delta_cmd = zeros(1, N_local);
    delta_applied = zeros(1, N_local);
    delta_rate = zeros(1, N_local);
    x(:,1) = x0;

    if nargin < 11 || isempty(disturbance_profile)
        disturbance_profile = zeros(1, N_local);
    end
    if nargin < 12 || isempty(Gd)
        Gd = zeros(4,1);
    end

    delta_prev = 0;
    for k = 1:N_local-1
        delta_ff = wheelbase * kappa_ref(k);
        delta_cmd(k) = delta_ff - K * x(:,k);
        [delta_applied(k), delta_rate(k)] = apply_actuator_limits( ...
            delta_cmd(k), delta_prev, Ts, delta_max, delta_rate_max);
        x(:,k+1) = Ad * x(:,k) + Bd * delta_applied(k) + Ed * kappa_ref(k) + Gd * disturbance_profile(k);
        delta_prev = delta_applied(k);
    end

    delta_cmd(N_local) = wheelbase * kappa_ref(N_local) - K * x(:,N_local);
    delta_applied(N_local) = delta_prev;
    if N_local > 1
        delta_rate(N_local) = delta_rate(N_local-1);
    end

    sim = struct('x', x, ...
                 'delta_cmd', delta_cmd, ...
                 'delta_applied', delta_applied, ...
                 'delta_rate', delta_rate);
end

function metrics = compute_tracking_metrics(x, delta_applied, delta_rate, delta_max, delta_rate_max)
%COMPUTE_TRACKING_METRICS Summarize tracking and actuator performance.

    active_idx = 1:max(numel(delta_applied) - 1, 1);
    e_y = x(1,:);

    metrics.e_y_rms = sqrt(mean(e_y.^2));
    metrics.e_y_max = max(abs(e_y));
    metrics.delta_applied_max = max(abs(delta_applied(active_idx)));
    metrics.delta_rate_max = max(abs(delta_rate(active_idx)));
    metrics.perc_rate_limited = 100 * mean(abs(delta_rate(active_idx)) >= delta_rate_max - 1e-9);
    metrics.perc_angle_saturated = 100 * mean(abs(delta_applied(active_idx)) >= delta_max - 1e-9);
end

function print_metrics_table(scenario_name, case_names, metrics_array)
%PRINT_METRICS_TABLE Print a compact comparison table for one scenario.

    fprintf('\n%s\n', scenario_name);
    fprintf('%-14s %12s %12s %18s %18s %18s %18s\n', ...
        'Case', 'RMS(e_y)', 'Max|e_y|', 'Max|delta_app|', 'Max|delta_rate|', ...
        '% rate-limited', '% angle-sat');
    for idx = 1:numel(case_names)
        metrics = metrics_array(idx);
        fprintf('%-14s %12.4f %12.4f %18.4f %18.4f %18.3f %18.3f\n', ...
            case_names{idx}, metrics.e_y_rms, metrics.e_y_max, ...
            metrics.delta_applied_max, metrics.delta_rate_max, ...
            metrics.perc_rate_limited, metrics.perc_angle_saturated);
    end
end

function [w_hat, w_inf, w_max] = compute_disturbance_bound(x, delta_applied, kappa_ref, Ad, Bd, Ed, M, noise_sigma)
%COMPUTE_DISTURBANCE_BOUND Estimate model-mismatch residual and rolling bound.

    N_local = size(x, 2);
    w_hat = NaN(size(x));
    w_inf = NaN(1, N_local);
    w_max = NaN(1, N_local);

    for k = 1:N_local-1
        x_meas = x(:,k) + noise_sigma * randn(size(x, 1), 1);
        x_pred = Ad * x_meas + Bd * delta_applied(k) + Ed * kappa_ref(k);
        w_hat(:,k) = x(:,k+1) - x_pred;
        w_inf(k) = norm(w_hat(:,k), Inf);

        window_start = max(1, k - M + 1);
        w_max(k) = max(w_inf(window_start:k));
    end
end

function stats = summarize_disturbance_metrics(w_inf, w_max, t, maneuver_time)
%SUMMARIZE_DISTURBANCE_METRICS Compute summary statistics for novelty signals.

    valid_idx = ~isnan(w_inf);
    maneuver_idx = valid_idx & (t <= maneuver_time);

    stats.max_w_inf = max(w_inf(valid_idx));
    stats.max_w_max = max(w_max(valid_idx));
    stats.mean_w_inf_maneuver = mean(w_inf(maneuver_idx));
    stats.mean_w_inf_full = mean(w_inf(valid_idx));
end

function print_disturbance_summary(scenario_name, case_names, stats_array)
%PRINT_DISTURBANCE_SUMMARY Print compact novelty-signal statistics.

    fprintf('\n%s\n', scenario_name);
    fprintf('%-14s %14s %14s %20s %18s\n', ...
        'Case', 'max(w_inf)', 'max(w_max)', 'mean(w_inf), t<=6s', 'mean(w_inf), all');
    for idx = 1:numel(case_names)
        stats = stats_array(idx);
        fprintf('%-14s %14.6g %14.6g %20.6g %18.6g\n', ...
            case_names{idx}, stats.max_w_inf, stats.max_w_max, ...
            stats.mean_w_inf_maneuver, stats.mean_w_inf_full);
    end
end

function export_figure_clean(fig_handle, filename)
%EXPORT_FIGURE_CLEAN Hide axes toolbars before exporting a figure.

    ax_handles = findall(fig_handle, 'Type', 'axes');
    for idx = 1:numel(ax_handles)
        if isprop(ax_handles(idx), 'Toolbar') && ~isempty(ax_handles(idx).Toolbar)
            ax_handles(idx).Toolbar.Visible = 'off';
        end
    end
    exportgraphics(fig_handle, filename, 'Resolution', 300);
end
