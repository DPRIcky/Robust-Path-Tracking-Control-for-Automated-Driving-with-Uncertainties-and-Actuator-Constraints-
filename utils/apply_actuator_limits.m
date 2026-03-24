function [delta_applied, delta_rate_limited] = apply_actuator_limits(delta_cmd, delta_prev, Ts, delta_max, delta_rate_max)
%APPLY_ACTUATOR_LIMITS Apply steering angle and rate limits
%   [DELTA_APPLIED, DELTA_RATE_LIMITED] = APPLY_ACTUATOR_LIMITS(DELTA_CMD, DELTA_PREV, TS, DELTA_MAX, DELTA_RATE_MAX)
%   computes the applied steering angle and the limited rate after applying
%   rate and angle saturation.
%
%   Inputs:
%       delta_cmd   - commanded steering angle (before limits) [rad]
%       delta_prev  - previous applied steering angle [rad]
%       Ts          - sample time [s]
%       delta_max   - maximum steering angle [rad]
%       delta_rate_max - maximum steering rate [rad/s]
%
%   Outputs:
%       delta_applied   - applied steering angle after limits [rad]
%       delta_rate_limited - limited steering rate [rad/s]

    % Rate limit
    delta_rate_cmd = (delta_cmd - delta_prev) / Ts;
    delta_rate_limited = max(-delta_rate_max, min(delta_rate_max, delta_rate_cmd));

    % Apply rate-limited delta change
    delta_rl = delta_prev + Ts * delta_rate_limited;

    % Angle limit
    delta_applied = max(-delta_max, min(delta_max, delta_rl));
end