function kappa_ref = generate_lane_change_kappa(t, kappa_peak, Tmaneuver)
%GENERATE_LANE_CHANGE_KAPPA Generate reference curvature for a lane change maneuver
%   KAPPA_REF = GENERATE_LANE_CHANGE_KAPPA(T, KAPPA_PEAK, TMANEUVER) returns
%   the reference curvature (1/m) for a smooth lane change maneuver.
%   The curvature starts at 0, rises smoothly to KAPPA_PEAK at Tmaneuver/2,
%   and returns to 0 at Tmaneuver, then remains 0.
%
%   Inputs:
%       t           - Time vector (s)
%       kappa_peak  - Peak curvature (1/m)
%       Tmaneuver   - Total duration of maneuver (s)
%
%   Output:
%       kappa_ref   - Reference curvature vector same size as t

kappa_ref = zeros(size(t));
% Define the maneuver period
in_maneuver = t >= 0 & t <= Tmaneuver;
% Use a raised cosine (Hann window) for smooth curvature
kappa_ref(in_maneuver) = kappa_peak * 0.5 * (1 - cos(2*pi*t(in_maneuver)/Tmaneuver));
end