function kappa_ref = generate_constant_turn_kappa(t, kappa_const, Tconst)
%GENERATE_CONSTANT_TURN_KAPPA Generate reference curvature for a constant radius turn
%   KAPPA_REF = GENERATE_CONSTANT_TURN_KAPPA(T, KAPPA_CONST, TCONST) returns
%   the reference curvature (1/m) for a constant radius turn maneuver.
%   The curvature is KAPPA_CONST for 0 <= t <= Tconst, and 0 otherwise.
%
%   Inputs:
%       t           - Time vector (s)
%       kappa_const - Constant curvature (1/m)
%       Tconst      - Duration of constant curvature (s)
%
%   Output:
%       kappa_ref   - Reference curvature vector same size as t

kappa_ref = zeros(size(t));
% Set curvature to kappa_const during the constant turn period
in_turn = t >= 0 & t <= Tconst;
kappa_ref(in_turn) = kappa_const;
end