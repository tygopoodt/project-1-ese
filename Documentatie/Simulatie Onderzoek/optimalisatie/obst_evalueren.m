function [J, Pv, Sg, Fv] = obst_evalueren(p, C, opstelling, seeds, var_seed0)
%OBST_EVALUEREN  Kost van elke kandidaat-instelling over een vaste set arena's.
%
%   Kost per kandidaat (lager = beter):
%       J = f_botsing + 0,25 (1 - s / (v0 T_run)) + 0,1 f_eis
%   f_botsing  fractie ritten met een botsing (F2: 'zonder een obstakel te raken')
%   s          gemiddeld afgelegde afstand (hoe 'snel en goed' hij rijdt)
%   f_eis      fractie stops die F2.3 of F2.4 schendt
%   Elke kandidaat rijdt in dezelfde arena's met dezelfde afwijkingen.

nc = size(C, 1);  na = numel(seeds);
[Bq, Aq, Sq, Vq] = deal(zeros(nc * na, 1));
parfor q = 1:nc * na
    [i, a] = ind2sub([nc na], q);
    pp = obst_zet(p, C(i, :), opstelling);
    w  = arena_cache(seeds(a));
    v  = variatie_trekken(pp, RandStream('mt19937ar', 'Seed', var_seed0 + seeds(a)));
    r  = sim_obstakels(pp, w, v);
    Bq(q) = r.botsing;  Aq(q) = r.afstand;  Sq(q) = r.n_stops;  Vq(q) = r.n_f23 + r.n_f24;
end
B = reshape(Bq, nc, na);  Af = reshape(Aq, nc, na);
S = reshape(Sq, nc, na);  V = reshape(Vq, nc, na);
Pv = 1 - mean(B, 2);
Sg = mean(Af, 2);
Fv = sum(V, 2) ./ max(sum(S, 2), 1);
J  = (1 - Pv) + 0.25 * (1 - Sg / (p.motor.v0 * p.obst.T_run)) + 0.1 * Fv;
end
