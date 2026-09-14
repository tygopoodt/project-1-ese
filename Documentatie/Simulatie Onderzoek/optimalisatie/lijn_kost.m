function J = lijn_kost(r, baan, p)
%LIJN_KOST  Kostfunctie van één lijnvolg-run (lager = beter).
%
%   Voltooid:
%       J = T v0 / L                                   genormaliseerde rondetijd
%         + lambda_e * max(0, e_max / e_toel - 1)      overschrijding F3.1 (met marge)
%         + lambda_k * f_kwijt                         tijdfractie lijn kwijt
%         + lambda_j * j                               PWM-gejitter per stap
%   T v0 / L = 1 betekent: de hele ronde op volle motorsnelheid.
%   Niet voltooid (van de lijn of omgekeerd):
%       J = 10 + 10 (1 - voortgang)
%   zodat elke voltooide run beter scoort dan elke mislukte.

if ~r.voltooid
    J = 10 + 10 * (1 - max(r.voortgang, 0));
else
    J = r.T * p.motor.v0 / baan.L ...
        + p.doel.lambda_e * max(0, r.e_max / p.doel.e_toel - 1) ...
        + p.doel.lambda_k * r.kwijt_frac ...
        + p.doel.lambda_j * r.jitter;
end
end
