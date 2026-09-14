function A = obst_afstellen(p, opstelling, seeds, var_seed0, n_rand, n_lok, th0)
%OBST_AFSTELLEN  Beste F2-instellingen voor één sonaropstelling.
%
%   A = obst_afstellen(p, opstelling, seeds, var_seed0, n_rand, n_lok, th0)
%   Random search in de genormaliseerde ruimte [0,1]^13 (obst_zet), gevolgd
%   door twee rondes lokale verfijning rond de beste drie (stapgrootte 0,08
%   en 0,04). th0 (optioneel, 1 x 13) wordt als eerste kandidaat meegenomen.

D  = 13;
rs = RandStream('mt19937ar', 'Seed', 11);
C = rs.rand(n_rand, D);
if nargin >= 7 && ~isempty(th0), C(1, :) = th0; end
[J, Pv, Sg, Fv] = obst_evalueren(p, C, opstelling, seeds, var_seed0);
for stap = [0.08 0.04]
    [~, o] = sort(J);
    top = C(o(1:min(3, end)), :);
    C2 = zeros(size(top, 1) * n_lok, D);
    for i = 1:size(top, 1)
        C2((i - 1) * n_lok + (1:n_lok), :) = top(i, :) + stap * rs.randn(n_lok, D);
    end
    C2 = min(max(C2, 0), 1);
    [J2, Pv2, Sg2, Fv2] = obst_evalueren(p, C2, opstelling, seeds, var_seed0);
    C = [C; C2];  J = [J; J2];  Pv = [Pv; Pv2];  Sg = [Sg; Sg2];  Fv = [Fv; Fv2]; %#ok<AGROW>
end
[A.J, ib] = min(J);
A.theta = C(ib, :);
A.p_vrij = Pv(ib);
A.afstand = Sg(ib);
A.f_eis = Fv(ib);
A.p = obst_zet(p, A.theta, opstelling);
A.alle = struct('C', C, 'J', J, 'Pv', Pv, 'Sg', Sg, 'Fv', Fv);
end
