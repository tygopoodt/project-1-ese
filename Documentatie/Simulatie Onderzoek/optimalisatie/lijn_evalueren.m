function [J, Tn, Em, Ok] = lijn_evalueren(p, C, banen, vars)
%LIJN_EVALUEREN  Kost van elke kandidaat-regelaar over alle trainingsbanen.
%
%   [J, Tn, Em, Ok] = lijn_evalueren(p, C, banen, vars)
%       C     nc x 5 kandidaten (zie lijn_regelaar)
%       banen cell met banen, vars cell met één variatie per baan
%   Uitvoer per kandidaat: gemiddelde kost J, gemiddelde genormaliseerde
%   rondetijd Tn, grootste afwijking Em [m], en of alle banen voltooid zijn.
%   Elke kandidaat ziet exact dezelfde banen en afwijkingen ('common random
%   numbers'), zodat verschillen in J door de regelaar komen en niet door toeval.

nc = size(C, 1);  ns = numel(banen);
[Jq, Tq, Eq, Oq] = deal(zeros(nc * ns, 1));
parfor q = 1:nc * ns
    [i, s] = ind2sub([nc ns], q);
    pp = lijn_regelaar(p, C(i, :));
    r  = sim_lijnvolgen(pp, banen{s}, vars{s});
    Jq(q) = lijn_kost(r, banen{s}, pp);
    Tq(q) = r.T * pp.motor.v0 / banen{s}.L;
    Eq(q) = r.e_max;
    Oq(q) = r.voltooid;
end
J  = mean(reshape(Jq, nc, ns), 2);
Tn = mean(reshape(Tq, nc, ns), 2);
Em = max(reshape(Eq, nc, ns), [], 2);
Ok = all(reshape(Oq, nc, ns), 2);
end
