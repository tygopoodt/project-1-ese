function A = lijn_afstellen(p, banen, vars, n_rand, n_lok, th0)
%LIJN_AFSTELLEN  Beste regelaar voor één sensoropstelling: random search + verfijning.
%
%   A = lijn_afstellen(p, banen, vars, n_rand, n_lok, th0)
%   1. n_rand kandidaten uniform in het zoekgebied (Kp, Kd, tau_d logaritmisch),
%      plus th0: de analytische schatting Kp = 2 v / L^2 (zie het rapport).
%   2. Twee rondes lokale verfijning: rond de beste drie telkens n_lok nieuwe
%      kandidaten, Gaussisch verstoord met 6 % en daarna 2,5 % van het zoekgebied.
%   Random search is bij vijf parameters verrassend effectief en laat zich
%   perfect paralleliseren (Hansen 2016 noemt CMA-ES pas zinvol bij meer
%   dimensies of een lastiger landschap; zie LEESLIJST D1).
%
%   Uitvoer A: theta (beste kandidaat), J, Tn, e_max, ok, en A.alle met alle
%   geëvalueerde kandidaten.

lo = [log10(10)   log10(0.01) log10(0.003) 0.15 0];
hi = [log10(1500) log10(20)   log10(0.1)   0.47 1];
rs = RandStream('mt19937ar', 'Seed', 7);

C = lo + (hi - lo) .* rs.rand(n_rand, 5);
if nargin >= 6 && ~isempty(th0), C(1, :) = min(max(th0, lo), hi); end
[J, Tn, Em, Ok] = lijn_evalueren(p, C, banen, vars);

for stap = [0.06 0.025]
    [~, o] = sort(J);
    top = C(o(1:min(3, end)), :);
    C2 = zeros(size(top, 1) * n_lok, 5);
    for i = 1:size(top, 1)
        C2((i - 1) * n_lok + (1:n_lok), :) = top(i, :) + stap * (hi - lo) .* rs.randn(n_lok, 5);
    end
    C2 = min(max(C2, lo), hi);
    [J2, Tn2, Em2, Ok2] = lijn_evalueren(p, C2, banen, vars);
    C = [C; C2];  J = [J; J2];  Tn = [Tn; Tn2];  Em = [Em; Em2];  Ok = [Ok; Ok2]; %#ok<AGROW>
end

[A.J, ib] = min(J);
A.theta = C(ib, :);
A.Tn    = Tn(ib);
A.e_max = Em(ib);
A.ok    = Ok(ib);
A.alle  = struct('C', C, 'J', J, 'Tn', Tn, 'Em', Em, 'Ok', Ok);
end
