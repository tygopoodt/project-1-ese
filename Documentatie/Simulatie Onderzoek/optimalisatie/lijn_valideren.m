function V = lijn_valideren(p, banen, n_trek, seed0)
%LIJN_VALIDEREN  Eén vaste instelling op banen die de optimizer nooit zag.
%
%   V = lijn_valideren(p, banen, n_trek, seed0)
%   Elke baan wordt n_trek keer gereden, elke keer met een nieuwe trekking
%   van de afwijkingen (variatie_trekken). Uitvoer: matrices nb x n_trek met
%   rondetijd T [s], genormaliseerde rondetijd Tn, e_max [m], voltooid,
%   fractie lijn kwijt, gejitter en kost J.

nb = numel(banen);  n = nb * n_trek;
vars = cell(n, 1);
for q = 1:n
    vars{q} = variatie_trekken(p, RandStream('mt19937ar', 'Seed', seed0 + q));
end
[T, Tn, Em, Ok, Kw, Jt, J] = deal(zeros(n, 1));
parfor q = 1:n
    b = mod(q - 1, nb) + 1;
    r = sim_lijnvolgen(p, banen{b}, vars{q});
    T(q)  = r.T;
    Tn(q) = r.T * p.motor.v0 / banen{b}.L;
    Em(q) = r.e_max;
    Ok(q) = r.voltooid;
    Kw(q) = r.kwijt_frac;
    Jt(q) = r.jitter;
    J(q)  = lijn_kost(r, banen{b}, p);
end
V.T  = reshape(T,  nb, n_trek);
V.Tn = reshape(Tn, nb, n_trek);
V.Em = reshape(Em, nb, n_trek);
V.Ok = reshape(Ok, nb, n_trek) > 0;
V.Kw = reshape(Kw, nb, n_trek);
V.Jt = reshape(Jt, nb, n_trek);
V.J  = reshape(J,  nb, n_trek);
end
