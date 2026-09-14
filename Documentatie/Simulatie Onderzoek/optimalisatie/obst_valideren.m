function R = obst_valideren(p, seeds, var_seed0, soort)
%OBST_VALIDEREN  Eén instelling, één testrit van T_run seconden per arena.
%
%   R = obst_valideren(p, seeds, var_seed0)
%   R = obst_valideren(p, seeds, var_seed0, 'rustig')
%   Per arena-seed een eigen trekking van de afwijkingen. Uitvoer (kolommen,
%   één rij per rit): botsing, afstand [m], stops, F2.3-fouten, F2.4-fouten,
%   tijdstip botsing [s], soort botsing (1 muur, 2 kist, 3 paal).

if nargin < 4, soort = 'rommel'; end
n = numel(seeds);
[bots, af, nst, f23, f24, tb, srt] = deal(zeros(n, 1));
parfor i = 1:n
    w = arena_cache(seeds(i), soort);
    v = variatie_trekken(p, RandStream('mt19937ar', 'Seed', var_seed0 + seeds(i)));
    r = sim_obstakels(p, w, v);
    bots(i) = r.botsing;  af(i) = r.afstand;  nst(i) = r.n_stops;
    f23(i) = r.n_f23;  f24(i) = r.n_f24;  tb(i) = r.t_botsing;  srt(i) = r.b_soort;
end
R.botsing = bots > 0;  R.afstand = af;  R.stops = nst;
R.f23 = f23;  R.f24 = f24;  R.t_botsing = tb;  R.soort = srt;
R.p_vrij = mean(~R.botsing);
end
