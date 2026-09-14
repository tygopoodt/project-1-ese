function [d, d_echt] = sonar_meten(pos, richting, wereld, s, rs)
%SONAR_METEN  Eén meting van een HC-SR04, als bundel van stralen in 2D.
%
%   [d, d_echt] = sonar_meten(pos, richting, wereld, s, rs)
%       pos       [x y] van de sensor in de wereld [m]
%       richting  kijkrichting van de sensor [rad]
%       wereld    struct met .seg (M x 4, [x1 y1 x2 y2]: muren en kistzijden)
%                 en .cirkel (K x 3, [xc yc r]: poten, flessen, blikken)
%       s         sonarparameters (zie qdat_parameters, veld p.sonar)
%       rs        RandStream voor ruis (reproduceerbaar per run)
%   Uitvoer:
%       d       gemeten afstand [m]; Inf = geen echo binnen de time-out
%       d_echt  afstand zonder ruis/uitval (voor controle in de analyse)
%
%   Model. De HC-SR04 geeft de afstand van de EERSTE echo die terugkomt:
%       d = c * t_echo / 2,   c = 331,3 + 0,606 * T  [m/s]   (T in graden C)
%   De bundel wordt benaderd door n stralen verdeeld over [-beta, +beta].
%   Een straal telt alleen mee als het oppervlak hem terugkaatst: de hoek
%   tussen straal en oppervlaknormaal (invalshoek) moet kleiner zijn dan
%   gamma_max. Bij een schuine muur of de flank van een ronde poot gaat de
%   echo anders weg (spiegelende reflectie) en ziet de sensor niets.
%       d_echt = min over geldige stralen van de trefafstand
%   Daarna ruis, uitval en afronding zoals de Arduino-code die ook heeft:
%       d = round((d_echt + sigma * N(0,1)) / q) * q
%   met kans p_uitval op een gemiste echo (d = Inf) en een dode zone onder d_min.

n   = max(5, ceil(2 * s.beta / deg2rad(1)) + 1);   % ~1 graad tussen stralen
phi = richting + linspace(-s.beta, s.beta, n)';     % n x 1
ux  = cos(phi);  uy = sin(phi);
cos_gmax = cos(s.gamma_max);
best = inf(n, 1);

% --- lijnstukken: p + t u = a + v (b - a) -------------------------------
if ~isempty(wereld.seg)
    S  = wereld.seg;
    ex = (S(:, 3) - S(:, 1))';  ey = (S(:, 4) - S(:, 2))';     % 1 x M
    wx = (S(:, 1) - pos(1))';   wy = (S(:, 2) - pos(2))';
    den = ux .* ey - uy .* ex;                                % n x M
    t   = (wx .* ey - wy .* ex) ./ den;
    v   = (wx .* uy - wy .* ux) ./ den;
    Ls  = hypot(ex, ey);
    cosi = abs(ux .* (-ey) + uy .* ex) ./ Ls;                 % |u . normaal|
    ok  = t > 0 & v >= 0 & v <= 1 & abs(den) > 1e-12 & cosi >= cos_gmax;
    t(~ok) = Inf;
    best = min(best, min(t, [], 2));
end

% --- cirkels -------------------------------------------------------------
if isfield(wereld, 'cirkel_sonar'), C = wereld.cirkel_sonar; else, C = wereld.cirkel; end
if ~isempty(C)
    cx = (C(:, 1) - pos(1))';  cy = (C(:, 2) - pos(2))';  r = C(:, 3)';
    tp = ux .* cx + uy .* cy;                                 % projectie
    h2 = (cx.^2 + cy.^2) - tp.^2;                             % loodafstand^2
    q  = r.^2 - h2;
    t  = tp - sqrt(max(q, 0));
    hx = ux .* t - cx;  hy = uy .* t - cy;                    % trefpunt - middelpunt
    cosi = abs(ux .* hx + uy .* hy) ./ r;
    ok = q > 0 & t > 0 & cosi >= cos_gmax;
    t(~ok) = Inf;
    best = min(best, min(t, [], 2));
end

d_echt = min(best);
if d_echt > s.d_max, d_echt = Inf; end

% --- ruis, uitval, dode zone, afronding --------------------------------
d = d_echt;
if rs.rand() < s.p_uitval
    d = Inf;
elseif isfinite(d)
    d = d + s.sigma * rs.randn();
    d = max(d, s.d_min);
    if s.q > 0, d = round(d / s.q) * s.q; end
end
end
