function res = sim_obstakels(p, w, var, bewaar_log)
%SIM_OBSTAKELS  Eén testrit autonoom rijden (F2) in een arena, zonder graphics.
%
%   res = sim_obstakels(p, w, var)
%   res = sim_obstakels(p, w, var, true)    ook tijdreeksen in res.log
%
%   Toestandsmachine (F2.1 - F2.4):
%     RIJDEN     rechtdoor; tussen d_slow en d_stop lineair afremmen naar v_kruip;
%                optioneel zijwaarts ontwijken met schuine sensoren (k_zij) en,
%                met encoders, koershouden (K_koers)
%     REMMEN     rijstrook vrij < d_stop: kortsluitrem, tegenstroom of uitrollen
%     KIEZEN     'draai': op de plaats draaien tot er >= d_vrij vrij is, daarna
%                         nog phi_extra verder (niet langs een muur blijven hangen)
%                'scan' : phi_scan naar links en rechts kijken, dan naar het
%                         MIDDEN van de breedste vrije sector draaien
%     ACHTERUIT  levert draaien niets op: s_achter terug, daarna opnieuw KIEZEN
%
%   Sonar. Elke HC-SR04 (opstelling: sonar_opstelling) vuurt om de beurt, om de
%   T_meas; een meting is pas t_echo = 2d/c na de puls binnen en gaat door een
%   mediaanfilter over N_f metingen. Geen echo (time-out) betekent: niets
%   binnen bereik, OF een vlak dat zo schuin staat dat de echo wegkaatst. Met
%   echo_nodig = true telt een time-out tijdens KIEZEN daarom niet als vrij.
%
%   Rijstrook-logica. Een echo op afstand d kan overal op de boog
%   [richting - beta_c, richting + beta_c] vandaan komen. Hij telt als
%   'in de weg' als een punt van die boog binnen de rijstrook |y| <= B/2 + marge
%   ligt; de vrije afstand is dan de kleinste x van die punten, gemeten vanaf
%   het sensorfront.
%
%   Koers. Zonder encoders kent de regelaar zijn koers alleen door de gevraagde
%   draaisnelheid te integreren; de onbekende dode zone maakt dat onnauwkeurig.
%   Met encoders (p.obst.encoders) telt hij pulsen van ds = 2 pi r_w / N:
%       theta_hat = (n_R - n_L) ds / b
%
%   Per stop wordt gecontroleerd (F2.3): remvertraging na 'obstakel < 20 cm'
%   <= 0,5 s, stilstand binnen 1,5 s, speling >= 2 cm; en (F2.4): binnen 4 s
%   weer op weg.

if nargin < 4, bewaar_log = false; end
o  = p.obst;
so = p.sonar;
so.beta = var.sonar.beta;  so.gamma_max = var.sonar.gamma_max;  so.p_uitval = var.sonar.p_uitval;
dt = 1 / o.f_s;
nstap = round(o.T_run / dt);
rs = RandStream('mt19937ar', 'Seed', var.seed);
v0 = p.motor.v0;  bnom = p.auto.b;  tau_n = p.motor.tau;

mnt  = sonar_opstelling(o.opstelling, o.toe, p);
ns   = size(mnt, 1);
buf  = repmat(so.d_max, ns, o.N_f);
d_f  = so.d_max * ones(ns, 1);
links  = mnt(:, 3) > 1e-6;
rechts = mnt(:, 3) < -1e-6;
boog   = linspace(-o.beta_c, o.beta_c, 5);
halfB  = p.auto.breedte / 2 + o.marge;

% omtrek van de auto, om de 1 cm, in het autoassenstelsel
A  = p.auto;
ex = linspace(-A.achter, A.voor, ceil((A.voor + A.achter) / 0.01) + 1);
ey = linspace(-A.breedte / 2, A.breedte / 2, ceil(A.breedte / 0.01) + 1);
fx = [ex, ex, -A.achter * ones(size(ey)), A.voor * ones(size(ey))];
fy = [-A.breedte/2 * ones(size(ex)), A.breedte/2 * ones(size(ex)), ey, ey];
voorkant = fx >= A.voor - 1e-9;
[nR, nC] = size(w.sdf);

RIJDEN = 1; REMMEN = 2; KIEZEN = 3; ACHTERUIT = 4;
toestand = RIJDEN;  vorige = 0;
st = [w.start 0 0];
th_hat = 0;  th_ref = 0;  s_enc = [0 0];
t_volg = 0;  k_meet = 0;  pend_t = inf;  pend_i = 1;  pend_d = inf;  pend_th = 0;
richting = 1;  scan_fase = 0;  scan_th = [];  scan_d = [];  th_doel = 0;
extra = false;  th_extra0 = 0;
t_detect = NaN;  t_rem = NaN;  t_stil = NaN;  t_k0 = NaN;  th_k0 = 0;  t_a0 = NaN;
t_wacht = 0;  t_tegen = 0;  v_rem0 = 0;  v_cmd = 0;  lat_k = 0;  stop_open = false;
lat = [];  tstop = [];  spel = [];  tkies = [];
afstand = 0;  n_rijden = 0;  n_achter = 0;  botsing = false;  t_botsing = NaN;
b_punt = [NaN NaN];  b_toestand = 0;  b_soort = 0;  b_hoek = NaN;
min_speling = inf;
nieuw_d = inf;  nieuw_th = 0;

if bewaar_log
    lg.t = zeros(nstap, 1); lg.x = lg.t; lg.y = lg.t; lg.th = lg.t; lg.v = lg.t;
    lg.toestand = lg.t; lg.d_voor = lg.t; lg.d_raw = nan(nstap, ns);
end

for k = 1:nstap
    t = (k - 1) * dt;
    c = cos(st(3));  s = sin(st(3));

    % --- sonar: resultaat binnen? --------------------------------------------
    nieuw = false;
    if t >= pend_t
        dm = pend_d;
        if ~isfinite(dm)
            if toestand == KIEZEN && o.echo_nodig, dm = 0; else, dm = so.d_max; end
        end
        dm = min(dm, so.d_max);
        buf(pend_i, :) = [buf(pend_i, 2:end), dm];
        d_f(pend_i) = median(buf(pend_i, :));
        nieuw = true;  nieuw_d = dm;  nieuw_th = pend_th;
        if bewaar_log, lg.d_raw(k, pend_i) = pend_d; end
        pend_t = inf;
    end
    % --- sonar: nieuwe puls ---------------------------------------------------
    if t >= t_volg && isinf(pend_t)
        i = mod(k_meet, ns) + 1;  k_meet = k_meet + 1;
        pos = [st(1) + c * mnt(i, 1) - s * mnt(i, 2), st(2) + s * mnt(i, 1) + c * mnt(i, 2)];
        dm = sonar_meten(pos, st(3) + mnt(i, 3), w, so, rs);
        if isfinite(dm), te = 2 * dm / so.c; else, te = so.t_timeout; end
        pend_i = i;  pend_t = t + te;  pend_d = dm;  pend_th = th_hat + mnt(i, 3);
        t_volg = max(t_volg + o.T_meas, t);
    end

    % --- vrije afstand in de rijstrook -----------------------------------------
    d_voor = inf;
    for i = 1:ns
        if d_f(i) < so.d_max
            px = mnt(i, 1) + d_f(i) * cos(mnt(i, 3) + boog);
            py = mnt(i, 2) + d_f(i) * sin(mnt(i, 3) + boog);
            in = abs(py) <= halfB & px > 0;
            if any(in), d_voor = min(d_voor, min(px(in)) - p.sonar.x); end
        end
    end

    % --- werkelijke afstand recht vooruit, voor de controle van F2.3 --------
    if toestand == RIJDEN && isnan(t_detect)
        if straal(st(1) + c * p.sonar.x, st(2) + s * p.sonar.x, st(3), w) < 0.20
            t_detect = t;
        end
    end

    % --- overgangen -------------------------------------------------------------
    switch toestand
        case RIJDEN
            if d_voor < o.d_stop
                toestand = REMMEN;  t_rem = t;  stop_open = true;  t_stil = NaN;
                if isnan(t_detect), lat_k = 0; else, lat_k = t - t_detect; end
                v_rem0 = v_cmd;
                t_tegen = tau_n * log(1 + v_rem0 / v0);
                switch o.rem
                    case 'kortsluit',   t_wacht = 5 * tau_n;
                    case 'tegenstroom', t_wacht = t_tegen + 0.10;
                    otherwise,          t_wacht = v_rem0 / p.motor.a_rol + 0.10;
                end
            end
        case REMMEN
            if t - t_rem >= t_wacht
                if any(links) && any(rechts)
                    if min(d_f(links)) >= min(d_f(rechts)), richting = 1; else, richting = -1; end
                else
                    richting = 2 * (rs.rand() < 0.5) - 1;
                end
                [toestand, t_k0, th_k0, scan_fase, scan_th, scan_d, buf, d_f] = ...
                    start_kiezen(KIEZEN, t, th_hat, buf, d_f);
                extra = false;
            end
        case KIEZEN
            gedraaid = th_hat - th_k0;
            if strcmp(o.strategie, 'scan')
                if nieuw && scan_fase < 3
                    scan_th(end + 1) = nieuw_th;  scan_d(end + 1) = nieuw_d; %#ok<AGROW>
                end
                if scan_fase == 1 && gedraaid >= o.phi_scan
                    scan_fase = 2;
                elseif scan_fase == 2 && gedraaid <= -o.phi_scan
                    th_doel = midden_vrije_sector(scan_th, scan_d, o.d_vrij, o.scan_min);
                    if isnan(th_doel)
                        toestand = ACHTERUIT;  t_a0 = t;
                    else
                        scan_fase = 3;
                    end
                elseif scan_fase == 3 && th_hat >= th_doel - deg2rad(2)
                    buf(:) = o.d_vrij;  d_f(:) = o.d_vrij;
                    toestand = RIJDEN;  t_detect = NaN;
                end
            else
                if nieuw && d_voor >= o.d_vrij && abs(gedraaid) >= o.phi_min
                    if o.phi_extra <= 0
                        toestand = RIJDEN;  t_detect = NaN;
                    elseif ~extra
                        extra = true;  th_extra0 = th_hat;
                    elseif abs(th_hat - th_extra0) >= o.phi_extra
                        toestand = RIJDEN;  t_detect = NaN;
                    end
                elseif nieuw && d_voor < o.d_vrij
                    extra = false;
                end
                if toestand == KIEZEN && (abs(gedraaid) >= 2 * pi || t - t_k0 > o.t_kies_max)
                    toestand = ACHTERUIT;  t_a0 = t;
                end
            end
            if toestand == RIJDEN && stop_open
                if isnan(t_stil), t0 = t_rem; else, t0 = t_stil; end
                tkies(end + 1) = t - t0; %#ok<AGROW>
                stop_open = false;
            end
        case ACHTERUIT
            if t - t_a0 >= o.s_achter / o.v_achter
                n_achter = n_achter + 1;
                richting = -richting;
                [toestand, t_k0, th_k0, scan_fase, scan_th, scan_d, buf, d_f] = ...
                    start_kiezen(KIEZEN, t, th_hat, buf, d_f);
                extra = false;
            end
    end
    if toestand == RIJDEN && vorige ~= RIJDEN, th_ref = th_hat; end
    vorige = toestand;

    % --- commando's --------------------------------------------------------------
    vrij = [false false];
    vLc = 0;  vRc = 0;  u = [];
    switch toestand
        case RIJDEN
            if d_voor >= o.d_slow
                v_cmd = o.v_cruise;
            else
                f = (d_voor - o.d_stop) / max(o.d_slow - o.d_stop, 1e-6);
                v_cmd = o.v_kruip + (o.v_cruise - o.v_kruip) * min(max(f, 0), 1);
            end
            % zijwaarts ontwijken met de schuine sensoren (Braitenberg):
            %   omega = k_zij (f_R - f_L),   f = max(0, 1 - d / d_zij)
            w_zij = 0;
            if o.k_zij > 0 && any(links) && any(rechts)
                fL = max(0, 1 - min(d_f(links))  / o.d_zij);
                fR = max(0, 1 - min(d_f(rechts)) / o.d_zij);
                w_zij = o.k_zij * (fR - fL);
            end
            % koershouden met encoders:  omega += K_koers (theta_ref - theta_hat)
            w_koers = 0;
            if o.encoders
                th_ref  = th_ref + w_zij * dt;
                w_koers = o.K_koers * (th_ref - th_hat);
            end
            wt = w_zij + w_koers;
            vLc = v_cmd - wt * bnom / 2;  vRc = v_cmd + wt * bnom / 2;
        case REMMEN
            switch o.rem
                case 'kortsluit'
                    u = [0 0];
                case 'tegenstroom'
                    if t - t_rem < t_tegen, u = -sign(v_rem0) * [1 1]; else, u = [0 0]; end
                otherwise
                    u = [0 0];  vrij = [true true];
            end
        case KIEZEN
            if strcmp(o.strategie, 'scan') && scan_fase == 2, rd = -1;
            elseif strcmp(o.strategie, 'scan'), rd = 1;
            else, rd = richting;
            end
            vRc = rd * o.w_draai * bnom / 2;  vLc = -vRc;
        case ACHTERUIT
            vLc = -o.v_achter;  vRc = -o.v_achter;
    end
    if isempty(u)
        u = pwm_omzetten([vLc vRc] / v0, p.motor.u_dz, p.motor.pwm_bits);
    end
    if ~o.encoders
        th_hat = th_hat + (vRc - vLc) / bnom * dt;
    end

    st = aandrijving_stap(st, u, vrij, var, p, dt);
    v_echt = (st(4) + st(5)) / 2;
    if o.encoders
        s_enc  = s_enc + st(4:5) * dt;
        q      = fix(s_enc / p.enc.ds) * p.enc.ds;
        th_hat = (q(2) - q(1)) / bnom;
    end
    if bewaar_log        % vóór de botsingscontrole, zodat ook de laatste stap in het log staat
        lg.t(k) = t;  lg.x(k) = st(1);  lg.y(k) = st(2);  lg.th(k) = st(3);
        lg.v(k) = v_echt;  lg.toestand(k) = toestand;  lg.d_voor(k) = d_voor;
    end

    % --- botsingscontrole met de afstandskaart ---------------------------------
    c = cos(st(3));  s = sin(st(3));
    wx = st(1) + c * fx - s * fy;
    wy = st(2) + s * fx + c * fy;
    jx = min(max(floor((wx - w.x0) / w.res) + 1, 1), nC);
    iy = min(max(floor((wy - w.y0) / w.res) + 1, 1), nR);
    dd = double(w.sdf(iy + (jx - 1) * nR)) - w.res / 2;
    speling = min(dd);
    min_speling = min(min_speling, speling);
    if speling <= 0
        botsing = true;  t_botsing = t;
        [~, ib] = min(dd);
        b_punt = [fx(ib) fy(ib)];  b_toestand = toestand;
        [b_soort, b_hoek] = botsing_soort(wx(ib), wy(ib), st(3), w);
        break
    end

    if stop_open && isnan(t_stil) && toestand >= REMMEN && abs(v_echt) < 0.003
        t_stil = t;
        if isnan(t_detect), t_ref = t_rem; else, t_ref = t_detect; end
        lat(end + 1)   = lat_k; %#ok<AGROW>
        tstop(end + 1) = t - t_ref; %#ok<AGROW>
        spel(end + 1)  = min(dd(voorkant)); %#ok<AGROW>
    end
    if toestand == RIJDEN
        afstand = afstand + max(v_echt, 0) * dt;
        n_rijden = n_rijden + 1;
    end
end

% een stop die nog open staat en al langer dan 4 s duurt, telt als F2.4-fout
if stop_open && ~botsing
    if isnan(t_stil), t0 = t_rem; else, t0 = t_stil; end
    if t - t0 > 4, tkies(end + 1) = t - t0; end
end

res.botsing     = botsing;
res.t_botsing   = t_botsing;
res.b_punt      = b_punt;       % geraakt punt in autoassenstelsel [m]
res.b_toestand  = b_toestand;   % 1 rijden, 2 remmen, 3 kiezen, 4 achteruit
res.b_soort     = b_soort;      % 1 muur, 2 kist, 3 paal
res.b_hoek      = b_hoek;       % bij een muur: hoek koers - muurnormaal [rad]
res.t_eind      = t;
res.afstand     = afstand;
res.v_gem       = afstand / o.T_run;
res.n_stops     = numel(lat);
res.lat         = lat;
res.t_stop      = tstop;
res.speling     = spel;
res.t_kies      = tkies;
res.n_f23       = sum(lat > 0.5 | tstop > 1.5 | spel < 0.02);
res.n_f24       = sum(tkies > 4);
res.n_achter    = n_achter;
res.min_speling = min_speling;
res.frac_rijden = n_rijden / k;
if bewaar_log
    f = fieldnames(lg);
    for i = 1:numel(f), lg.(f{i}) = lg.(f{i})(1:k, :); end
    res.log = lg;
end
end

% -------------------------------------------------------------------------
function [toestand, t_k0, th_k0, fase, sth, sd, buf, d_f] = start_kiezen(KIEZEN, t, th_hat, buf, d_f)
% Oude metingen gelden niet meer zodra de auto draait: vul het filter met 0
% (= geblokkeerd), zodat pas N_f verse metingen 'vrij' kunnen melden.
toestand = KIEZEN;  t_k0 = t;  th_k0 = th_hat;
fase = 1;  sth = [];  sd = [];
buf(:) = 0;  d_f(:) = 0;
end

function th = midden_vrije_sector(ths, ds, d_vrij, w_min)
% Midden van de breedste aaneengesloten reeks metingen met d >= d_vrij.
% Door naar het midden te sturen in plaats van naar de langste echo, rijdt de
% auto niet evenwijdig langs de rand van een vrije sector (vaak een muur).
th = NaN;
if isempty(ths), return; end
[ths, o] = sort(ths);  vrij = ds(o) >= d_vrij;
beste = -inf;  i = 1;  n = numel(ths);
while i <= n
    if vrij(i)
        j = i;
        while j < n && vrij(j + 1), j = j + 1; end
        breedte = ths(j) - ths(i);
        if breedte > beste, beste = breedte; th = (ths(i) + ths(j)) / 2; end
        i = j + 1;
    else
        i = i + 1;
    end
end
if beste < w_min, th = NaN; end
end

function d = straal(x0, y0, th, w)
% Afstand langs één straal tot het eerste obstakel (zonder ruis of invalshoek).
ux = cos(th);  uy = sin(th);  d = inf;
S  = w.seg;
ex = S(:, 3) - S(:, 1);  ey = S(:, 4) - S(:, 2);
wx = S(:, 1) - x0;       wy = S(:, 2) - y0;
den = ux * ey - uy * ex;
t = (wx .* ey - wy .* ex) ./ den;
v = (wx * uy - wy * ux) ./ den;
t(~(t > 0 & v >= 0 & v <= 1)) = inf;
d = min(d, min(t));
if ~isempty(w.cirkel)
    C  = w.cirkel;
    cx = C(:, 1) - x0;  cy = C(:, 2) - y0;
    tp = ux * cx + uy * cy;
    q  = C(:, 3).^2 - (cx.^2 + cy.^2 - tp.^2);
    t  = tp - sqrt(max(q, 0));
    t(~(q > 0 & t > 0)) = inf;
    d = min(d, min(t));
end
end

function [soort, hoek] = botsing_soort(x, y, th, w)
% Wat is er geraakt? Bij een muur ook de hoek tussen koers en muurnormaal
% (0 = recht op de muur af, 90 graden = evenwijdig).
hoek = NaN;
[dm, im] = min([x, w.W - x, y, w.H - y]);
if dm < 0.012
    soort = 1;
    normaal = [1 0; -1 0; 0 1; 0 -1];
    hoek = acos(min(1, abs(-normaal(im, :) * [cos(th); sin(th)])));
elseif ~isempty(w.cirkel) && any(hypot(w.cirkel(:, 1) - x, w.cirkel(:, 2) - y) - w.cirkel(:, 3) < 0.012)
    soort = 3;
else
    soort = 2;
end
end
