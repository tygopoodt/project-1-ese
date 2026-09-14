%% ONDERZOEK_OBSTAKELS  Simulatie 2: HC-SR04-opstelling en gedrag voor autonoom rijden (F2)
%
%  Onderzoeksvragen
%   O1. Hoe hard mag de auto rijden bij een gegeven meetfrequentie, filter en
%       remmethode, zodat hij volgens F2.3 op tijd stilstaat? (analytisch)
%   O2. Welk deel van de breedte van de auto 'ziet' de sonar, en welke
%       opstelling en uitdraaihoek van twee of drie HC-SR04's dekt de auto het
%       best? (analytisch)
%   O3. Welke opstelling + gedragsparameters geven de grootste kans op een
%       botsingsvrije rit van 2 minuten (F2), en hoeveel helpen encoders?
%       (simulatie + optimalisatie)
%   O4. Hoe gevoelig is die kans voor de grootheid die we het slechtst kennen:
%       tot welke invalshoek een vlak nog een echo geeft?
%
%  Methode
%   Analytische formules voor O1/O2; voor O3 per opstelling een random search
%   + lokale verfijning over 13 gedragsparameters (obst_zet) in 24 willekeurige
%   arena's, daarna validatie van het optimum in 100 nieuwe arena's van twee
%   soorten ('rommel': kisten onder willekeurige hoek en dunne poten;
%   'rustig': alleen kisten van >= 15 cm). Aanname: de motoren zijn op 1 %
%   na gekalibreerd (eis F2.1, zie rapport).
%
%  Uitvoer (map resultaten/): figuren obst_*.pdf/.png, resultaten_obst.tex,
%  tabel_obst_*.tex, obst_optimum.mat (voor demo_obstakels.m).
%  Duur: ~15 min op 8 workers. Proefrun: setenv('QDAT_SNEL','1')

clear; clc; close all;
SNEL = ~isempty(getenv('QDAT_SNEL'));
root  = fileparts(mfilename('fullpath'));
paden = {root, fullfile(root, 'model'), fullfile(root, 'optimalisatie')};
addpath(paden{:});
uit = fullfile(root, 'resultaten');
if ~exist(uit, 'dir'), mkdir(uit); end
if isempty(gcp('nocreate'))
    cl = parcluster('Processes');                 % profiel zelf blijft ongewijzigd
    cl.NumWorkers = min(8, feature('numcores'));
    parpool(cl, cl.NumWorkers);
end
pctRunOnAll(sprintf('addpath(''%s'', ''%s'', ''%s'');', paden{:}));
t_start = tic;

p = qdat_parameters();
p.var.delta_motor = 0.01;
mac = containers.Map();
if SNEL
    n_rand = 12;  n_lok = 3;  s_train = 1:8;  s_val = 1001:1020;  s_rust = 2001:2020;
else
    n_rand = 40;  n_lok = 5;  s_train = 1:16; s_val = 1001:1100;  s_rust = 2001:2100;
end

%% 1. Analytisch: remweg, maximale snelheid, drift, draaifout, zwaaistraal
v0 = p.motor.v0;  tau = p.motor.tau;  cs = p.sonar.c;  B = p.auto.breedte;
s_rem = @(v, soort) ...
    (strcmp(soort, 'kortsluit'))   .* (v * tau) + ...
    (strcmp(soort, 'tegenstroom')) .* (v * tau - v0 * tau * log(1 + v / v0)) + ...
    (strcmp(soort, 'uitrollen'))   .* (v.^2 / (2 * p.motor.a_rol));
t_lat = @(n, T, Nf) n * T * (1 + (Nf - 1) / 2) + 2 * 0.20 / cs + 1 / p.obst.f_s;
d_beschik = 0.20 - 0.02;               % F2.3: remmen vanaf 20 cm, 2 cm speling over
vmax = @(n, T, Nf, soort) vmax_rem(@(v) v * t_lat(n, T, Nf) + s_rem(v, soort) - d_beschik, v0);

mac('ObstSremKort')  = fmt(100 * s_rem(0.30, 'kortsluit'), 1);
mac('ObstSremTegen') = fmt(100 * s_rem(0.30, 'tegenstroom'), 1);
mac('ObstSremRol')   = fmt(100 * s_rem(0.30, 'uitrollen'), 1);
mac('ObstTlatEen')   = fmt(1000 * t_lat(1, 0.06, 3), 0);
mac('ObstTlatDrie')  = fmt(1000 * t_lat(3, 0.0667, 3), 0);
mac('ObstVmaxEen')   = fmt(vmax(1, 0.06, 3, 'kortsluit'), 2);
mac('ObstVmaxDrie')  = fmt(vmax(3, 0.0667, 3, 'kortsluit'), 2);
mac('ObstVmaxRol')   = fmt(vmax(1, 0.06, 3, 'uitrollen'), 2);
mac('ObstBbeschA')   = fmt(1000 * 2 * 0.22 * tan(deg2rad(7.5)), 0);
mac('ObstBbeschB')   = fmt(1000 * 2 * 0.22 * tan(deg2rad(15)), 0);
mac('ObstDnodigA')   = fmt(100 * (B / 2) / tan(deg2rad(7.5)), 0);
mac('ObstDnodigB')   = fmt(100 * (B / 2) / tan(deg2rad(15)), 0);
mac('ObstDeltaMax')  = fmt(100 * 0.40 * p.auto.b / 2^2, 1);
mac('ObstRcirkel')   = fmt(p.auto.b / (2 * 0.05), 1);
mac('ObstRzwaai')    = fmt(1000 * hypot(p.auto.achter, B / 2), 0);
mac('ObstRzwaaiKort') = fmt(1000 * hypot(0.090, B / 2), 0);
mac('ObstRvoor')     = fmt(1000 * hypot(p.auto.voor, B / 2), 0);
mac('ObstWmaxEen')   = fmt(2 * deg2rad(7.5) / 0.06, 1);
mac('ObstWmaxDrie')  = fmt(2 * deg2rad(7.5) / (3 * 0.0667), 1);
u = 2.0 * p.auto.b / 2 / v0;  up = p.motor.u_dz + (1 - p.motor.u_dz) * u;
mac('ObstDraaiU')    = fmt(100 * u, 0);
mac('ObstDraaiMin')  = fmt(100 * (((up - 0.22) / 0.78) / u - 1), 0);
mac('ObstDraaiMax')  = fmt(100 * (((up - 0.10) / 0.90) / u - 1), 0);
mac('ObstEncStap')   = fmt(rad2deg(p.enc.ds / p.auto.b), 1);

%% 2. Analytisch: welk deel van de autobreedte wordt gezien? (dekking vs uitdraaihoek)
%   Een punt (x, y) ligt in de bundel van sensor i als
%       | atan2(y - y_i, x - x_i) - alpha_i | <= beta
%   Een obstakel op zijafstand y wordt op tijd gezien als dat geldt voor een
%   x tussen x_a (remweg) en d_stop voor het sensorfront. Dekking = het deel
%   van |y| <= B/2 waarvoor dat zo is.
alph = deg2rad(0:2:60);
opst_a = {'uit2', 'kruis2', 'drie'};  bet = deg2rad([7.5 15]);
dek = zeros(numel(opst_a), numel(bet), numel(alph));
for i = 1:numel(opst_a)
    for j = 1:numel(bet)
        for k = 1:numel(alph)
            dek(i, j, k) = dekking(sonar_opstelling(opst_a{i}, alph(k), p), bet(j), B / 2, p.sonar.x + 0.05, p.sonar.x + 0.22);
        end
    end
end
dek_recht = [dekking(sonar_opstelling('recht', 0, p), bet(1), B / 2, p.sonar.x + 0.05, p.sonar.x + 0.22), ...
             dekking(sonar_opstelling('recht', 0, p), bet(2), B / 2, p.sonar.x + 0.05, p.sonar.x + 0.22)];
dk = squeeze(mean(dek(2, :, :), 2));            % kruis2, gemiddeld over beide beta
plateau = rad2deg(alph(dk >= max(dk) - 0.005));
mac('ObstDekRechtA') = fmt(100 * dek_recht(1), 0);
mac('ObstDekRechtB') = fmt(100 * dek_recht(2), 0);
mac('ObstAlfaKruisLo') = fmt(min(plateau), 0);
mac('ObstAlfaKruisHi') = fmt(max(plateau), 0);
mac('ObstDekKruisOpt') = fmt(100 * max(dk), 0);

%% 3. Huidig ontwerp en optimalisatie per opstelling, met en zonder encoders
p0 = p;  p0.var.delta_motor = 0.05;           % zoals nu: één sonar, ongekalibreerd
R0v = obst_valideren(p0, s_val, 5000);
R0r = obst_valideren(p0, s_rust, 5000, 'rustig');
fprintf('Huidig ontwerp: vrij %.0f %% (rommel), %.0f %% (rustig)\n', 100 * R0v.p_vrij, 100 * R0r.p_vrij);

th0 = [0.54 0.2 0.5 0 0.5 0.38 0.4 0 0.4 0.2 0.2 0.8 0.5];
opst = {'recht', 'kruis2', 'uit2', 'drie'};
enc  = [false true];
RES = struct([]);
for e = 1:2
    for o = 1:numel(opst)
        pe = p;  pe.obst.encoders = enc(e);
        A = obst_afstellen(pe, opst{o}, s_train, 1000, n_rand, n_lok, th0);
        Rv = obst_valideren(A.p, s_val, 5000);
        Rr = obst_valideren(A.p, s_rust, 5000, 'rustig');
        i = numel(RES) + 1;
        RES(i).opst = opst{o};  RES(i).enc = enc(e);  RES(i).A = A;  RES(i).Rv = Rv;  RES(i).Rr = Rr; %#ok<SAGROW>
        fprintf('%-7s enc=%d: train J=%.3f vrij %.0f%% | validatie vrij %.0f%% (rommel) %.0f%% (rustig), afstand %.1f m  (%.1f min)\n', ...
            opst{o}, enc(e), A.J, 100 * A.p_vrij, 100 * Rv.p_vrij, 100 * Rr.p_vrij, mean(Rv.afstand), toc(t_start) / 60);
    end
end
[~, ib] = min(arrayfun(@(r) r.A.J, RES));
best = RES(ib);
pbest = best.A.p;
[~, ibz] = min(arrayfun(@(r) r.A.J + 1e3 * r.enc, RES));   % beste zonder encoders
bestz = RES(ibz);

%% 4. Gevoeligheid, kortere achterkant
gam = [30 40 50 60];
[Pg_best, Pg_recht] = deal(zeros(size(gam)));
ir = find(strcmp({RES.opst}, 'recht') & ~[RES.enc], 1);
for g = 1:numel(gam)
    pg = pbest;  pg.var.gamma = gam(g) * [1 1];
    Pg_best(g) = obst_valideren(pg, s_val, 5000).p_vrij;
    pg = RES(ir).A.p;  pg.var.gamma = gam(g) * [1 1];
    Pg_recht(g) = obst_valideren(pg, s_val, 5000).p_vrij;
end
pk = pbest;  pk.auto.achter = 0.090;
Rk = obst_valideren(pk, s_val, 5000);

%% 5. Figuren
set(groot, 'defaultAxesFontSize', 8, 'defaultTextFontSize', 8, 'defaultLineLineWidth', 1.2);
kl = struct('b', [0.13 0.40 0.67], 'o', [0.90 0.45 0.10], 'g', [0.20 0.60 0.30], ...
            'r', [0.80 0.15 0.20], 'k', [0.15 0.15 0.15], 'gr', [0.6 0.6 0.6], 'p', [0.50 0.30 0.65]);

% 5a. remweg en maximale snelheid
fig = figure('Visible', 'off', 'Units', 'centimeters', 'Position', [0 0 17 5.8]);
tl = tiledlayout(fig, 1, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
nexttile(tl); hold on; box on; grid on;
vv = linspace(0, v0, 100);
plot(vv, 100 * s_rem(vv, 'kortsluit'), 'Color', kl.b, 'DisplayName', 'kortsluitrem (IN1 = IN2)');
plot(vv, 100 * s_rem(vv, 'tegenstroom'), 'Color', kl.g, 'DisplayName', 'tegenstroom');
plot(vv, 100 * s_rem(vv, 'uitrollen'), 'Color', kl.r, 'DisplayName', 'uitrollen (EN laag)');
plot(vv, 100 * vv * t_lat(1, 0.06, 3), '--', 'Color', kl.k, 'DisplayName', 'reactieweg, 1 sonar, N_f = 3');
xlabel('snelheid v [m/s]'); ylabel('afstand [cm]'); title('a. remweg en reactieweg');
legend('Location', 'northwest', 'Box', 'off', 'FontSize', 6.5);
nexttile(tl); hold on; box on; grid on;
TT = linspace(0.06, 0.20, 30);  nf = [1 3 5];  kn = {kl.b, kl.o, kl.r};
for j = 1:3
    plot(1000 * TT, arrayfun(@(T) vmax(1, T, nf(j), 'kortsluit'), TT), 'Color', kn{j}, ...
        'DisplayName', sprintf('1 sonar, N_f = %d', nf(j)));
    plot(1000 * TT, arrayfun(@(T) vmax(3, T, nf(j), 'kortsluit'), TT), ':', 'Color', kn{j}, ...
        'DisplayName', sprintf('3 sonars, N_f = %d', nf(j)));
end
xlabel('tijd tussen pulsen T_{meas} [ms]'); ylabel('v_{max} [m/s]');
title('b. hoogste snelheid die F2.3 haalt (kortsluitrem)'); ylim([0 v0]);
legend('Location', 'southwest', 'Box', 'off', 'FontSize', 6, 'NumColumns', 2);
opslaan(fig, uit, 'obst_remweg');

% 5b. dekking
fig = figure('Visible', 'off', 'Units', 'centimeters', 'Position', [0 0 17 6.2]);
tl = tiledlayout(fig, 1, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
nexttile(tl); hold on; box on; grid on;
kc = {kl.b, kl.o, kl.g};  ls = {'-', '--'};
for i = 1:3
    for j = 1:2
        plot(rad2deg(alph), 100 * squeeze(dek(i, j, :)), ls{j}, 'Color', kc{i}, ...
            'DisplayName', sprintf('%s, \\beta = %.1f°', opst_a{i}, rad2deg(bet(j))));
    end
end
yline(100 * dek_recht(1), '-', 'Color', kl.k, 'DisplayName', 'recht, \beta = 7,5°');
yline(100 * dek_recht(2), '--', 'Color', kl.k, 'DisplayName', 'recht, \beta = 15°');
xlabel('uitdraaihoek \alpha [°]'); ylabel('gedekte autobreedte [%]'); ylim([0 105]);
title('a. dekking van de rijstrook (5-22 cm voor de sensor)');
legend('Location', 'southoutside', 'Box', 'off', 'FontSize', 6, 'NumColumns', 3);
nexttile(tl); hold on; box on; axis equal;
teken_bundels(pbest, kl);
title(sprintf('b. optimum: %s, \\alpha = %.0f°', pbest.obst.opstelling, rad2deg(pbest.obst.toe)));
opslaan(fig, uit, 'obst_dekking');

% 5c. resultaten per opstelling
fig = figure('Visible', 'off', 'Units', 'centimeters', 'Position', [0 0 17 6]);
tl = tiledlayout(fig, 1, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
ax = nexttile(tl); hold on; box on; grid on;
nr = numel(RES);  X = 1:nr + 1;
Pv = [R0v.p_vrij, arrayfun(@(r) r.Rv.p_vrij, RES)];
Pr = [R0r.p_vrij, arrayfun(@(r) r.Rr.p_vrij, RES)];
nv = numel(s_val);
bh = bar(ax, X, 100 * [Pv; Pr]', 'grouped');
bh(1).FaceColor = kl.b;  bh(2).FaceColor = kl.o;
for m = 1:2
    Pm = [Pv; Pr];  xe = bh(m).XEndPoints;
    for i = 1:numel(xe)
        [lo, hi] = wilson(round(Pm(m, i) * nv), nv);
        plot(ax, [xe(i) xe(i)], 100 * [lo hi], 'k-', 'LineWidth', 0.8);
    end
end
lab = [{'huidig'}, arrayfun(@(r) sprintf('%s%s', r.opst, ternair(r.enc, '+enc', '')), RES, 'UniformOutput', false)];
ax.XTick = X;  ax.XTickLabel = lab;  ax.XTickLabelRotation = 40;
ylabel(ax, 'botsingsvrije ritten van 2 min [%]');  ylim(ax, [0 100]);
legend(ax, {'rommel', 'rustig'}, 'Location', 'northwest', 'Box', 'off');
title(ax, 'a. validatie in 100 nieuwe arena''s (95%-interval)');
ax = nexttile(tl); hold on; box on; grid on;
pp_ = linspace(0, 1, 101);
plot(ax, 100 * pp_, 100 * (3 * pp_.^2 - 2 * pp_.^3), 'Color', kl.k);
plot(ax, 100 * Pv, 100 * (3 * Pv.^2 - 2 * Pv.^3), 'o', 'Color', kl.b, 'MarkerFaceColor', kl.b);
plot(ax, 100 * Pr, 100 * (3 * Pr.^2 - 2 * Pr.^3), 's', 'Color', kl.o, 'MarkerFaceColor', kl.o);
xlabel(ax, 'kans p op een botsingsvrije rit [%]'); ylabel(ax, 'kans test gehaald [%]');
title(ax, 'b. F2: minstens 2 van 3 ritten goed = 3p^2 - 2p^3');
opslaan(fig, uit, 'obst_resultaat');

% 5d. gevoeligheid voor gamma_max
fig = figure('Visible', 'off', 'Units', 'centimeters', 'Position', [0 0 8.5 5.6]);
hold on; box on; grid on;
plot(gam, 100 * Pg_best, '-o', 'Color', kl.b, 'DisplayName', sprintf('optimum (%s)', pbest.obst.opstelling));
plot(gam, 100 * Pg_recht, '-s', 'Color', kl.r, 'DisplayName', 'één sonar recht (geoptimaliseerd)');
xlabel('grootste invalshoek met echo \gamma_{max} [°]'); ylabel('botsingsvrij [%]'); ylim([0 100]);
legend('Location', 'northwest', 'Box', 'off');
opslaan(fig, uit, 'obst_gamma');

% 5e. voorbeeldrit van het optimum
w = arena_cache(s_val(1));
[~, iv] = max(best.Rv.afstand .* ~best.Rv.botsing + 0.001 * best.Rv.afstand);
w = arena_cache(s_val(iv));
v = variatie_trekken(pbest, RandStream('mt19937ar', 'Seed', 5000 + s_val(iv)));
rr = sim_obstakels(pbest, w, v, true);
fig = figure('Visible', 'off', 'Units', 'centimeters', 'Position', [0 0 12 9]);
hold on; box on; axis equal;
teken_arena(w, kl);
tk = {kl.b, kl.r, kl.o, kl.p};  tn = {'rijden', 'remmen', 'kiezen', 'achteruit'};
for q = 1:4
    m = rr.log.toestand == q;
    plot(rr.log.x(m), rr.log.y(m), '.', 'Color', tk{q}, 'MarkerSize', 3, 'DisplayName', tn{q});
end
plot(rr.log.x(1), rr.log.y(1), 'kp', 'MarkerFaceColor', 'g', 'MarkerSize', 9, 'DisplayName', 'start');
xlim([-0.05 w.W + 0.05]); ylim([-0.05 w.H + 0.05]);
xlabel('x [m]'); ylabel('y [m]');
legend('Location', 'eastoutside', 'Box', 'off');
title(sprintf('%s: %.1f m in %.0f s, %d stops%s', w.naam, rr.afstand, rr.t_eind, rr.n_stops, ...
    ternair(rr.botsing, ', botsing', ', geen botsing')));
opslaan(fig, uit, 'obst_rit');

%% 6. Getallen, tabel, opslaan
[lo, hi] = wilson(round(best.Rv.p_vrij * nv), nv);
o = pbest.obst;
mac('ObstBestOpst')   = o.opstelling;
mac('ObstBestEnc')    = ternair(best.enc, 'met encoders', 'zonder encoders');
mac('ObstBestAlfa')   = fmt(rad2deg(o.toe), 0);
mac('ObstBestV')      = fmt(o.v_cruise, 2);
mac('ObstBestDstop')  = fmt(100 * o.d_stop, 0);
mac('ObstBestDslow')  = fmt(100 * o.d_slow, 0);
mac('ObstBestTmeas')  = fmt(1000 * o.T_meas, 0);
mac('ObstBestNf')     = sprintf('%d', o.N_f);
mac('ObstBestWdraai') = fmt(o.w_draai, 1);
mac('ObstBestStrat')  = o.strategie;
mac('ObstBestRem')    = o.rem;
mac('ObstBestEcho')   = ternair(o.echo_nodig, 'ja', 'nee');
mac('ObstBestExtra')  = fmt(rad2deg(o.phi_extra), 0);
mac('ObstBestKzij')   = fmt(o.k_zij, 1);
mac('ObstBestDzij')   = fmt(100 * o.d_zij, 0);
mac('ObstBestPv')     = fmt(100 * best.Rv.p_vrij, 0);
mac('ObstBestPvLo')   = fmt(100 * lo, 0);
mac('ObstBestPvHi')   = fmt(100 * hi, 0);
mac('ObstBestPr')     = fmt(100 * best.Rr.p_vrij, 0);
mac('ObstBestPtest')  = fmt(100 * (3 * best.Rv.p_vrij^2 - 2 * best.Rv.p_vrij^3), 0);
mac('ObstBestPtestR') = fmt(100 * (3 * best.Rr.p_vrij^2 - 2 * best.Rr.p_vrij^3), 0);
mac('ObstBestAfst')   = fmt(mean(best.Rv.afstand), 1);
mac('ObstBestZonder') = sprintf('%s', bestz.opst);
mac('ObstBestZonderPv') = fmt(100 * bestz.Rv.p_vrij, 0);
mac('ObstHuidigPv')   = fmt(100 * R0v.p_vrij, 0);
mac('ObstHuidigPr')   = fmt(100 * R0r.p_vrij, 0);
mac('ObstHuidigT')    = fmt(median(R0v.t_botsing(R0v.botsing)), 0);
mac('ObstKortPv')     = fmt(100 * Rk.p_vrij, 0);
mac('ObstGamDertig')  = fmt(100 * Pg_best(1), 0);
mac('ObstGamZestig')  = fmt(100 * Pg_best(end), 0);
mac('ObstNval')       = sprintf('%d', nv);
mac('ObstNtrain')     = sprintf('%d', numel(s_train));
mac('ObstNkand')      = sprintf('%d', n_rand + 6 * n_lok);
mac('ObstDuur')       = fmt(toc(t_start) / 60, 0);

fid = fopen(fullfile(uit, 'tabel_obst_opstellingen.tex'), 'w');
fprintf(fid, '\\begin{tabular}{@{}llrrrrr@{}}\n\\toprule\n');
fprintf(fid, 'opstelling & koers & botsingsvrij & botsingsvrij & test & afstand & fout F2.3/2.4 \\\\\n');
fprintf(fid, ' & & rommel & rustig & gehaald & [m/2 min] & per stop \\\\\n\\midrule\n');
rij(fid, 'huidig (1 recht, ongekal.)', 'tijd', R0v, R0r);
fprintf(fid, '\\midrule\n');
for i = 1:numel(RES)
    rij(fid, RES(i).opst, ternair(RES(i).enc, 'encoders', 'tijd'), RES(i).Rv, RES(i).Rr);
end
fprintf(fid, '\\bottomrule\n\\end{tabular}\n');
fclose(fid);

schrijf_macros(fullfile(uit, 'resultaten_obst.tex'), mac, 'onderzoek_obstakels.m');
save(fullfile(uit, 'obst_optimum.mat'), 'pbest', 'RES', 'R0v', 'R0r', 'Pg_best', 'Pg_recht', 'gam', 'Rk', 'dek', 'alph');
fprintf('\nKlaar in %.1f min. Optimum: %s (%s), vrij %.0f %% [%.0f-%.0f] rommel, %.0f %% rustig\n', toc(t_start) / 60, ...
    o.opstelling, ternair(best.enc, 'encoders', 'zonder encoders'), 100 * best.Rv.p_vrij, 100 * lo, 100 * hi, 100 * best.Rr.p_vrij);

%% ---------------------------------------------------------------------------
function v = vmax_rem(f, v0)
if f(v0) <= 0, v = v0; elseif f(1e-4) > 0, v = 0; else, v = fzero(f, [1e-4 v0]); end
end

function f = dekking(mnt, beta, halfB, xa, xb)
ys = linspace(-halfB, halfB, 141);
xs = linspace(xa, xb, 80)';
gezien = false(size(ys));
for i = 1:size(mnt, 1)
    hk = atan2(ys - mnt(i, 2), xs - mnt(i, 1)) - mnt(i, 3);
    gezien = gezien | any(abs(hk) <= beta, 1);
end
f = mean(gezien);
end

function [lo, hi] = wilson(k, n)
z = 1.96;  ph = k / n;
m = (ph + z^2 / (2 * n)) / (1 + z^2 / n);
h = z * sqrt(ph * (1 - ph) / n + z^2 / (4 * n^2)) / (1 + z^2 / n);
lo = max(0, m - h);  hi = min(1, m + h);
end

function s = ternair(c, a, b)
if c, s = a; else, s = b; end
end

function rij(fid, naam, koers, Rv, Rr)
pt = 3 * Rv.p_vrij^2 - 2 * Rv.p_vrij^3;
fe = sum(Rv.f23 + Rv.f24) / max(sum(Rv.stops), 1);
fprintf(fid, '%s & %s & %s\\,\\%% & %s\\,\\%% & %s\\,\\%% & %s & %s\\,\\%% \\\\\n', naam, koers, ...
    fmt(100 * Rv.p_vrij, 0), fmt(100 * Rr.p_vrij, 0), fmt(100 * pt, 0), fmt(mean(Rv.afstand), 1), fmt(100 * fe, 0));
end

function teken_bundels(p, kl)
A = p.auto;  B = A.breedte;
fill([-A.achter A.voor A.voor -A.achter], [-B/2 -B/2 B/2 B/2], [0.85 0.88 0.95], 'EdgeColor', kl.k);
plot([A.voor A.voor + 0.5], [B/2 + p.obst.marge, B/2 + p.obst.marge], ':', 'Color', kl.k);
plot([A.voor A.voor + 0.5], -[B/2 + p.obst.marge, B/2 + p.obst.marge], ':', 'Color', kl.k);
mnt = sonar_opstelling(p.obst.opstelling, p.obst.toe, p);
for bb = deg2rad([15 7.5])
    for i = 1:size(mnt, 1)
        a = mnt(i, 3) + linspace(-bb, bb, 30);
        r = 0.5;
        fill([mnt(i, 1), mnt(i, 1) + r * cos(a)], [mnt(i, 2), mnt(i, 2) + r * sin(a)], kl.o, ...
            'FaceAlpha', 0.15, 'EdgeColor', 'none');
    end
end
plot(mnt(:, 1), mnt(:, 2), 'ks', 'MarkerFaceColor', 'k', 'MarkerSize', 4);
plot(A.voor + [0.05 0.05], [-0.15 0.15], '--', 'Color', kl.gr);
plot(p.sonar.x + [0.22 0.22], [-0.15 0.15], '--', 'Color', kl.gr);
xlim([-0.15 0.6]); ylim([-0.25 0.25]); xlabel('x [m]'); ylabel('y [m]');
text(p.sonar.x + 0.225, 0.2, 'd_{stop}', 'FontSize', 7);
end

function teken_arena(w, kl)
plot([0 w.W w.W 0 0], [0 0 w.H w.H 0], 'Color', kl.k, 'LineWidth', 2, 'HandleVisibility', 'off');
for i = 1:numel(w.kist)
    fill(w.kist{i}(:, 1), w.kist{i}(:, 2), [0.75 0.75 0.75], 'EdgeColor', kl.k, 'HandleVisibility', 'off');
end
for i = 1:size(w.cirkel, 1)
    c = w.cirkel(i, :);
    rectangle('Position', [c(1) - c(3), c(2) - c(3), 2 * c(3), 2 * c(3)], 'Curvature', [1 1], ...
        'FaceColor', [0.4 0.4 0.4], 'EdgeColor', kl.k);
end
end

function s = fmt(x, d)
s = strrep(sprintf(['%.' num2str(d) 'f'], x), '.', '{,}');
end

function opslaan(fig, map, naam)
exportgraphics(fig, fullfile(map, [naam '.pdf']), 'ContentType', 'vector');
exportgraphics(fig, fullfile(map, [naam '.png']), 'Resolution', 170);
close(fig);
end

function schrijf_macros(bestand, mac, bron)
fid = fopen(bestand, 'w');
fprintf(fid, '%% automatisch gegenereerd door %s -- niet met de hand aanpassen\n', bron);
k = keys(mac);
for i = 1:numel(k)
    fprintf(fid, '\\newcommand{\\%s}{%s}\n', k{i}, mac(k{i}));
end
fclose(fid);
end
