%% ONDERZOEK_LIJNVOLGEN  Simulatie 1: optimale lijnsensor-opstelling en regelaar (F3.1)
%
%  Onderzoeksvragen
%   L1. Hoe ver voor de aandrijfas moet de sensorbalk zitten (look-ahead L)?
%   L2. Welke onderlinge afstand (steek) tussen de vier sensoren is optimaal?
%   L3. Levert analoog uitlezen (ADC) of een ander aantal sensoren iets op?
%   L4. Hoe robuust is het optimum voor tapebreedte, lusfrequentie en
%       sensorhoogte, en hoeveel beter is het dan de huidige instellingen?
%
%  Methode
%   Voor ELKE sensoropstelling wordt de regelaar eerst zelf optimaal afgesteld
%   (lijn_afstellen: random search + lokale verfijning). Pas daarna worden
%   opstellingen vergeleken: een opstelling met een slecht afgestelde
%   regelaar vergelijken zegt niets over de opstelling.
%   Alle runs gebruiken domain randomization (variatie_trekken): motorverschil
%   L/R, dode zone, accuspanning, sensordrempels, -ruis en tapebreedte
%   verschillen per run. Het optimum wordt daarna gevalideerd op banen die de
%   optimizer nooit heeft gezien, waaronder het testparcours uit
%   RobotCar_LineFollower_Sim.m.
%
%  Uitvoer (map resultaten/):  figuren lijn_*.pdf/.png, resultaten_lijn.tex
%  (getallen voor het rapport), tabel_lijn_*.tex, lijn_optimum.mat (voor
%  demo_lijnvolgen.m).
%
%  Duur: ~10 min op 12 kernen. Proefrun (~1 min): setenv('QDAT_SNEL','1')

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
p.regel.v_min = 0.08;
mac = containers.Map();

if SNEL
    n_rand = 10;  n_lok = 3;  n_trek = 1;
    L_grid = [0.045 0.08 0.13];
    s_grid = [0.010 0.0127 0.018];
else
    n_rand = 40;  n_lok = 6;  n_trek = 3;
    L_grid = [0.03 0.045 0.06 0.08 0.10 0.13];
    s_grid = [0.008 0.010 0.0127 0.015 0.018 0.022];
end

%% 1. Banen: training (optimizer) en test (validatie)
train = {baan_maken('willekeurig', 11, 0.20, 6), baan_maken('willekeurig', 12, 0.15, 6), ...
         baan_maken('haaks', 0.03), baan_maken('willekeurig', 13, 0.30, 6)};
seed_train = 100 + (1:numel(train));
test = {baan_maken('parcours'), baan_maken('haaks', 0.01)};
for k = 1:8
    test{end + 1} = baan_maken('willekeurig', 20 + k, 0.15 + 0.05 * mod(k, 4), 7); %#ok<SAGROW>
end
if SNEL, test = test(1:3); end

%% 2. Analytische voorspelling (lineair model, zie rapport par. 3)
%   Lijnfout e aan de as, koersfout psi, sensor op afstand L voor de as:
%       e_s ~ e + L psi - L^2 kappa / 2,   de/dt = v psi,   dpsi/dt = omega - v kappa
%   P-regelaar omega = -K e_s geeft
%       e'' + K L e' + K v e = ...   ->   omega_n = sqrt(K v),  zeta = (L/2) sqrt(K/v)
%   en in een bocht met constante kromming kappa:  e_ss = kappa (L^2/2 - v/K).
%   K* = 2 v / L^2 maakt e_ss = 0 EN geeft zeta = 1/sqrt(2).
v_ref  = 0.30;
T_d    = p.motor.tau + 0.5 / p.regel.f_s;
s_uni  = p.lijn.tape_w / 1.5;                 % uniforme kwantisatie: w/s = k + 1/2
e_b    = 1.5 * s_uni + p.lijn.tape_w / 2;     % meetbereik 4 sensoren
R_krap = 0.15;
L_max  = sqrt(2 * R_krap * e_b);              % lijn blijft onder de balk in de krapste bocht
L_min  = sqrt(2) * v_ref * T_d / 0.3;         % vuistregel omega_n T_d <= 0,3 rad
v_bocht = p.motor.v0 / (1 + p.auto.b / (2 * R_krap));   % buitenwiel op v0
fprintf('Analytisch: T_d = %.0f ms, L_max(R=%.2f) = %.0f mm, L_min(v=%.1f) = %.0f mm, v_bocht = %.2f m/s, steek_uniform = %.1f mm\n', ...
    1000 * T_d, R_krap, 1000 * L_max, v_ref, 1000 * L_min, v_bocht, 1000 * s_uni);
mac('LijnVnul')     = fmt(p.motor.v0, 2);
mac('LijnTaum')     = fmt(1000 * p.motor.tau, 0);
mac('LijnTd')       = fmt(1000 * T_d, 0);
mac('LijnLmax')     = fmt(1000 * L_max, 0);
mac('LijnLmin')     = fmt(1000 * L_min, 0);
mac('LijnVbocht')   = fmt(v_bocht, 2);
mac('LijnSteekUni') = fmt(1000 * s_uni, 1);
mac('LijnEbereik')  = fmt(1000 * e_b, 1);
mac('LijnRkrap')    = fmt(100 * R_krap, 0);

%% 3. Huidige instellingen (RobotCar_LineFollower_Sim.m, teken gecorrigeerd)
%   Gewichten -1,8..1,8 horen bij -22..22 mm: 1 eenheid = 12,2 mm.
pb = p;
pb.lijn.L = 0.060;  pb.lijn.y = [-0.022 -0.007 0.007 0.022];
pb.regel.f_s = 50;  pb.regel.Kp = 28 / 0.01222;  pb.regel.Kd = 1.8 / 0.01222;
pb.regel.tau_d = 0;  pb.regel.v_max = 0.35;  pb.regel.v_min = 0.14;  pb.regel.k_v = 0.9;
pb.regel.w_max = 4.8;  pb.regel.snelheidswet = 'omega';

%% 4. Kaart: look-ahead L x steek s, bij elke opstelling een eigen optimale regelaar
nL = numel(L_grid);  nS = numel(s_grid);
[Jmap, Tmap, Emap] = deal(nan(nL, nS));  Okmap = false(nL, nS);  TH = zeros(nL, nS, 5);
for iL = 1:nL
    for iS = 1:nS
        pg = p;  pg.lijn.L = L_grid(iL);  pg.lijn.y = steek(4, s_grid(iS));
        vars = trek(pg, seed_train);
        th0 = [log10(2 * v_ref / L_grid(iL)^2), log10(0.5), log10(0.02), v_ref, 0.5];
        A = lijn_afstellen(pg, train, vars, n_rand, n_lok, th0);
        Jmap(iL, iS) = A.J;  Tmap(iL, iS) = A.Tn;  Emap(iL, iS) = A.e_max;
        Okmap(iL, iS) = A.ok;  TH(iL, iS, :) = A.theta;
        fprintf('L = %3.0f mm, steek = %4.1f mm: J = %6.3f  Tn = %.2f  e_max = %4.1f mm  ok = %d  (%.0f s)\n', ...
            1000 * L_grid(iL), 1000 * s_grid(iS), A.J, A.Tn, 1000 * A.e_max, A.ok, toc(t_start));
    end
end
[~, ib] = min(Jmap(:));
[iLb, iSb] = ind2sub([nL nS], ib);
L_best = L_grid(iLb);  s_best = s_grid(iSb);
pgeo = p;  pgeo.lijn.L = L_best;  pgeo.lijn.y = steek(4, s_best);
pbest = lijn_regelaar(pgeo, squeeze(TH(iLb, iSb, :))');
th_ana = [log10(2 * v_ref / L_best^2), log10(0.5), log10(0.02), v_ref, 0.5];
pana = lijn_regelaar(pgeo, th_ana);

%% 5. Aantal sensoren en analoog uitlezen (bij L_best)
var_def = {'2 sensoren',              2, s_best, 'digitaal';
           '3 sensoren',              3, s_best, 'digitaal';
           '4 sensoren (optimum)',    4, s_best, 'digitaal';
           '5 sensoren',              5, s_best, 'digitaal';
           '4 sensoren, analoog',     4, s_best, 'analoog';
           '4, analoog, steek 22 mm', 4, 0.022,  'analoog'};
nv = size(var_def, 1);
VAR = struct('naam', var_def(:, 1), 'J', 0, 'Tn', 0, 'Em', 0, 'ok', false, 'p', []);
for i = 1:nv
    pv = p;  pv.lijn.L = L_best;  pv.lijn.y = steek(var_def{i, 2}, var_def{i, 3});
    pv.lijn.modus = var_def{i, 4};
    if i == 3
        VAR(i).J = Jmap(iLb, iSb);  VAR(i).Tn = Tmap(iLb, iSb);
        VAR(i).Em = Emap(iLb, iSb);  VAR(i).ok = Okmap(iLb, iSb);  VAR(i).p = pbest;
        continue
    end
    A = lijn_afstellen(pv, train, trek(pv, seed_train), n_rand, n_lok, th_ana);
    VAR(i).J = A.J;  VAR(i).Tn = A.Tn;  VAR(i).Em = A.e_max;  VAR(i).ok = A.ok;
    VAR(i).p = lijn_regelaar(pv, A.theta);
    fprintf('%-26s J = %6.3f  Tn = %.2f  e_max = %4.1f mm\n', var_def{i, 1}, A.J, A.Tn, 1000 * A.e_max);
end

%% 6. Validatie op onbekende banen
pc = p;  pc.lijn.L = 0.060;  pc.lijn.y = [-0.022 -0.007 0.007 0.022];
Acur = lijn_afstellen(pc, train, trek(pc, seed_train), n_rand, n_lok, ...
    [log10(2 * v_ref / 0.06^2), log10(0.5), log10(0.02), v_ref, 0.5]);
pcur = lijn_regelaar(pc, Acur.theta);
cfg_naam = {'huidige sim (teken gecorrigeerd)', 'huidige sensoren, regelaar geoptimaliseerd', ...
            'optimale sensoren, analytische K^*', 'optimum (sensoren + regelaar)'};
cfg_p = {pb, pcur, pana, pbest};
VAL = cell(1, 4);
for i = 1:4
    VAL{i} = lijn_valideren(cfg_p{i}, test, n_trek, 5000);
    fprintf('%-44s voltooid %d/%d  e_max %5.1f mm  Tn %.2f  T_parcours %.1f s\n', cfg_naam{i}, ...
        sum(VAL{i}.Ok(:)), numel(VAL{i}.Ok), 1000 * max(VAL{i}.Em(:)), mean(VAL{i}.Tn(:)), mean(VAL{i}.T(1, :)));
end

%% 7. Robuustheid van het optimum
rob.tape  = [0.010 0.015 0.019 0.025];
rob.fs    = [50 100 200 500];
rob.sigma = [0.0015 0.003 0.005 0.008];
velden = {'tape', 'fs', 'sigma'};
for f = 1:3
    x = rob.(velden{f});
    [rob.([velden{f} '_E']), rob.([velden{f} '_T']), rob.([velden{f} '_ok'])] = deal(zeros(size(x)));
    for i = 1:numel(x)
        pr = pbest;
        switch velden{f}
            case 'tape',  pr.lijn.tape_w = x(i);
            case 'fs',    pr.regel.f_s = x(i);
            case 'sigma', pr.lijn.sigma = x(i);
        end
        V = lijn_valideren(pr, test, max(1, n_trek - 1), 7000);
        rob.([velden{f} '_E'])(i)  = max(V.Em(:));
        rob.([velden{f} '_T'])(i)  = mean(V.Tn(V.Ok));
        rob.([velden{f} '_ok'])(i) = mean(V.Ok(:));
    end
end

%% 8. Figuren
set(groot, 'defaultAxesFontSize', 8, 'defaultTextFontSize', 8, 'defaultLineLineWidth', 1.2);
kl = struct('b', [0.13 0.40 0.67], 'o', [0.90 0.45 0.10], 'g', [0.20 0.60 0.30], ...
            'r', [0.80 0.15 0.20], 'k', [0.15 0.15 0.15], 'gr', [0.6 0.6 0.6]);

% 8a. Sensormodel en kwantisatie
fig = figure('Visible', 'off', 'Units', 'centimeters', 'Position', [0 0 17 5.6]);
tl = tiledlayout(fig, 1, 3, 'TileSpacing', 'compact', 'Padding', 'compact');
nexttile(tl); hold on; box on; grid on;
d = linspace(-0.03, 0.03, 601);  w = p.lijn.tape_w;
sg = [0.0015 0.003 0.006];  kc = {kl.b, kl.o, kl.g};
for i = 1:3
    plot(1000 * d, dekking(d, w, sg(i)), 'Color', kc{i}, 'DisplayName', sprintf('\\sigma = %.1f mm', 1000 * sg(i)));
end
yline(0.5, ':', 'Color', kl.gr, 'HandleVisibility', 'off');
xlabel('afstand sensor - tapehartlijn d [mm]'); ylabel('dekkingsgraad c(d)');
title('a. sensorrespons, tape 19 mm'); legend('Location', 'south', 'Box', 'off');
nexttile(tl); hold on; box on; grid on;
e = linspace(-0.035, 0.035, 2801);
plot(1000 * e, 1000 * e, ':', 'Color', kl.gr, 'HandleVisibility', 'off');
plot(1000 * e, 1000 * schatting(e, pb.lijn.y, w, 0.003, 'digitaal'), 'Color', kl.r, 'DisplayName', 'nu: -22 -7 7 22 mm');
plot(1000 * e, 1000 * schatting(e, steek(4, s_uni), w, 0.003, 'digitaal'), 'Color', kl.b, ...
    'DisplayName', sprintf('steek %.1f mm (w/1,5)', 1000 * s_uni));
xlabel('werkelijke lijnpositie e [mm]'); ylabel('geschat \^e [mm]');
title('b. digitaal: kwantisatie'); legend('Location', 'northwest', 'Box', 'off');
nexttile(tl); hold on; box on; grid on;
plot(1000 * e, 1000 * e, ':', 'Color', kl.gr, 'HandleVisibility', 'off');
plot(1000 * e, 1000 * schatting(e, steek(4, s_best), w, 0.003, 'analoog'), 'Color', kl.b, ...
    'DisplayName', sprintf('steek %.1f mm', 1000 * s_best));
plot(1000 * e, 1000 * schatting(e, steek(4, 0.022), w, 0.003, 'analoog'), 'Color', kl.o, 'DisplayName', 'steek 22 mm');
xlabel('werkelijke lijnpositie e [mm]'); ylabel('geschat \^e [mm]');
title('c. analoog (ADC)'); legend('Location', 'northwest', 'Box', 'off');
opslaan(fig, uit, 'lijn_sensormodel');

% 8b. Kaart
fig = figure('Visible', 'off', 'Units', 'centimeters', 'Position', [0 0 17 6.4]);
tl = tiledlayout(fig, 1, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
haalbaar = Okmap & Emap <= p.doel.e_toel;
kaarten = {Tmap, 1000 * Emap};  tit = {'a. genormaliseerde rondetijd T v_0/L', 'b. grootste afwijking e_{max} [mm]'};
for m = 1:2
    ax = nexttile(tl);
    K = kaarten{m};  K(~haalbaar) = NaN;
    imagesc(ax, K, 'AlphaData', ~isnan(K));  ax.Color = [0.85 0.85 0.85];
    colormap(ax, flipud(parula));  colorbar(ax);
    for iL = 1:nL
        for iS = 1:nS
            if haalbaar(iL, iS)
                if m == 1, txt = sprintf('%.2f', Tmap(iL, iS)); else, txt = sprintf('%.0f', 1000 * Emap(iL, iS)); end
            else
                txt = 'x';
            end
            if iL == iLb && iS == iSb, txt = ['[' txt ']']; end
            text(ax, iS, iL, txt, 'HorizontalAlignment', 'center', 'FontSize', 7);
        end
    end
    ax.XTick = 1:nS;  ax.XTickLabel = compose('%.1f', 1000 * s_grid);
    ax.YTick = 1:nL;  ax.YTickLabel = compose('%.0f', 1000 * L_grid);
    xlabel(ax, 'steek s [mm]');  ylabel(ax, 'look-ahead L [mm]');  title(ax, tit{m});
end
opslaan(fig, uit, 'lijn_kaart');

% 8c. Venster voor L
fig = figure('Visible', 'off', 'Units', 'centimeters', 'Position', [0 0 8.5 6]);
hold on; box on; grid on;
R = linspace(0.08, 0.6, 200);
eb_best = 1.5 * s_best + w / 2;  eb_nu = 0.022 + w / 2;
plot(R, 1000 * sqrt(2 * R * eb_best), 'Color', kl.b, 'DisplayName', sprintf('L_{max}, bereik %.0f mm', 1000 * eb_best));
plot(R, 1000 * sqrt(2 * R * eb_nu), '--', 'Color', kl.b, 'DisplayName', sprintf('L_{max}, bereik %.1f mm (nu)', 1000 * eb_nu));
vv = [0.2 0.3 0.4];  kv = {kl.g, kl.o, kl.r};
for i = 1:3
    yline(1000 * sqrt(2) * vv(i) * T_d / 0.3, ':', 'Color', kv{i}, 'LineWidth', 1.2, ...
        'DisplayName', sprintf('L_{min} bij v = %.1f m/s', vv(i)));
end
plot(R_krap, 1000 * L_best, 'kp', 'MarkerFaceColor', 'k', 'MarkerSize', 8, 'DisplayName', 'simulatie-optimum');
xlabel('krapste bochtstraal R [m]'); ylabel('look-ahead L [mm]'); ylim([0 200]);
legend('Location', 'northwest', 'Box', 'off', 'FontSize', 6.5);
opslaan(fig, uit, 'lijn_venster');

% 8d. Tijdreeks en baan in een haakse bocht: huidig vs optimum
bh = baan_maken('haaks', 0.03);
rb = sim_lijnvolgen(pb, bh, variatie_trekken(pb), true);
ro = sim_lijnvolgen(pbest, bh, variatie_trekken(pbest), true);
fig = figure('Visible', 'off', 'Units', 'centimeters', 'Position', [0 0 17 6]);
tl = tiledlayout(fig, 1, 3, 'TileSpacing', 'compact', 'Padding', 'compact');
nexttile(tl, [1 2]); hold on; box on; grid on;
plot(rb.log.t, 1000 * rb.log.e, 'Color', kl.r, 'DisplayName', 'huidige sim');
plot(ro.log.t, 1000 * ro.log.e, 'Color', kl.b, 'DisplayName', 'optimum');
yline(1000 * [-0.05 0.05], '--', 'Color', kl.k, 'HandleVisibility', 'off');
xlabel('tijd [s]'); ylabel('afwijking aandrijfas e [mm]'); legend('Location', 'southeast', 'Box', 'off');
title('a. haaks parcours (10,7 m, hoekstraal 30 mm); stippellijn = eis F3.1');
nexttile(tl); hold on; box on; axis equal;
plot(bh.x, bh.y, 'Color', [0.1 0.1 0.1], 'LineWidth', 4);
plot(rb.log.x, rb.log.y, 'Color', kl.r);  plot(ro.log.x, ro.log.y, 'Color', kl.b);
xlim([2.75 3.15]); ylim([-0.12 0.28]); xlabel('x [m]'); ylabel('y [m]'); title('b. eerste bocht');
opslaan(fig, uit, 'lijn_tijdreeks');

% 8e. Robuustheid
fig = figure('Visible', 'off', 'Units', 'centimeters', 'Position', [0 0 17 5.2]);
tl = tiledlayout(fig, 1, 3, 'TileSpacing', 'compact', 'Padding', 'compact');
xl = {'tapebreedte [mm]', 'regellusfrequentie [Hz]', 'breedte lichtvlek \sigma [mm]'};
sc = [1000 1 1000];
for f = 1:3
    ax = nexttile(tl); hold(ax, 'on'); box(ax, 'on'); grid(ax, 'on');
    x = sc(f) * rob.(velden{f});
    yyaxis(ax, 'left');  plot(ax, x, 1000 * rob.([velden{f} '_E']), '-o', 'Color', kl.b);
    yline(ax, 1000 * p.doel.e_toel, ':', 'Color', kl.b);
    ylabel(ax, 'e_{max} [mm]');  ax.YColor = kl.b;
    yyaxis(ax, 'right'); plot(ax, x, 100 * rob.([velden{f} '_ok']), '-s', 'Color', kl.o);
    ylabel(ax, 'voltooid [%]');  ylim(ax, [0 105]);  ax.YColor = kl.o;
    xlabel(ax, xl{f});
    if f == 2, ax.XScale = 'log'; ax.XTick = rob.fs; end
end
opslaan(fig, uit, 'lijn_robuust');

%% 9. Getallen en tabellen voor het rapport
Vb = VAL{4};  V0 = VAL{1};
mac('LijnBestL')      = fmt(1000 * L_best, 0);
mac('LijnBestSteek')  = fmt(1000 * s_best, 1);
mac('LijnBestKp')     = fmt(pbest.regel.Kp, 0);
mac('LijnBestKd')     = fmt(pbest.regel.Kd, 2);
mac('LijnBestTaud')   = fmt(1000 * pbest.regel.tau_d, 0);
mac('LijnBestVmax')   = fmt(pbest.regel.v_max, 2);
mac('LijnBestKv')     = fmt(pbest.regel.k_v, 2);
mac('LijnBestKster')  = fmt(2 * pbest.regel.v_max / L_best^2, 0);
mac('LijnBestBereik') = fmt(1000 * (1.5 * s_best + w / 2), 1);
mac('LijnBestPwmMm')  = fmt(pbest.regel.Kp * 0.001 * p.auto.b / 2 / p.motor.v0 * 255, 1);
mac('LijnBestTpar')   = fmt(mean(Vb.T(1, :)), 1);
mac('LijnBestVpar')   = fmt(test{1}.L / mean(Vb.T(1, :)), 2);
mac('LijnBestEmax')   = fmt(1000 * max(Vb.Em(:)), 0);
mac('LijnBestOk')     = sprintf('%d/%d', sum(Vb.Ok(:)), numel(Vb.Ok));
mac('LijnBaseTpar')   = fmt(mean(V0.T(1, :)), 1);
mac('LijnBaseEmax')   = fmt(1000 * max(V0.Em(:)), 0);
mac('LijnBaseOk')     = sprintf('%d/%d', sum(V0.Ok(:)), numel(V0.Ok));
mac('LijnBaseJit')    = fmt(mean(V0.Jt(:)), 2);
mac('LijnBestJit')    = fmt(mean(Vb.Jt(:)), 3);
mac('LijnBaseKp')     = fmt(pb.regel.Kp, 0);
mac('LijnNtest')      = sprintf('%d', numel(test));
mac('LijnNrun')       = sprintf('%d', numel(Vb.Ok));
mac('LijnTapeTienE')  = fmt(1000 * rob.tape_E(1), 0);
mac('LijnTapeTienOk') = fmt(100 * rob.tape_ok(1), 0);
mac('LijnDuur')       = fmt(toc(t_start) / 60, 0);
mac('LijnNkand')      = sprintf('%d', n_rand + 6 * n_lok);

fid = fopen(fullfile(uit, 'tabel_lijn_validatie.tex'), 'w');
fprintf(fid, '\\begin{tabular}{@{}lrrrrr@{}}\n\\toprule\n');
fprintf(fid, ' & voltooid & $T$ testparcours & $\\bar v$ & $e_{\\max}$ & gejitter \\\\\n');
fprintf(fid, ' & & [s] & [m/s] & [mm] & [PWM/stap] \\\\\n\\midrule\n');
for i = 1:4
    V = VAL{i};
    if all(V.Ok(1, :)), tp = fmt(mean(V.T(1, :)), 1); vp = fmt(test{1}.L / mean(V.T(1, :)), 2);
    else, tp = '--'; vp = '--'; end
    fprintf(fid, '%s & %d/%d & %s & %s & %s & %s \\\\\n', strrep(cfg_naam{i}, 'K^*', '$K^*$'), ...
        sum(V.Ok(:)), numel(V.Ok), tp, vp, fmt(1000 * max(V.Em(:)), 0), fmt(mean(V.Jt(:)), 3));
end
fprintf(fid, '\\bottomrule\n\\end{tabular}\n');
fclose(fid);

fid = fopen(fullfile(uit, 'tabel_lijn_varianten.tex'), 'w');
fprintf(fid, '\\begin{tabular}{@{}lrrr@{}}\n\\toprule\n');
fprintf(fid, 'variant (bij $L = %s$~mm) & kost $J^*$ & $T v_0/L$ & $e_{\\max}$ [mm] \\\\\n\\midrule\n', fmt(1000 * L_best, 0));
for i = 1:nv
    if VAR(i).ok, tn = fmt(VAR(i).Tn, 2); else, tn = 'mislukt'; end
    fprintf(fid, '%s & %s & %s & %s \\\\\n', VAR(i).naam, fmt(VAR(i).J, 2), tn, fmt(1000 * VAR(i).Em, 0));
end
fprintf(fid, '\\bottomrule\n\\end{tabular}\n');
fclose(fid);

schrijf_macros(fullfile(uit, 'resultaten_lijn.tex'), mac, 'onderzoek_lijnvolgen.m');
save(fullfile(uit, 'lijn_optimum.mat'), 'pbest', 'pb', 'pcur', 'pana', 'Jmap', 'Tmap', 'Emap', ...
    'Okmap', 'TH', 'L_grid', 's_grid', 'VAR', 'VAL', 'rob', 'cfg_naam');
fprintf('\nKlaar in %.1f min. Optimum: L = %.0f mm, steek = %.1f mm, Kp = %.0f, Kd = %.2f, tau_d = %.0f ms, v_max = %.2f, k_v = %.2f\n', ...
    toc(t_start) / 60, 1000 * L_best, 1000 * s_best, pbest.regel.Kp, pbest.regel.Kd, ...
    1000 * pbest.regel.tau_d, pbest.regel.v_max, pbest.regel.k_v);

%% ---------------------------------------------------------------------------
function y = steek(N, s)
y = ((1:N) - (N + 1) / 2) * s;
end

function vars = trek(p, seeds)
vars = cell(size(seeds));
for i = 1:numel(seeds)
    vars{i} = variatie_trekken(p, RandStream('mt19937ar', 'Seed', seeds(i)));
end
end

function c = dekking(d, w, sigma)
c = 0.5 * (erf((w / 2 - d) / (sqrt(2) * sigma)) + erf((w / 2 + d) / (sqrt(2) * sigma)));
end

function eh = schatting(e, y, w, sigma, modus)
% Geschatte lijnpositie zonder ruis, voor de kwantisatiefiguur.
eh = nan(size(e));
for k = 1:numel(e)
    a = dekking(abs(y - e(k)), w, sigma);
    if strcmp(modus, 'digitaal')
        b = a > 0.5;
        if any(b), eh(k) = mean(y(b)); end
    else
        g = max(a - 0.2, 0);
        if max(a) > 0.3, eh(k) = sum(y .* g) / sum(g); end
    end
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
