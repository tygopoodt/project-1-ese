function demo_obstakels(seed, opt)
%DEMO_OBSTAKELS  3D-weergave van een testrit autonoom rijden (F2) in een arena.
%
%   demo_obstakels            arena 1001 (uit de validatieset), optimale instellingen
%   demo_obstakels(seed)      andere arena
%   demo_obstakels(seed, opt) opt.video = true   -> MP4 in resultaten/
%                             opt.huidig = true  -> huidig ontwerp (één sonar recht)
%                             opt.rustig = true  -> arena met alleen grote kisten
%                             opt.snelheid = 2   -> twee keer zo snel afspelen
%
%   De rit wordt eerst met sim_obstakels doorgerekend (zelfde kern als het
%   onderzoek) en daarna afgespeeld. De gele waaiers zijn de sonarbundels; hun
%   lengte is de laatst gemeten afstand (rood = geen echo). Spatie = pauze.

if nargin < 1 || isempty(seed), seed = 1001; end
if nargin < 2, opt = struct(); end
opt = standaard(opt, 'video', false, 'huidig', false, 'rustig', false, 'snelheid', 1, ...
    'zichtbaar', true, 'plaatje', '', 'max_frames', inf);

root = fileparts(mfilename('fullpath'));
addpath(root, fullfile(root, 'model'), fullfile(root, 'optimalisatie'));
p = qdat_parameters();
bestand = fullfile(root, 'resultaten', 'obst_optimum.mat');
if ~opt.huidig && exist(bestand, 'file')
    S = load(bestand, 'pbest');  p = S.pbest;
end
w = arena_maken(seed, struct('rustig', opt.rustig));
v = variatie_trekken(p, RandStream('mt19937ar', 'Seed', 5000 + seed));
r = sim_obstakels(p, w, v, true);
fprintf('%s: %.1f m gereden, %d stops, %s\n', w.naam, r.afstand, r.n_stops, ...
    ternair(r.botsing, sprintf('BOTSING na %.1f s', r.t_botsing), 'geen botsing'));
lg = r.log;

%% scène
M = auto_3d_model(p);
fig = figure('Name', 'Q-Dat robot-car - autonoom rijden', 'Color', 'w', 'Position', [60 60 1200 760], ...
    'Visible', ternair(opt.zichtbaar, 'on', 'off'));
ax = axes(fig);  hold(ax, 'on');  axis(ax, 'equal');  grid(ax, 'on');
patch(ax, [0 w.W w.W 0], [0 0 w.H w.H], zeros(1, 4), [0.92 0.91 0.88], 'EdgeColor', 'none');
muur = [0 0; w.W 0; w.W w.H; 0 w.H];
for i = 1:4
    j = mod(i, 4) + 1;
    blok(ax, [muur(i, :); muur(j, :)], 0.02, 0.25, [0.75 0.75 0.78], 0.35);
end
for i = 1:numel(w.kist)
    extrudeer(ax, w.kist{i}, 0.18, [0.80 0.62 0.40]);
end
for i = 1:size(w.cirkel, 1)
    [X, Y, Z] = cylinder(w.cirkel(i, 3), 16);
    surf(ax, X + w.cirkel(i, 1), Y + w.cirkel(i, 2), 0.40 * Z, 'FaceColor', [0.35 0.35 0.35], 'EdgeColor', 'none');
end
hb = patch(ax, 'Faces', M.body.F, 'Vertices', M.body.V, 'FaceColor', [0.13 0.40 0.67], 'EdgeColor', 'none');
hwL = patch(ax, 'Faces', M.wiel.F, 'Vertices', M.wiel.V, 'FaceColor', [0.1 0.1 0.1], 'EdgeColor', 'none');
hwR = patch(ax, 'Faces', M.wiel.F, 'Vertices', M.wiel.V, 'FaceColor', [0.1 0.1 0.1], 'EdgeColor', 'none');
mnt = sonar_opstelling(p.obst.opstelling, p.obst.toe, p);
ns = size(mnt, 1);
for i = 1:ns
    hs(i) = patch(ax, 'Faces', M.sonar.F, 'Vertices', M.sonar.V, 'FaceColor', [0.1 0.35 0.7], 'EdgeColor', 'none'); %#ok<AGROW>
    hk(i) = patch(ax, nan, nan, nan, [1 0.85 0.1], 'FaceAlpha', 0.25, 'EdgeColor', 'none'); %#ok<AGROW>
end
hspoor = plot3(ax, nan, nan, nan, '-', 'Color', [0.13 0.40 0.67], 'LineWidth', 1);
camlight(ax, 'headlight');  lighting(ax, 'gouraud');  material(ax, 'dull');
view(ax, 30, 55);  xlim(ax, [-0.1 w.W + 0.1]);  ylim(ax, [-0.1 w.H + 0.1]);  zlim(ax, [0 0.6]);
hud = annotation(fig, 'textbox', [0.01 0.84 0.34 0.14], 'FontName', 'Consolas', 'FontSize', 9, ...
    'BackgroundColor', [1 1 1 0.85], 'EdgeColor', [0.6 0.6 0.6], 'Interpreter', 'none');
title(ax, sprintf('Autonoom rijden - %s - sonar: %s', w.naam, p.obst.opstelling));
pauze = false;
set(fig, 'WindowKeyPressFcn', @(~, e) wissel(e));
if opt.video
    vw = VideoWriter(fullfile(root, 'resultaten', sprintf('obstakels_%d.mp4', seed)), 'MPEG-4');
    vw.FrameRate = 30;  open(vw);
end

%% afspelen
d_laatst = nan(1, ns);  kvorig = 1;
toestanden = {'RIJDEN', 'REMMEN', 'KIEZEN', 'ACHTERUIT'};
fps = 30;  dtl = 1 / p.obst.f_s;
tf = 0:(opt.snelheid / fps):lg.t(end);
tf = tf(1:min(end, opt.max_frames));
for f = 1:numel(tf)
    if ~isvalid(fig), break; end
    while pauze && isvalid(fig), pause(0.05); end
    k = min(numel(lg.t), max(1, round(tf(f) / dtl) + 1));
    for q = kvorig:k
        g = ~isnan(lg.d_raw(q, :));  d_laatst(g) = lg.d_raw(q, g);
    end
    kvorig = k;
    th = lg.th(k);  R = [cos(th) -sin(th) 0; sin(th) cos(th) 0; 0 0 1];  o = [lg.x(k) lg.y(k) 0];
    set(hb, 'Vertices', M.body.V * R' + o);
    set(hwL, 'Vertices', (M.wiel.V + [0 -p.auto.b/2 p.auto.r_w]) * R' + o);
    set(hwR, 'Vertices', (M.wiel.V + [0  p.auto.b/2 p.auto.r_w]) * R' + o);
    for i = 1:ns
        Rs = [cos(mnt(i, 3)) -sin(mnt(i, 3)) 0; sin(mnt(i, 3)) cos(mnt(i, 3)) 0; 0 0 1];
        set(hs(i), 'Vertices', (M.sonar.V * Rs' + [mnt(i, 1) mnt(i, 2) p.sonar.z]) * R' + o);
        dl = d_laatst(i);  kleur = [1 0.85 0.1];
        if isnan(dl) || ~isfinite(dl), dl = 0.6; kleur = [0.9 0.2 0.2]; end
        a = mnt(i, 3) + linspace(-v.sonar.beta, v.sonar.beta, 12);
        P = [mnt(i, 1), mnt(i, 2); mnt(i, 1) + min(dl, 1.2) * cos(a'), mnt(i, 2) + min(dl, 1.2) * sin(a')];
        P = [P, p.sonar.z * ones(size(P, 1), 1)] * R' + o;
        set(hk(i), 'XData', P(:, 1), 'YData', P(:, 2), 'ZData', P(:, 3), 'FaceColor', kleur);
    end
    set(hspoor, 'XData', lg.x(1:k), 'YData', lg.y(1:k), 'ZData', 0.002 * ones(1, k));
    set(hud, 'String', {sprintf('t = %5.1f s   v = %+.2f m/s', lg.t(k), lg.v(k)), ...
        sprintf('toestand: %s', toestanden{max(1, lg.toestand(k))}), ...
        sprintf('vrij in rijstrook: %s', afstandtekst(lg.d_voor(k))), 'spatie = pauze'});
    drawnow limitrate;
    if opt.video, writeVideo(vw, getframe(fig)); elseif opt.zichtbaar, pause(1 / fps); end
end
if opt.video, close(vw); end
if ~isempty(opt.plaatje), exportgraphics(fig, opt.plaatje, 'Resolution', 110); end
if ~opt.zichtbaar, close(fig); end

    function wissel(e)
        if strcmp(e.Key, 'space'), pauze = ~pauze; end
    end
end

% -------------------------------------------------------------------------
function s = afstandtekst(d)
if isfinite(d), s = sprintf('%.2f m', d); else, s = 'geen echo'; end
end

function blok(ax, P, dikte, hoogte, kleur, alfa)
d = P(2, :) - P(1, :);  n = [-d(2) d(1)] / norm(d) * dikte / 2;
Q = [P(1, :) + n; P(2, :) + n; P(2, :) - n; P(1, :) - n];
extrudeer(ax, Q, hoogte, kleur, alfa);
end

function extrudeer(ax, P, h, kleur, alfa)
if nargin < 5, alfa = 1; end
n = size(P, 1);
V = [P zeros(n, 1); P h * ones(n, 1)];
F = [1:n; n + 1:2 * n];
Fz = zeros(n, 4);
for i = 1:n
    j = mod(i, n) + 1;
    Fz(i, :) = [i j j + n i + n];
end
patch(ax, 'Faces', F, 'Vertices', V, 'FaceColor', kleur, 'EdgeColor', [0.3 0.3 0.3], 'FaceAlpha', alfa);
patch(ax, 'Faces', Fz, 'Vertices', V, 'FaceColor', kleur * 0.85, 'EdgeColor', [0.3 0.3 0.3], 'FaceAlpha', alfa);
end

function s = standaard(s, varargin)
for i = 1:2:numel(varargin)
    if ~isfield(s, varargin{i}), s.(varargin{i}) = varargin{i + 1}; end
end
end

function s = ternair(c, a, b)
if c, s = a; else, s = b; end
end
