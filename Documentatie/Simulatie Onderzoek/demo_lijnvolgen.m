function demo_lijnvolgen(baansoort, opt)
%DEMO_LIJNVOLGEN  3D-weergave van één ronde lijnvolgen: optimum naast huidige sim.
%
%   demo_lijnvolgen                    testparcours, optimum (blauw) + huidig (rood)
%   demo_lijnvolgen('haaks')           haaks parcours met 90-gradenbochten
%   demo_lijnvolgen('willekeurig')     willekeurige baan
%   demo_lijnvolgen(..., opt)          opt.video = true  -> MP4 in resultaten/
%                                      opt.snelheid = 2  -> twee keer zo snel afspelen
%                                      opt.vergelijk = false -> alleen het optimum
%                                      opt.volg = true   -> camera volgt de auto
%
%   Eerst wordt de hele ronde met sim_lijnvolgen doorgerekend (dezelfde kern
%   als in het onderzoek), daarna afgespeeld. Instellingen komen uit
%   resultaten/lijn_optimum.mat als onderzoek_lijnvolgen.m al gedraaid heeft,
%   anders uit qdat_parameters.m.
%   Toetsen tijdens het afspelen: spatie = pauze, c = camera volgen aan/uit.

if nargin < 1 || isempty(baansoort), baansoort = 'parcours'; end
if nargin < 2, opt = struct(); end
opt = standaard(opt, 'video', false, 'snelheid', 1, 'vergelijk', true, 'volg', false, ...
    'zichtbaar', true, 'plaatje', '', 'max_frames', inf);

root = fileparts(mfilename('fullpath'));
addpath(root, fullfile(root, 'model'), fullfile(root, 'optimalisatie'));
p = qdat_parameters();
pb = [];
bestand = fullfile(root, 'resultaten', 'lijn_optimum.mat');
if exist(bestand, 'file')
    S = load(bestand, 'pbest', 'pb');
    p = S.pbest;  pb = S.pb;
    bron = 'optimum uit onderzoek_lijnvolgen.m';
else
    bron = 'standaardwaarden uit qdat_parameters.m';
end
switch baansoort
    case 'haaks',       baan = baan_maken('haaks', 0.03);
    case 'willekeurig', baan = baan_maken('willekeurig', 21, 0.2, 8);
    otherwise,          baan = baan_maken('parcours');
end

fprintf('Rekenen (%s)...\n', bron);
ro = sim_lijnvolgen(p, baan, variatie_trekken(p), true);
fprintf('  optimum : %s in %.1f s, e_max = %.1f mm\n', ternair(ro.voltooid, 'ronde', 'MISLUKT'), ro.T, 1000 * ro.e_max);
runs = {ro};  pp = {p};
if opt.vergelijk && ~isempty(pb)
    rb = sim_lijnvolgen(pb, baan, variatie_trekken(pb), true);
    fprintf('  huidig  : %s in %.1f s, e_max = %.1f mm\n', ternair(rb.voltooid, 'ronde', 'MISLUKT'), rb.T, 1000 * rb.e_max);
    runs{2} = rb;  pp{2} = pb;
end

%% scène
M = auto_3d_model(p);
fig = figure('Name', 'Q-Dat robot-car - lijnvolgen', 'Color', 'w', 'Position', [60 60 1200 760], ...
    'Visible', ternair(opt.zichtbaar, 'on', 'off'));
ax = axes(fig);  hold(ax, 'on');  axis(ax, 'equal');  ax.Clipping = 'off';
ax.Color = [0.93 0.93 0.91];  grid(ax, 'on');
xlabel(ax, 'x [m]'); ylabel(ax, 'y [m]'); zlabel(ax, 'z [m]');
m = 0.3;
xl = [min(baan.x) - m, max(baan.x) + m];  yl = [min(baan.y) - m, max(baan.y) + m];
patch(ax, xl([1 2 2 1]), yl([1 1 2 2]), zeros(1, 4), [0.92 0.91 0.88], 'EdgeColor', 'none');
teken_tape(ax, baan, p.lijn.tape_w);
kleur = {[0.13 0.40 0.67], [0.80 0.15 0.20]};
naam  = {'optimum', 'huidige sim'};
for i = 1:numel(runs)
    h(i).body = patch(ax, 'Faces', M.body.F, 'Vertices', M.body.V, 'FaceColor', kleur{i}, ...
        'EdgeColor', 'none', 'FaceAlpha', ternair(i == 1, 0.95, 0.45)); %#ok<AGROW>
    h(i).wL = patch(ax, 'Faces', M.wiel.F, 'Vertices', M.wiel.V, 'FaceColor', [0.1 0.1 0.1], 'EdgeColor', 'none');
    h(i).wR = patch(ax, 'Faces', M.wiel.F, 'Vertices', M.wiel.V, 'FaceColor', [0.1 0.1 0.1], 'EdgeColor', 'none');
    h(i).spoor = plot3(ax, nan, nan, nan, '-', 'Color', kleur{i}, 'LineWidth', 1.2);
    N = numel(pp{i}.lijn.y);
    h(i).led = scatter3(ax, zeros(1, N), zeros(1, N), zeros(1, N), 30, repmat([0.2 0.8 0.2], N, 1), 'filled');
end
camlight(ax, 'headlight');  lighting(ax, 'gouraud');  material(ax, 'dull');
view(ax, 35, 50);  xlim(ax, xl);  ylim(ax, yl);  zlim(ax, [0 0.6]);
hud = annotation(fig, 'textbox', [0.01 0.80 0.30 0.18], 'FontName', 'Consolas', 'FontSize', 9, ...
    'BackgroundColor', [1 1 1 0.85], 'EdgeColor', [0.6 0.6 0.6], 'Interpreter', 'none');
title(ax, sprintf('Lijnvolgen - %s', baan.naam));
st.pauze = false;  st.volg = opt.volg;
set(fig, 'WindowKeyPressFcn', @(~, e) toets(e));

if opt.video
    vw = VideoWriter(fullfile(root, 'resultaten', ['lijnvolgen_' baansoort '.mp4']), 'MPEG-4');
    vw.FrameRate = 30;  open(vw);
end

%% afspelen
fps = 30;  dt = 1 / pp{1}.regel.f_s;
T_eind = max(cellfun(@(r) r.log.t(end), runs));
tf = 0:(opt.snelheid / fps):T_eind;
tf = tf(1:min(end, opt.max_frames));
for f = 1:numel(tf)
    if ~isvalid(fig), break; end
    while st.pauze && isvalid(fig), pause(0.05); end
    tekst = {};
    for i = 1:numel(runs)
        lg = runs{i}.log;
        k = min(numel(lg.t), max(1, round(tf(f) / (1 / pp{i}.regel.f_s))));
        zet_auto(h(i), M, pp{i}, lg.x(k), lg.y(k), lg.th(k), lg.s(k, :));
        set(h(i).spoor, 'XData', lg.x(1:k), 'YData', lg.y(1:k), 'ZData', 0.002 * ones(1, k));
        tekst{end + 1} = sprintf('%-11s t=%5.1f s  v=%.2f m/s  e=%+5.1f mm%s', naam{i}, lg.t(k), lg.v(k), ...
            1000 * lg.e(k), ternair(lg.kwijt(k), '  LIJN KWIJT', '')); %#ok<AGROW>
    end
    set(hud, 'String', [{'Q-Dat robot-car'}, tekst, {'spatie = pauze, c = camera'}]);
    if st.volg
        lg = runs{1}.log;  k = min(numel(lg.t), max(1, round(tf(f) / dt)));
        campos(ax, [lg.x(k) - 0.7 * cos(lg.th(k)), lg.y(k) - 0.7 * sin(lg.th(k)), 0.45]);
        camtarget(ax, [lg.x(k) + 0.3 * cos(lg.th(k)), lg.y(k) + 0.3 * sin(lg.th(k)), 0]);
    end
    drawnow limitrate;
    if opt.video, writeVideo(vw, getframe(fig)); elseif opt.zichtbaar, pause(1 / fps); end
end
if opt.video, close(vw); end
if ~isempty(opt.plaatje), exportgraphics(fig, opt.plaatje, 'Resolution', 110); end
if ~opt.zichtbaar, close(fig); end

    function toets(e)
        switch e.Key
            case 'space', st.pauze = ~st.pauze;
            case 'c'
                st.volg = ~st.volg;
                if ~st.volg, view(ax, 35, 50); xlim(ax, xl); ylim(ax, yl); end
        end
    end
end

% -------------------------------------------------------------------------
function zet_auto(h, M, p, x, y, th, s)
R = [cos(th) -sin(th) 0; sin(th) cos(th) 0; 0 0 1];
o = [x y 0];
set(h.body, 'Vertices', M.body.V * R' + o);
set(h.wL, 'Vertices', (M.wiel.V + [0 -p.auto.b/2 p.auto.r_w]) * R' + o);
set(h.wR, 'Vertices', (M.wiel.V + [0  p.auto.b/2 p.auto.r_w]) * R' + o);
ys = p.lijn.y(:);
P = [p.lijn.L * ones(size(ys)), ys, p.lijn.h * ones(size(ys))] * R' + o;
c = repmat([0.2 0.8 0.2], numel(ys), 1);
if islogical(s) || all(s == 0 | s == 1)
    c(s > 0.5, :) = repmat([0.9 0.1 0.1], nnz(s > 0.5), 1);
else
    c = [0.2 + 0.7 * s(:), 0.8 - 0.7 * s(:), 0.2 * ones(numel(s), 1)];
end
set(h.led, 'XData', P(:, 1), 'YData', P(:, 2), 'ZData', P(:, 3), 'CData', c);
end

function teken_tape(ax, baan, w)
n = numel(baan.x);
tx = gradient(baan.x);  ty = gradient(baan.y);  L = hypot(tx, ty);
nx = -ty ./ L;  ny = tx ./ L;
V = [[baan.x + nx * w/2; baan.y + ny * w/2; 0.001 * ones(1, n)]'; ...
     [baan.x - nx * w/2; baan.y - ny * w/2; 0.001 * ones(1, n)]'];
k = (1:n)';  k2 = mod(k, n) + 1;
F = [k, k2, k2 + n, k + n];
patch(ax, 'Faces', F, 'Vertices', V, 'FaceColor', [0.08 0.08 0.08], 'EdgeColor', 'none');
end

function s = standaard(s, varargin)
for i = 1:2:numel(varargin)
    if ~isfield(s, varargin{i}), s.(varargin{i}) = varargin{i + 1}; end
end
end

function s = ternair(c, a, b)
if c, s = a; else, s = b; end
end
