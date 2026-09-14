function w = arena_maken(seed, opt)
%ARENA_MAKEN  Willekeurige testruimte met obstakels voor autonoom rijden (F2).
%
%   w = arena_maken(seed)
%   w = arena_maken(seed, opt)   opt.W, opt.H (afmetingen [m]), opt.n (aantal
%                                obstakels), opt.gap (kleinste doorgang [m]),
%                                opt.rustig (alleen kisten >= 15 cm, geen palen)
%
%   Een rechthoekige ruimte van 2-3 m bij 1,6-2,4 m met 4-8 obstakels:
%   ~60 % kisten van 8-35 cm onder een willekeurige hoek (schuine vlakken:
%   daar mist de HC-SR04 soms de echo) en ~40 % ronde obstakels met straal
%   1-5 cm (stoel- en tafelpoten, flessen). Tussen obstakels en muren blijft
%   minstens opt.gap vrij, zodat de auto (143 mm breed) er in principe door kan.
%
%   Uitvoer:
%     w.seg     muren en kistzijden als lijnstukken [x1 y1 x2 y2]  (sonar)
%     w.cirkel  ronde obstakels [xc yc r]                            (sonar)
%     w.kist    cell met hoekpunten van elke kist                    (tekenen)
%     w.sdf     afstandskaart: afstand van elk vrij rastervak tot het
%               dichtstbijzijnde obstakel [m] (bwdist), voor botsingscontrole
%     w.start   [x y theta] vrije startpositie

if nargin < 2, opt = struct(); end
rs  = RandStream('mt19937ar', 'Seed', seed);
W   = veld(opt, 'W', 2.0 + rs.rand());
H   = veld(opt, 'H', 1.6 + 0.8 * rs.rand());
nob = veld(opt, 'n', 4 + rs.randi(4));
gap = veld(opt, 'gap', 0.25);

seg = [0 0 W 0; W 0 W H; W H 0 H; 0 H 0 0];
cirkel = zeros(0, 3);
kist = {};
obj = zeros(0, 3);                       % [xc yc R_omhullend]
for i = 1:nob
    for poging = 1:200
        is_kist = rs.rand() < 0.6 || veld(opt, 'rustig', false);
        if is_kist
            a = 0.08 + 0.27 * rs.rand();  b = 0.08 + 0.27 * rs.rand();
            if veld(opt, 'rustig', false), a = max(a, 0.15); b = max(b, 0.15); end
            phi = rs.rand() * pi / 2;
            R = hypot(a, b) / 2;
        else
            r = 0.01 + 0.04 * rs.rand();
            R = r;
        end
        vx = W - 2 * (R + gap);  vy = H - 2 * (R + gap);
        if vx <= 0 || vy <= 0, continue; end
        c = [R + gap + vx * rs.rand(), R + gap + vy * rs.rand()];
        if ~isempty(obj) && any(hypot(obj(:, 1) - c(1), obj(:, 2) - c(2)) - obj(:, 3) - R < gap)
            continue
        end
        obj(end + 1, :) = [c R]; %#ok<AGROW>
        if is_kist
            Rm = [cos(phi) -sin(phi); sin(phi) cos(phi)];
            P  = (Rm * [-a a a -a; -b -b b b] / 2 + c')';
            kist{end + 1} = P; %#ok<AGROW>
            seg = [seg; P, P([2 3 4 1], :)]; %#ok<AGROW>
        else
            cirkel(end + 1, :) = [c r]; %#ok<AGROW>
        end
        break
    end
end

% --- afstandskaart voor de botsingscontrole -----------------------------
res = 0.004;  m = 0.02;
xg = -m + res/2 : res : W + m;
yg = -m + res/2 : res : H + m;
[X, Y] = meshgrid(xg, yg);
bezet = X < 0 | X > W | Y < 0 | Y > H;
for i = 1:numel(kist)
    bezet = bezet | inpolygon(X, Y, kist{i}(:, 1), kist{i}(:, 2));
end
for i = 1:size(cirkel, 1)
    bezet = bezet | (X - cirkel(i, 1)).^2 + (Y - cirkel(i, 2)).^2 <= cirkel(i, 3)^2;
end
sdf = single(bwdist(bezet) * res);

% --- vrije startpositie ------------------------------------------------------
vrij = find(sdf >= 0.30);
k0 = vrij(rs.randi(numel(vrij)));

% Voor de sonar: kistranden en binnenhoeken van de ruimte geven ook een echo
% als het vlak ernaast te schuin staat (diffractie aan een rand; een
% binnenhoek werkt als hoekreflector). Benaderd als dunne cilinders.
hoeken = [0 0; W 0; W H; 0 H];
for i = 1:numel(kist), hoeken = [hoeken; kist{i}]; end %#ok<AGROW>
w.cirkel_sonar = [cirkel; hoeken, 0.004 * ones(size(hoeken, 1), 1)];

w.W = W;  w.H = H;
w.seg = seg;  w.cirkel = cirkel;  w.kist = kist;
w.sdf = sdf;  w.res = res;  w.x0 = xg(1);  w.y0 = yg(1);
w.start = [X(k0), Y(k0), 2 * pi * rs.rand()];
w.naam = sprintf('arena %d (%.1f x %.1f m, %d obstakels)', seed, W, H, numel(kist) + size(cirkel, 1));
end

function v = veld(s, naam, standaard)
if isfield(s, naam), v = s.(naam); else, v = standaard; end
end
