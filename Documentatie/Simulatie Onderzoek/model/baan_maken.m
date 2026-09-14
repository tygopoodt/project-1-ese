function baan = baan_maken(soort, varargin)
%BAAN_MAKEN  Lijnparcours als dicht bemonsterde hartlijn van de zwarte tape.
%
%   baan = baan_maken('parcours')
%       Het testparcours met 30 controlepunten uit RobotCar_LineFollower_Sim.m.
%   baan = baan_maken('haaks', r_hoek)
%       Rechthoek van 3,0 x 1,6 m met een inham, dus linker- en rechterbochten
%       van 90 graden. r_hoek [m] is de straal waarmee de tape de hoek om gaat
%       (standaard 0,03 m: tape die in een hoek wordt geplakt).
%   baan = baan_maken('willekeurig', seed, R_min, lengte)
%       Gesloten, zichzelf niet kruisende lus met willekeurige bochten. De
%       krapste bochtstraal is R_min [m] (standaard 0,25) en de lengte is
%       lengte [m] (standaard 8). Zelfde seed = zelfde baan.
%
%   Uitvoer (struct):
%       x, y    hartlijn [m], om de ds = 2 mm
%       s       booglengte [m]
%       kappa   kromming [1/m], positief = bocht naar links
%       L       lengte van de lus [m]
%       naam    omschrijving voor in figuren
%
%   Kromming uit de afgeleiden naar de booglengte s:
%       kappa = (x' y'' - y' x'') / (x'^2 + y'^2)^(3/2)

ds = 0.002;

switch soort
    case 'parcours'
        P = [ 0.00  0.00;  1.10  0.05;  2.10  0.30;  2.90  0.90;  3.30  1.90;
              2.90  2.90;  2.00  3.20;  1.00  2.70;  0.40  1.90; -0.10  1.30;
             -0.70  1.10; -1.50  1.50; -1.90  2.50; -2.60  3.20; -3.30  2.70;
             -3.50  1.60; -2.90  0.60; -2.10  0.00; -2.70 -0.90; -3.30 -1.90;
             -2.80 -2.80; -1.60 -2.70; -0.80 -1.90;  0.20 -2.30;  1.30 -2.60;
              2.30 -2.20;  2.80 -1.30;  2.30 -0.60;  1.20 -0.25;  0.00  0.00];
        t  = linspace(0, 1, size(P, 1));
        tf = linspace(0, 1, 40000);
        x  = spline(t, P(:, 1)', tf);
        y  = spline(t, P(:, 2)', tf);
        naam = 'testparcours (30 punten)';

    case 'haaks'
        r_hoek = optarg(varargin, 1, 0.03);
        H = [0 0; 3 0; 3 1.6; 1.8 1.6; 1.8 0.8; 1.2 0.8; 1.2 1.6; 0 1.6];
        [x, y] = afgeronde_veelhoek(H, r_hoek);
        naam = sprintf('haaks parcours, hoekstraal %.0f mm', 1000 * r_hoek);

    case 'willekeurig'
        seed   = optarg(varargin, 1, 1);
        R_min  = optarg(varargin, 2, 0.25);
        lengte = optarg(varargin, 3, 8.0);
        rs = RandStream('mt19937ar', 'Seed', seed);
        phi = linspace(0, 2*pi, 6001); phi(end) = [];
        k = (2:6)';
        a = rs.rand(5, 1) .* 0.30 ./ k.^1.1;
        f = 2*pi * rs.rand(5, 1);
        rek = 1 + 0.8 * rs.rand();              % lengte/breedte-verhouding
        for poging = 1:60
            r = 1 + sum(a .* cos(k .* phi + f), 1);
            x = rek * r .* cos(phi);
            y = r .* sin(phi);
            L0 = sum(hypot(diff([x x(1)]), diff([y y(1)])));
            x = x * lengte / L0;  y = y * lengte / L0;
            kap = kromming(x, y);
            if max(abs(kap)) <= 1 / R_min && min(r) > 0.3, break; end
            a = 0.85 * a;                       % bochten te krap: amplitudes omlaag
        end
        x = [x x(1)];  y = [y y(1)];
        naam = sprintf('willekeurige baan %d (R_{min} = %.2f m)', seed, R_min);

    otherwise
        error('baan_maken: onbekende baansoort ''%s''.', soort);
end

% Herbemonsteren op vaste booglengte ds
seg = hypot(diff(x), diff(y));
keep = [true, seg > 1e-9];
x = x(keep);  y = y(keep);
s0 = [0, cumsum(hypot(diff(x), diff(y)))];
L  = s0(end);
s  = 0:ds:L - ds;
baan.x = interp1(s0, x, s);
baan.y = interp1(s0, y, s);

if strcmp(soort, 'haaks')           % start midden op de onderste lange zijde
    [~, i0] = min(hypot(baan.x - 1.5, baan.y));
    baan.x = circshift(baan.x, -(i0 - 1));
    baan.y = circshift(baan.y, -(i0 - 1));
end

baan.s     = s;
baan.L     = L;
baan.ds    = ds;
baan.kappa = movmean(kromming(baan.x, baan.y), 7);
baan.naam  = naam;
end

% -------------------------------------------------------------------------
function kap = kromming(x, y)
xp = gradient(x);  yp = gradient(y);
xpp = gradient(xp); ypp = gradient(yp);
kap = (xp .* ypp - yp .* xpp) ./ max((xp.^2 + yp.^2).^1.5, eps);
end

function [x, y] = afgeronde_veelhoek(H, r)
% Gesloten veelhoek waarvan elke hoek met een cirkelboog van straal r is
% afgerond. Tangentlengte per hoek: t = r * tan(|alpha| / 2).
n = size(H, 1);
x = [];  y = [];
for i = 1:n
    Pm = H(mod(i - 2, n) + 1, :);  Pi = H(i, :);  Pp = H(mod(i, n) + 1, :);
    din  = (Pi - Pm) / norm(Pi - Pm);
    dout = (Pp - Pi) / norm(Pp - Pi);
    alpha = atan2(din(1)*dout(2) - din(2)*dout(1), dot(din, dout));
    t = r * tan(abs(alpha) / 2);
    A = Pi - t * din;                               % begin van de boog
    nrm = sign(alpha) * [-din(2), din(1)];          % naar het middelpunt
    C = A + r * nrm;
    th0 = atan2(A(2) - C(2), A(1) - C(1));
    th  = th0 + linspace(0, alpha, max(3, ceil(abs(alpha) * r / 0.001)));
    x = [x, C(1) + r * cos(th)]; %#ok<AGROW>
    y = [y, C(2) + r * sin(th)]; %#ok<AGROW>
end
x = [x x(1)];  y = [y y(1)];
end

function v = optarg(c, i, standaard)
if numel(c) >= i && ~isempty(c{i}), v = c{i}; else, v = standaard; end
end
