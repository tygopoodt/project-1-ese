function M = auto_3d_model(p)
%AUTO_3D_MODEL  3D-geometrie van de robot-car voor de demo's (niet voor de rekenkern).
%
%   M = auto_3d_model(p)
%   M.body.V/F   carrosserie (STL uit p.auto.stl, anders een blok)
%   M.wiel.V/F   één wiel, as langs y, middelpunt in de oorsprong
%   M.sonar.V/F  één HC-SR04-printje met twee transducers, front in de oorsprong
%   Alle maten in meter, in het autoassenstelsel met z = 0 op de VLOER.
%
%   Wordt het 3D-model verbeterd: exporteer het als STL met dezelfde
%   oriëntatie en pas zo nodig alleen de transformatie hieronder aan
%   (oorsprong midden op de aandrijfas). De simulaties zelf gebruiken alleen
%   p.auto.voor/achter/breedte uit qdat_parameters.

r = p.auto.r_w;
M.body = [];
if exist(p.auto.stl, 'file')
    try
        TR = stlread(p.auto.stl);
        P  = TR.Points;
        % STL: z = lengte (wielkast op 130,4 mm), x = breedte (hart op 72,36 mm),
        % y = hoogte (onderkant op 20 mm). Zelfde transformatie als in
        % RobotCar_LineFollower_Sim.m, maar met z = 0 op de vloer.
        M.body.V = [(P(:, 3) - 130.4) / 1000, -(P(:, 1) - 72.36) / 1000, (P(:, 2) - 20.0) / 1000 - 0.012 + r];
        M.body.F = TR.ConnectivityList;
    catch
        M.body = [];
    end
end
if isempty(M.body)
    A = p.auto;  B = A.breedte / 2;  z0 = 0.020;  z1 = 0.070;
    M.body.V = [-A.achter -B z0; A.voor -B z0; A.voor B z0; -A.achter B z0;
                -A.achter -B z1; A.voor -B z1; A.voor B z1; -A.achter B z1];
    M.body.F = [1 2 3 4; 5 6 7 8; 1 2 6 5; 2 3 7 6; 3 4 8 7; 4 1 5 8];
end

% wiel: cilinder met 24 zijden
n = 24;  th = linspace(0, 2*pi, n + 1);  th(end) = [];
wb = 0.026;
x = r * cos(th);  z = r * sin(th);
M.wiel.V = [x', -wb/2 * ones(n, 1), z'; x', wb/2 * ones(n, 1), z'; 0 -wb/2 0; 0 wb/2 0];
F = zeros(3 * n, 4);
for i = 1:n
    j = mod(i, n) + 1;
    F(i, :) = [i j j + n i + n];
    F(n + i, :) = [2*n + 1 i j 2*n + 1];
    F(2*n + i, :) = [2*n + 2 j + n i + n 2*n + 2];
end
M.wiel.F = F;

% HC-SR04: printje 45 x 20 mm met twee 'ogen'
w = 0.045;  h = 0.020;  d = 0.002;
V = [-d -w/2 -h/2; 0 -w/2 -h/2; 0 w/2 -h/2; -d w/2 -h/2; -d -w/2 h/2; 0 -w/2 h/2; 0 w/2 h/2; -d w/2 h/2];
F = [1 2 3 4; 5 6 7 8; 1 2 6 5; 2 3 7 6; 3 4 8 7; 4 1 5 8];
ne = 12;  te = linspace(0, 2*pi, ne + 1);  te(end) = [];
for yo = [-0.013 0.013]
    b = size(V, 1);
    V = [V; zeros(ne, 1), 0.008 * cos(te') + yo, 0.008 * sin(te'); 0.012 * ones(ne, 1), 0.008 * cos(te') + yo, 0.008 * sin(te')]; %#ok<AGROW>
    for i = 1:ne
        j = mod(i, ne) + 1;
        F(end + 1, :) = [b + i, b + j, b + j + ne, b + i + ne]; %#ok<AGROW>
    end
end
M.sonar.V = V;  M.sonar.F = F;
end
