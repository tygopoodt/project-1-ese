%% =========================================================================
%  Q-DAT SYSTEMS - PROFESSIONELE 3D ROBOT-CAR SIMULATIE (4-KANAALS LIJNVOLGER)
%  Project 1 ESE - Hogeschool van Arnhem en Nijmegen (HAN)
%
%  Specificaties (conform Frame-Analyse & Werkelijke STL Geometrie):
%  - 2 Aangedreven wielen EXACT parallel in de fenders van jullie 3D-body
%  - 1 Zwenkwiel (castor) achter onder het achterdek
%  - Dunne zwarte tape (19 mm) op een complex en realistisch testparcours
%  - Laadt automatisch de werkelijke '3d print main body.STL' van jullie auto
%  - HC-SR04 ultrasoonmodule voorop op 55 mm hoogte met 3D zichtkegel
%  - 4x Infrarood lijnsensoren met actieve status-LEDs onder de voorbumper (8 mm)
%  - Vloeiende bocht-adaptieve PD-regeling (volgt superstrak zonder oscilleren)
%  - TOUCHPAD & LAPTOP BEDIENING:
%    * Knoppen op het scherm voor Inzoomen, Uitzoomen, Draaien, Camera & Pauze
%    * Toetsenbord: [+] Inzoomen, [-] Uitzoomen, [C] Camera mode, [Spatie] Pauze
% =========================================================================
clear; clc; close all;

%% 1. SIMULATIE & CAMERA INSTELLINGEN
global sim_cfg;
sim_cfg.follow_camera = false;  % true = chase-cam, false = 3D vogelvlucht
sim_cfg.is_paused = false;
sim_cfg.zoom_factor = 1.0;
sim_cfg.target_cam_dist = 0.85;

dt = 0.02;                      % Tijdstap (50 Hz)
total_time = 80.0;              % Maximale simulatieduur (seconden)
steps = round(total_time / dt);

% 2. VOERTUIG PARAMETERS (EXACT GEMETEN UIT DE STL & ANALYSE.PY)
% De spatborden (fenders) in jullie STL zitten op Z = 130.4 mm en X = 18 mm & 126.7 mm
b = 0.108;                      % Spoorbreedte h.o.h. wielen in de fenders (108 mm)
wheel_dia = 0.064;              % Diameter hoofdwielen (64 mm, past in de 78 mm wielkast)
wheel_r = wheel_dia / 2;        % Wielradius (32 mm = ashoogte boven de vloer)
wheel_w = 0.026;                % Bandbreedte (26 mm, past in de 30 mm fender opening)

% Zwenkwiel (castor) achterzijde
caster_x = -0.085;              % 85 mm achter de aandrijfas onder het achterdek
caster_r = 0.009;               % Straal vrijloopwieltje (9 mm)
caster_w = 0.010;               % Breedte zwenkwieltje (10 mm)

% Dunne zwarte tape
tape_w = 0.019;                 % Standaard zwarte PVC isolatietape: 19 mm breed

% 4-Kanaals Lijnsensor positie (onder de voorste bumperbeugel op z = 8 mm)
x_sens = 0.060;                 % 60 mm vóór de aandrijfas
y_sens = [-0.022, -0.007, 0.007, 0.022]; % [S1(links-buit), S2(links-bin), S3(rechts-bin), S4(rechts-buit)]
z_sens = 0.008;                 % 8 mm boven de vloer

% Regelaar parameters (Vloeiende PD met dynamische bochtsnelheid)
v_max = 0.35;                   % Topsnelheid op rechte stukken (0.35 m/s)
v_min = 0.14;                   % Minimumsnelheid in haarspeldbochten
Kp = 28.0;                      % Proportionele actie
Kd = 1.8;                       % Differentiële demping
prev_err = 0.0;
last_dir = 1;                   % Richtinggeheugen bij lijnverlies

%% 3. COMPLEX TESTPARCOURS (30 CONTROLEPUNTEN)
P = [
     0.00,  0.00;   % Start / Finish
     1.10,  0.05;   % Acceleratiestrook
     2.10,  0.30;
     2.90,  0.90;   % Rechter doordraaier
     3.30,  1.90;
     2.90,  2.90;   % Bocht 2
     2.00,  3.20;
     1.00,  2.70;
     0.40,  1.90;   % Chicane ingang links
    -0.10,  1.30;   % Chicane knik rechts
    -0.70,  1.10;
    -1.50,  1.50;   % Snelle S-bocht
    -1.90,  2.50;
    -2.60,  3.20;   % Noordelijke haarspeldbocht
    -3.30,  2.70;
    -3.50,  1.60;
    -2.90,  0.60;   % Technische afdaling
    -2.10,  0.00;
    -2.70, -0.90;   % Scherpere knik
    -3.30, -1.90;   % Zuidwestelijke lus
    -2.80, -2.80;
    -1.60, -2.70;   % Brede bocht onderin
    -0.80, -1.90;   % Korte knik
     0.20, -2.30;
     1.30, -2.60;   % Zuidoostelijke lus
     2.30, -2.20;
     2.80, -1.30;
     2.30, -0.60;   % Aansnijden laatste bocht
     1.20, -0.25;   % Opstart rechte stuk
     0.00,  0.00    % Terug bij Start
];

num_pts = 5000;
t_ctrl = linspace(0, 1, size(P, 1));
t_fine = linspace(0, 1, num_pts);
track_x = spline(t_ctrl, P(:, 1)', t_fine);
track_y = spline(t_ctrl, P(:, 2)', t_fine);

% Bereken 3D-ribbon geometrie voor de dunne zwarte tape
dx_tr = gradient(track_x);
dy_tr = gradient(track_y);
norm_len = hypot(dx_tr, dy_tr);
nx = -dy_tr ./ norm_len;
ny =  dx_tr ./ norm_len;

tape_left  = [track_x + nx * (tape_w/2); track_y + ny * (tape_w/2); repmat(0.001, 1, num_pts)];
tape_right = [track_x - nx * (tape_w/2); track_y - ny * (tape_w/2); repmat(0.001, 1, num_pts)];

%% 4. STARTPOSITIE AUTO (OP DE AANDRIJFAS)
x = track_x(1);
y = track_y(1);
z = wheel_r; % Aandrijfas zit op wielhoogte (32 mm boven de grond)
dx0 = track_x(2) - track_x(1);
dy0 = track_y(2) - track_y(1);
theta = atan2(dy0, dx0);

%% 5. 3D SCÈNE & VISUALISATIE SETUP
h_fig = figure('Name', 'Q-Dat Systems - Professionele 3D Simulatie (2 Wielen in Fenders + Caster)', ...
               'Color', [0.12 0.13 0.16], 'Position', [40 40 1260 800]);
hold on; axis equal; grid on;
ax = gca;
ax.Color = [0.92 0.93 0.95]; % Linoleum tegelvloer
ax.GridColor = [0.70 0.74 0.80];
ax.GridAlpha = 0.55;
xlabel('X [m]', 'Color', 'w'); ylabel('Y [m]', 'Color', 'w'); zlabel('Z [m]', 'Color', 'w');
set(ax, 'XColor', [0.6 0.6 0.6], 'YColor', [0.6 0.6 0.6], 'ZColor', [0.6 0.6 0.6]);
title('Q-Dat Robot-Car 3D Lijnvolg-Simulatie (2WD in Fenders)', ...
      'Color', 'w', 'FontSize', 13, 'FontWeight', 'bold');

% Vloeroppervlak (7x6 meter ruimte)
floor_x = [-4.2 4.2 4.2 -4.2];
floor_y = [-3.8 -3.8 4.2 4.2];
patch('XData', floor_x, 'YData', floor_y, 'ZData', [0 0 0 0], ...
      'FaceColor', [0.91 0.92 0.94], 'EdgeColor', 'none', 'FaceAlpha', 1.0);

% Teken de zwarte isolatietape als een echte 19 mm 3D-strook
tape_faces = zeros(num_pts-1, 4);
tape_verts = zeros((num_pts-1)*2, 3);
for k = 1:num_pts-1
    tape_faces(k, :) = [2*k-1, 2*k, 2*k+2, 2*k+1];
end
tape_verts(1:2:end, :) = tape_left(:, 1:num_pts-1)';
tape_verts(2:2:end, :) = tape_right(:, 1:num_pts-1)';
patch('Faces', tape_faces, 'Vertices', tape_verts, ...
      'FaceColor', [0.10 0.10 0.12], 'EdgeColor', 'none', 'FaceAlpha', 0.98);

% Dunne witte referentie-hartlijn op de tape
plot3(track_x, track_y, repmat(0.0015, 1, num_pts), 'w:', 'LineWidth', 0.5);

% Afgelegde rijlijn (trail)
h_trail = plot3(x, y, 0.002, 'c-', 'LineWidth', 1.3);
trail_x = x; trail_y = y;

%% 6. LAAD EN POSITIONEER 3D CHASSIS (EXACT GECENTREERD OP DE FENDERS!)
stl_path = fullfile(fileparts(mfilename('fullpath')), '..', '..', 'Hardware', '3D Files', '3d print main body.STL');
loaded_stl = false;
if exist(stl_path, 'file')
    try
        TR = stlread(stl_path);
        v_stl = TR.Points;
        % Exacte transformatie zodat de wielkasten van de STL gecentreerd zijn op de wielas (x = 0):
        % Het hart van de wielkast in de STL ligt exact op Z = 130.4 mm!
        % De breedtehartlijn van de carrosserie ligt op X = 72.36 mm.
        % De onderkant van de carrosserie ligt op Y = 20.0 mm (bodemvrijheid 20 mm).
        body_v = [(v_stl(:,3) - 130.4)/1000, -(v_stl(:,1) - 72.36)/1000, (v_stl(:,2) - 20.0)/1000 - 0.012];
        body_f = TR.ConnectivityList;
        loaded_stl = true;
    catch
        loaded_stl = false;
    end
end

if ~loaded_stl
    [body_v, body_f] = generate_fallback_chassis();
end

h_chassis = patch('Faces', body_f, 'Vertices', body_v, ...
                  'FaceColor', [0.18 0.52 0.88], 'EdgeColor', 'none', ...
                  'FaceAlpha', 0.95, 'SpecularStrength', 0.4);

% 2x Grote Aangedreven Wielen (Geel/Zwart, EXACT parallel binnen de fenders)
[wheel_v, wheel_f] = generate_3d_wheel(wheel_dia, wheel_w);
h_wL = patch('Faces', wheel_f, 'Vertices', wheel_v, ...
             'FaceColor', [0.15 0.15 0.15], 'EdgeColor', [0.9 0.7 0.1], ...
             'LineWidth', 0.8, 'SpecularStrength', 0.3);
h_wR = patch('Faces', wheel_f, 'Vertices', wheel_v, ...
             'FaceColor', [0.15 0.15 0.15], 'EdgeColor', [0.9 0.7 0.1], ...
             'LineWidth', 0.8, 'SpecularStrength', 0.3);

% 1x 3D Zwenkwiel (Caster) aan de achterzijde
[caster_v, caster_f] = generate_3d_wheel(caster_r*2, caster_w);
h_caster = patch('Faces', caster_f, 'Vertices', caster_v, ...
                 'FaceColor', [0.3 0.3 0.35], 'EdgeColor', [0.6 0.6 0.6], 'LineWidth', 0.5);

% Caster montagebeugeltje
[c_bracket_v, c_bracket_f] = generate_caster_bracket(caster_x, caster_r, wheel_r);
h_cbracket = patch('Faces', c_bracket_f, 'Vertices', c_bracket_v, ...
                   'FaceColor', [0.65 0.65 0.70], 'EdgeColor', 'k', 'LineWidth', 0.5);

% 3D HC-SR04 Ultrasoon Sensor op de voorbumper
[sonar_v, sonar_f] = generate_3d_hcsr04();
h_sonar_model = patch('Faces', sonar_f, 'Vertices', sonar_v, ...
                      'FaceColor', [0.1 0.35 0.7], 'EdgeColor', [0.7 0.7 0.7], 'LineWidth', 0.5);

% Ultrasone 3D Zichtkegel
[cone_v, cone_f] = generate_sonar_cone(0.40, deg2rad(24));
h_sonar_cone = patch('Faces', cone_f, 'Vertices', cone_v, ...
                     'FaceColor', [1 0.85 0.1], 'FaceAlpha', 0.15, 'EdgeColor', 'none');

% 4x Lijnsensoren met actieve status-LEDs onder de neus
h_leds = gobjects(4, 1);
for si = 1:4
    h_leds(si) = plot3(0, 0, 0, 'o', 'MarkerSize', 7, ...
                       'MarkerFaceColor', [0.1 0.8 0.2], 'MarkerEdgeColor', 'k');
end

% Verlichting & Standaard Camera
camlight('headlight');
lighting gouraud;
material dull;
view(38, 44);
xlim([-4.0 3.8]); ylim([-3.5 3.8]); zlim([0 1.5]);

% Schakel MATLAB's ingebouwde camera-werkbalk in
cameratoolbar(h_fig, 'Show');
cameratoolbar(h_fig, 'SetMode', 'orbit');

%% 7. INTERACTIEVE ON-SCREEN KNOPPEN VOOR ZOOMEN ZONDER MUIS
btn_bg = [0.22 0.25 0.30];
btn_fg = [1.0 1.0 1.0];

% Zoom In knop
uicontrol('Parent', h_fig, 'Style', 'pushbutton', 'String', '🔍 Zoom In (+)', ...
    'Units', 'normalized', 'Position', [0.02 0.03 0.08 0.045], ...
    'BackgroundColor', btn_bg, 'ForegroundColor', btn_fg, 'FontWeight', 'bold', ...
    'Tooltip', 'Zoom dichterbij (of druk op +)', ...
    'Callback', @(~,~) cb_zoom_in());

% Zoom Out knop
uicontrol('Parent', h_fig, 'Style', 'pushbutton', 'String', '🔍 Zoom Out (-)', ...
    'Units', 'normalized', 'Position', [0.105 0.03 0.08 0.045], ...
    'BackgroundColor', btn_bg, 'ForegroundColor', btn_fg, 'FontWeight', 'bold', ...
    'Tooltip', 'Zoom verder weg (of druk op -)', ...
    'Callback', @(~,~) cb_zoom_out());

% Camera wissel knop (Vogelvlucht / Achter de auto)
uicontrol('Parent', h_fig, 'Style', 'pushbutton', 'String', '🎥 Volg Auto (C)', ...
    'Units', 'normalized', 'Position', [0.19 0.03 0.09 0.045], ...
    'BackgroundColor', [0.15 0.45 0.75], 'ForegroundColor', btn_fg, 'FontWeight', 'bold', ...
    'Tooltip', 'Wissel tussen overzicht en chase-cam (of druk op C)', ...
    'Callback', @(~,~) cb_toggle_cam());

% Roteer Links / Rechts knoppen
uicontrol('Parent', h_fig, 'Style', 'pushbutton', 'String', '↺ Draai Links', ...
    'Units', 'normalized', 'Position', [0.285 0.03 0.07 0.045], ...
    'BackgroundColor', btn_bg, 'ForegroundColor', btn_fg, ...
    'Callback', @(~,~) camorbit(-15, 0));

uicontrol('Parent', h_fig, 'Style', 'pushbutton', 'String', '↻ Draai Rechts', ...
    'Units', 'normalized', 'Position', [0.36 0.03 0.07 0.045], ...
    'BackgroundColor', btn_bg, 'ForegroundColor', btn_fg, ...
    'Callback', @(~,~) camorbit(15, 0));

% Reset Camera knop
uicontrol('Parent', h_fig, 'Style', 'pushbutton', 'String', '⟲ Reset Camera (R)', ...
    'Units', 'normalized', 'Position', [0.435 0.03 0.09 0.045], ...
    'BackgroundColor', btn_bg, 'ForegroundColor', btn_fg, ...
    'Callback', @(~,~) cb_reset_cam());

% Pauze / Hervat knop
uicontrol('Parent', h_fig, 'Style', 'pushbutton', 'String', '⏸ Pauze (Spatie)', ...
    'Units', 'normalized', 'Position', [0.53 0.03 0.08 0.045], ...
    'BackgroundColor', [0.75 0.35 0.15], 'ForegroundColor', btn_fg, 'FontWeight', 'bold', ...
    'Callback', @(~,~) cb_toggle_pause());

% Koppel toetsenbord sneltoetsen en touchpad scroll aan het venster
set(h_fig, 'WindowKeyPressFcn', @cb_key_press);
set(h_fig, 'WindowScrollWheelFcn', @(~, e) camzoom(1.0 - 0.15 * e.VerticalScrollCount));

% Live Telemetrie HUD Box (Dark Theme)
hud_box = annotation('textbox', [0.02, 0.65, 0.27, 0.32], ...
    'String', '', 'FontName', 'Consolas', 'FontSize', 9.5, ...
    'Color', [0.95 0.95 0.95], 'BackgroundColor', [0.08 0.10 0.14], ...
    'EdgeColor', [0.25 0.55 0.95], 'LineWidth', 1.5, 'FitBoxToText', 'on');

%% 8. HOOFDSIMULATIE LUS (50 Hz)
tic;
step = 1;
while step <= steps
    if ~ishandle(h_fig), break; end
    
    % Als de pauzeknop is ingedrukt, wacht even en luister naar de knoppen
    if sim_cfg.is_paused
        pause(0.05);
        drawnow;
        continue;
    end
    
    curr_time = (step - 1) * dt;
    
    % ---------------------------------------------------------------------
    % A. ROTATIEMATRIX & POSITIE (DRAAIPUNT = AANDRIJFAS OP x, y)
    % ---------------------------------------------------------------------
    R = [cos(theta), -sin(theta), 0;
         sin(theta),  cos(theta), 0;
                  0,           0, 1];
    axle_center = [x; y; z];
    
    % ---------------------------------------------------------------------
    % B. 4-KANAALS LIJNSENSOREN UITLEZEN (DIGITAAL 0 OF 1)
    % ---------------------------------------------------------------------
    sensor_bits = zeros(1, 4);
    sens_world_pos = zeros(4, 3);
    
    for si = 1:4
        p_loc = [x_sens; y_sens(si); z_sens - wheel_r];
        p_world = axle_center + R * p_loc;
        sens_world_pos(si, :) = p_world';
        
        % Afstand van dit sensorkopje tot de dichtstbijzijnde hartlijn van de tape
        d_center = min(hypot(track_x - p_world(1), track_y - p_world(2)));
        
        % Detectie: binnen de halve breedte van de tape (19 mm / 2 = 9.5 mm)
        if d_center <= (tape_w / 2)
            sensor_bits(si) = 1; % Zwart tape!
        else
            sensor_bits(si) = 0; % Wit linoleum
        end
    end
    
    % ---------------------------------------------------------------------
    % C. CENTROID PD-LIJNVOLGER MET SNELHEIDSMODULATIE
    % ---------------------------------------------------------------------
    weights = [-1.8, -0.6, 0.6, 1.8];
    num_active = sum(sensor_bits);
    
    if num_active > 0
        % Gewogen centrum van de gedetecteerde lijn
        pos_error = sum(weights .* sensor_bits) / num_active;
        last_dir = sign(pos_error);
        if last_dir == 0, last_dir = 1; end
    else
        % Lijn even kwijt -> agressief insturen in laatst bekende richting
        pos_error = last_dir * 2.2;
    end
    
    % PD-formule voor draaisnelheid (omega)
    d_error = (pos_error - prev_err) / dt;
    prev_err = pos_error;
    
    omega = - (Kp * pos_error + Kd * d_error);
    omega = max(min(omega, 4.8), -4.8); % Begrens op fysieke motorcapaciteit
    
    % Dynamische snelheid: gas terugnemen in scherpe bochten, vlot op rechte stukken
    v = v_max / (1.0 + 0.9 * abs(omega));
    v = max(v, v_min);
    
    % Bereken wielsnelheden van de 2 aangedreven motoren
    v_L = v - (omega * b) / 2;
    v_R = v + (omega * b) / 2;
    
    % ---------------------------------------------------------------------
    % D. DIFFERENTIËLE VOERTUIGKINEMATICA (DRAAIT OM HOOFDAS)
    % ---------------------------------------------------------------------
    theta = theta + omega * dt;
    x = x + v * cos(theta) * dt;
    y = y + v * sin(theta) * dt;
    
    % ---------------------------------------------------------------------
    % E. 3D VISUALISATIE UPDATEN
    % ---------------------------------------------------------------------
    % 1. Chassis (gecentreerd op wielas)
    cur_chassis_v = (R * body_v')' + axle_center';
    set(h_chassis, 'Vertices', cur_chassis_v);
    
    % 2. De 2 Hoofdwielen (EXACT PARALLEL IN DE FENDERS: x = 0, y = ± b/2)
    wL_offset = R * [0; -b/2; 0];
    wR_offset = R * [0;  b/2; 0];
    set(h_wL, 'Vertices', (R * wheel_v')' + (axle_center + wL_offset)');
    set(h_wR, 'Vertices', (R * wheel_v')' + (axle_center + wR_offset)');
    
    % 3. Zwenkwiel achter (caster op x = -85 mm, z = 9 mm boven vloer)
    caster_offset = R * [caster_x; 0; caster_r - wheel_r];
    set(h_caster, 'Vertices', (R * caster_v')' + (axle_center + caster_offset)');
    set(h_cbracket, 'Vertices', (R * c_bracket_v')' + axle_center');
    
    % 4. HC-SR04 Ultrasoon Sensor op de voorbumper
    sonar_mount = R * [0.066; 0; 0.055 - wheel_r];
    set(h_sonar_model, 'Vertices', (R * sonar_v')' + (axle_center + sonar_mount)');
    set(h_sonar_cone,  'Vertices', (R * cone_v')'  + (axle_center + sonar_mount)');
    
    % 5. 4 Lijnsensoren met actieve LEDs
    for si = 1:4
        set(h_leds(si), 'XData', sens_world_pos(si, 1), ...
                        'YData', sens_world_pos(si, 2), ...
                        'ZData', sens_world_pos(si, 3));
        if sensor_bits(si) == 1
            set(h_leds(si), 'MarkerFaceColor', [1.0 0.1 0.1], 'MarkerSize', 8); % ROOD = tape!
        else
            set(h_leds(si), 'MarkerFaceColor', [0.1 0.8 0.2], 'MarkerSize', 5); % GROEN = vloer
        end
    end
    
    % 6. Spoorlijn
    trail_x(end+1) = x; trail_y(end+1) = y; %#ok<AGROW>
    set(h_trail, 'XData', trail_x, 'YData', trail_y);
    
    % 7. Camera Tracking
    if sim_cfg.follow_camera
        cam_dist = sim_cfg.target_cam_dist;
        cam_h = 0.55 * (cam_dist / 0.85);
        c_pos = [x - cam_dist*cos(theta), y - cam_dist*sin(theta), cam_h];
        c_targ = [x + 0.3*cos(theta), y + 0.3*sin(theta), 0.04];
        campos(c_pos);
        camtarget(c_targ);
    end
    
    % 8. Live HUD
    bit_str = sprintf('[%d %d %d %d]', sensor_bits(1), sensor_bits(2), sensor_bits(3), sensor_bits(4));
    hud_msg = sprintf([ ...
        '\\bf\\color[rgb]{0.3,0.7,1.0}Q-DAT ROBOT-CAR 2WD TELEMETRIE\\rm\n' ...
        '---------------------------------\n' ...
        'Tijd:        %5.2f s\n' ...
        'Sensoren:    \\bf%s\\rm\n' ...
        'Snelheid:    %5.2f m/s  (%4.1f km/h)\n' ...
        'Hoeksnelh.:  %5.2f rad/s\n' ...
        'Motor Links: %5.2f m/s\n' ...
        'Motor Rechts:%5.2f m/s\n' ...
        'Status:      %s\n' ...
        '---------------------------------\n' ...
        '\\color[rgb]{0.7,0.7,0.7}Toetsen: [+] Inzoom [-] Uitzoom\n' ...
        '[C] Volg Auto  [Spatie] Pauze\\rm'], ...
        curr_time, bit_str, v, v*3.6, omega, v_L, v_R, ...
        iif(num_active > 0, '\\color{green}ON TRACK', '\\color{red}SEARCHING'));
    set(hud_box, 'String', hud_msg);
    
    drawnow limitrate;
    pause(0.006);
    step = step + 1;
end

%% =========================================================================
%  CALLBACKS VOOR CAMERA EN ZOOMEN ZONDER MUIS
% =========================================================================

function cb_zoom_in()
    global sim_cfg;
    if sim_cfg.follow_camera
        sim_cfg.target_cam_dist = max(0.35, sim_cfg.target_cam_dist * 0.80);
    else
        camzoom(1.25);
    end
end

function cb_zoom_out()
    global sim_cfg;
    if sim_cfg.follow_camera
        sim_cfg.target_cam_dist = min(3.50, sim_cfg.target_cam_dist * 1.25);
    else
        camzoom(0.80);
    end
end

function cb_toggle_cam()
    global sim_cfg;
    sim_cfg.follow_camera = ~sim_cfg.follow_camera;
    if ~sim_cfg.follow_camera
        cb_reset_cam();
    end
end

function cb_reset_cam()
    global sim_cfg;
    sim_cfg.target_cam_dist = 0.85;
    view(38, 44);
    xlim([-4.0 3.8]); ylim([-3.5 3.8]); zlim([0 1.5]);
    camzoom('reset');
end

function cb_toggle_pause()
    global sim_cfg;
    sim_cfg.is_paused = ~sim_cfg.is_paused;
end

function cb_key_press(~, event)
    switch event.Key
        case {'add', 'equal', 'plus'}
            cb_zoom_in();
        case {'subtract', 'hyphen', 'minus'}
            cb_zoom_out();
        case 'c'
            cb_toggle_cam();
        case 'r'
            cb_reset_cam();
        case 'space'
            cb_toggle_pause();
        case 'leftarrow'
            camorbit(-10, 0);
        case 'rightarrow'
            camorbit(10, 0);
        case 'uparrow'
            camorbit(0, 8);
        case 'downarrow'
            camorbit(0, -8);
    end
end

%% =========================================================================
%  HULPFUNCTIES VOOR 3D GEOMETRIE
% =========================================================================

function [V, F] = generate_fallback_chassis()
    L_f = 0.066; L_r = -0.130; W = 0.144; H = 0.035;
    v0 = [L_r -W/2 0; L_f -W/2 0; L_f W/2 0; L_r W/2 0;
          L_r -W/2 H; L_f -W/2 H; L_f W/2 H; L_r W/2 H];
    f0 = [1 2 3 4; 5 6 7 8; 1 2 6 5; 2 3 7 6; 3 4 8 7; 4 1 5 8];
    V = v0; F = f0;
end

function [V, F] = generate_3d_wheel(dia, width)
    n = 16;
    r = dia / 2;
    th = linspace(0, 2*pi, n+1);
    th(end) = [];
    x = r * cos(th);
    z = r * sin(th);
    
    V = [x', repmat(-width/2, n, 1), z';
         x', repmat( width/2, n, 1), z';
         0,  -width/2,               0;
         0,   width/2,               0];
    c1 = 2*n + 1;
    c2 = 2*n + 2;
    
    F = zeros(3*n, 4);
    for i = 1:n
        next = mod(i, n) + 1;
        F(i, :)     = [i, next, next+n, i+n];
        F(n+i, :)   = [c1, i, next, c1];
        F(2*n+i, :) = [c2, next+n, i+n, c2];
    end
end

function [V, F] = generate_caster_bracket(cx, cr, wr)
    h_top = 0.012;
    h_bot = cr - wr;
    w_b = 0.014;
    l_b = 0.020;
    V = [cx-l_b/2 -w_b/2 h_bot; cx+l_b/2 -w_b/2 h_bot; cx+l_b/2 w_b/2 h_bot; cx-l_b/2 w_b/2 h_bot;
         cx-l_b/2 -w_b/2 h_top; cx+l_b/2 -w_b/2 h_top; cx+l_b/2 w_b/2 h_top; cx-l_b/2 w_b/2 h_top];
    F = [1 2 3 4; 5 6 7 8; 1 2 6 5; 2 3 7 6; 3 4 8 7; 4 1 5 8];
end

function [V, F] = generate_3d_hcsr04()
    w = 0.045; h = 0.020; d = 0.003;
    V = [-d -w/2 -h/2; 0 -w/2 -h/2; 0 w/2 -h/2; -d w/2 -h/2;
         -d -w/2  h/2; 0 -w/2  h/2; 0 w/2  h/2; -d w/2  h/2];
    F = [1 2 3 4; 5 6 7 8; 1 2 6 5; 2 3 7 6; 3 4 8 7; 4 1 5 8];
    
    eye_r = 0.0075; eye_len = 0.012;
    ne = 10;
    th = linspace(0, 2*pi, ne+1); th(end) = [];
    ey = eye_r * cos(th); ez = eye_r * sin(th);
    for side = [-0.013, 0.013]
        v_idx = size(V, 1);
        V_eye = [repmat(0, ne, 1),       ey' + side, ez';
                 repmat(eye_len, ne, 1), ey' + side, ez'];
        V = [V; V_eye]; %#ok<AGROW>
        for i = 1:ne
            nxt = mod(i, ne) + 1;
            F(end+1, :) = [v_idx + i, v_idx + nxt, v_idx + nxt + ne, v_idx + i + ne]; %#ok<AGROW>
        end
    end
end

function [V, F] = generate_sonar_cone(range, fov)
    n = 12;
    th = linspace(-fov/2, fov/2, n);
    x = range * cos(th);
    y = range * sin(th);
    V = [0 0 0; [x' y' repmat(0, n, 1)]];
    F = zeros(n-1, 4);
    for i = 1:n-1
        F(i, :) = [1, i+1, i+2, 1];
    end
end

function res = iif(cond, val_true, val_false)
    if cond, res = val_true; else, res = val_false; end
end
