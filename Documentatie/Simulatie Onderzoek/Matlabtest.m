%% 1. SIMULATIE PARAMETERS & 3D CIRCUIT
dt = 0.04;              
total_time = 60;        
steps = total_time / dt;

s = linspace(0, 2*pi, 1500);
track_x = 12 * cos(s);
track_y = 7 * sin(s);
track_z = 1.2 * sin(2*s); % Soepele golving in hoogte

%% 2. AUTO SETUP
x = track_x(1);
y = track_y(1);
z = track_z(1);
theta = atan2(track_y(2)-track_y(1), track_x(2)-track_x(1)); 

v = 1.6;               % Vaste, rustige snelheid
sensor_offset = 0.5;
sensor_spacing = 0.3;

Kp = 4.0; 
Kd = 0.6;
prev_error = 0;

%% 3. FIGURE SETUP (VASTE 3D CAMERA)
figure(1); clf;
set(gcf, 'Color', 'w', 'Position', [100 80 900 650]);

% Baan en grondvlak
plot3(track_x, track_y, track_z, 'k--', 'LineWidth', 2.5); hold on; grid on; axis equal;
fill3([-16 16 16 -16], [-12 -12 12 12], [-2 -2 -2 -2], [0.93 0.93 0.93], 'EdgeColor', 'none');

% 3D Auto Chassis
L = 0.8; W = 0.45; H = 0.25;
[cube_v, cube_f] = get_cube_geometry(L, W, H);
h_car = patch('Faces', cube_f, 'Vertices', cube_v, 'FaceColor', [0 0.45 0.85], ...
              'FaceAlpha', 0.9, 'EdgeColor', 'k');

h_sensors = plot3([x, x], [y, y], [z, z], 'ro', 'MarkerFaceColor', 'r', 'MarkerSize', 6);
h_trail = plot3(x, y, z, 'm-', 'LineWidth', 1.5);

xlim([-15 15]); ylim([-10 10]); zlim([-3 4]);
view(35, 40); % Rustige vaste schuine kijkhoek

trail_x = x; trail_y = y; trail_z = z;

%% 4. SIMULATIE LUS
for t = 1:steps
    % 1. Sensor posities (2D vlak projectie)
    sL_x = x + sensor_offset*cos(theta) - sensor_spacing*sin(theta);
    sL_y = y + sensor_offset*sin(theta) + sensor_spacing*cos(theta);
    sR_x = x + sensor_offset*cos(theta) + sensor_spacing*sin(theta);
    sR_y = y + sensor_offset*sin(theta) - sensor_spacing*cos(theta);
    
    % 2. Zoek kortste afstand tot lijn
    distL = min(hypot(track_x - sL_x, track_y - sL_y));
    distR = min(hypot(track_x - sR_x, track_y - sR_y));
    
    % 3. PD Correctie
    error = distR - distL;
    d_error = (error - prev_error) / dt;
    prev_error = error;
    
    omega = Kp * error + Kd * d_error;
    omega = max(min(omega, 3.0), -3.0);
    
    % 4. Positie update
    theta = theta + omega * dt;
    x = x + v * cos(theta) * dt;
    y = y + v * sin(theta) * dt;
    
    % 5. Hoogte vloeiend meepakken
    [~, idx] = min(hypot(track_x - x, track_y - y));
    z = track_z(idx) + H/2;
    
    % 6. Auto transformeren in 3D
    R = [cos(theta) -sin(theta) 0; sin(theta) cos(theta) 0; 0 0 1];
    trans_verts = (R * cube_v')' + [x, y, z];
    set(h_car, 'Vertices', trans_verts);
    
    % 7. Sensoren & Trail
    set(h_sensors, 'XData', [sL_x, sR_x], 'YData', [sL_y, sR_y], 'ZData', [z, z]);
    trail_x(end+1) = x; trail_y(end+1) = y; trail_z(end+1) = z; %#ok<AGROW>
    set(h_trail, 'XData', trail_x, 'YData', trail_y, 'ZData', trail_z);
    
    drawnow;
    pause(dt);
end

%% HULPFUNCTIE VOOR HET 3D BLOKJE
function [vertices, faces] = get_cube_geometry(l, w, h)
    vertices = [-l/2 -w/2 -h/2;  l/2 -w/2 -h/2;  l/2  w/2 -h/2; -l/2  w/2 -h/2;
                -l/2 -w/2  h/2;  l/2 -w/2  h/2;  l/2  w/2  h/2; -l/2  w/2  h/2];
    faces = [1 2 6 5; 2 3 7 6; 3 4 8 7; 4 1 5 8; 1 2 3 4; 5 6 7 8];
end