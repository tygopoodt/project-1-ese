function p = qdat_parameters()
%QDAT_PARAMETERS  Alle vaste gegevens van de Q-Dat robot-car op één plek.
%
%   p = qdat_parameters()
%
%   Wordt het 3D-model of de hardware aangepast, dan verander je het HIER;
%   alle simulaties en onderzoeksscripts lezen deze struct. Bij elke waarde
%   staat waar hij vandaan komt:
%       [STL]   gemeten uit Hardware/3D Files/3d print main body.STL
%       [FA]    Documentatie/Frame-analyse (analyse.py / resultaten.tex)
%       [DB]    datablad
%       [EIS]   Functionele/Technische eisen MoSCoW.csv
%       [AANN]  aanname: nog meten aan de echte auto (zie het rapport, par. 6)
%   Assenstelsel: oorsprong midden op de aandrijfas, x naar voren, y naar links.

%% Auto
p.auto.b       = 0.108;    % spoorbreedte hart band-hart band [m]            [STL]
p.auto.r_w     = 0.032;    % wielstraal [m]                                  [FA]
p.auto.massa   = 0.539;    % totale massa [kg]                               [FA]
p.auto.voor    = 0.066;    % neus voor de aandrijfas [m]                     [STL]
p.auto.achter  = 0.130;    % achterkant achter de aandrijfas [m]             [STL]
p.auto.breedte = 0.143;    % breedte over de spatborden [m]                  [STL]
p.auto.stl     = fullfile(fileparts(mfilename('fullpath')), '..', '..', ...
                          'Hardware', '3D Files', '3d print main body.STL');

%% Motor + L298N (lineair gelijkstroommotormodel)
%   Onbelaste wielsnelheid:   v0    = omega0 * r_w
%   Mechanische tijdconstante: tau_m = J_eq * omega0 / tau_s,  J_eq = m r_w^2 / 2
%   (halve automassa per aangedreven wiel; volgt uit m*dv/dt = F_s (1 - v/v0))
U_ref    = 6.0;                % spanning van de datablad-waarden [V]       [DB]
n0_ref   = 200;                % onbelast toerental bij 6 V [rpm]           [DB]
tau_s6   = 0.070;              % blokkeerkoppel bij 6 V [Nm]                [DB]
p.motor.U_m   = 4.25;          % klemspanning na accu-inzakking en L298N [V] [FA]
omega0        = n0_ref * 2*pi/60 * p.motor.U_m / U_ref;
tau_s         = tau_s6 * p.motor.U_m / U_ref;
p.motor.v0    = omega0 * p.auto.r_w;                               % ~0,47 m/s
p.motor.tau   = (p.auto.massa * p.auto.r_w^2 / 2) * omega0 / tau_s;  % ~0,08 s
p.motor.a_slip = 0.80 * 0.709 * 9.81;  % mu * sigma * g: grip aangedreven wielen [FA]
p.motor.a_rol  = 0.50;         % vertraging bij uitrollen (EN laag) [m/s^2]  [AANN]
p.motor.u_dz   = 0.15;         % dode zone: PWM-fractie waaronder niets draait [AANN]
p.motor.pwm_bits = 8;          % analogWrite 0..255                         [DB]

%% Lijnsensor (4-kanaals tracker, TCRT5000-kopjes)
%   Dekkingsgraad van een Gaussische lichtvlek (breedte sigma) door tape met
%   breedte w, op zijdelingse afstand d van de tapehartlijn:
%       c(d) = 1/2 [ erf((w/2 - d)/(sqrt(2) sigma)) + erf((w/2 + d)/(sqrt(2) sigma)) ]
p.lijn.tape_w  = 0.019;        % zwart PVC-tape [m]            [EIS T9.1, bestellijst]
p.lijn.L       = 0.060;        % sensorbalk voor de aandrijfas [m]  (huidige sim)
p.lijn.y       = [-0.022 -0.007 0.007 0.022];   % sensorposities dwars [m] (huidige sim)
p.lijn.h       = 0.008;        % hoogte boven de vloer [m]                   [FA]
p.lijn.sigma   = 0.003;        % breedte lichtvlek bij h = 8 mm [m]          [AANN]
p.lijn.ruis    = 0.03;         % meetruis, fractie van zwart-wit verschil    [AANN]
p.lijn.drempel = 0.50;         % comparator-drempel (potmeter)               [AANN]
p.lijn.modus   = 'digitaal';   % 'digitaal' (LM393-uitgang) of 'analoog' (ADC)
p.lijn.adc_bits = 10;          % ATmega328P ADC                              [DB]
p.lijn.a_det   = 0.30;         % analoog: minimaal signaal om 'lijn gezien'

%% Lijnvolg-regelaar (PD op de zijdelingse positie van de lijn)
%   e    = zwaartepunt van de actieve sensoren: e = sum(y_i s_i) / sum(s_i)  [m]
%   omega = Kp e + Kd de/dt     (D-term door een 1e-orde filter met tau_d)
%   v    = max(v_min, v_max (1 - k_v |e| / e_bereik))
p.regel.f_s    = 200;          % regellusfrequentie [Hz]
p.regel.Kp     = 60;           % [rad/s per m]
p.regel.Kd     = 1.5;          % [rad/s per m/s]
p.regel.tau_d  = 0.02;         % filter op de D-term [s]
p.regel.v_max  = 0.30;         % [m/s]
p.regel.v_min  = 0.10;         % [m/s]
p.regel.k_v    = 0.6;          % snelheidsreductie bij grote fout [-]
p.regel.w_max  = 8.0;          % begrenzing hoeksnelheid [rad/s]
p.regel.snelheidswet = 'fout'; % 'fout' (bovenstaand) of 'omega' (huidige sim)
p.regel.e_kwijt_factor = 1.22; % lijn kwijt: e = +/- factor * buitenste sensor
p.regel.a_basis = 0.20;        % analoog: basisniveau dat van elk signaal af gaat

%% HC-SR04
%   d = c t_echo / 2,  c = 331,3 + 0,606 T  [m/s]
p.sonar.x        = 0.066;      % sensorfront voor de aandrijfas [m]          [STL]
p.sonar.z        = 0.058;      % hoogte [m]                                  [FA]
p.sonar.beta     = deg2rad(7.5); % halve openingshoek [rad]           [DB, FA]
p.sonar.gamma_max = deg2rad(40); % grootste invalshoek met echo [rad]     [AANN]
p.sonar.sigma    = 0.003;      % ruis [m]                                    [DB]
p.sonar.q        = 0.01;       % afronding: Arduino-code rekent in hele cm
p.sonar.p_uitval = 0.02;       % kans op gemiste echo                        [AANN]
p.sonar.d_min    = 0.02;       % dode zone [m]                               [DB]
p.sonar.d_max    = 4.0;        % bereik [m]                                  [DB]
p.sonar.c        = 343;        % geluidssnelheid bij 20 graden C [m/s]
p.sonar.t_timeout = 0.038;     % echo-pin hoog als er niets terugkomt [s]    [DB]

%% Autonoom rijden (F2), toestandsmachine RIJDEN-REMMEN-KIEZEN-ACHTERUIT
p.obst.f_s       = 100;        % regellus [Hz]
p.obst.T_run     = 120;        % duur van een testrit [s]                     [EIS F2]
p.obst.T_meas    = 0.060;      % tijd tussen twee sonarpulsen [s] (DB: >= 60 ms)
p.obst.N_f       = 3;          % mediaanfilter over N_f metingen
p.obst.opstelling = 'recht';   % 'recht', 'uit2', 'kruis2', 'drie' (sonar_opstelling)
p.obst.toe       = deg2rad(20); % uitdraaihoek van de schuine sensoren [rad]
p.obst.beta_c    = deg2rad(10); % openingshoek waar de REGELAAR mee rekent [rad]
p.obst.marge     = 0.01;       % extra breedte naast de auto die vrij moet zijn [m]
p.obst.v_cruise  = 0.30;       % kruissnelheid [m/s]
p.obst.v_kruip   = 0.06;       % laagste snelheid tijdens het naderen [m/s]
p.obst.d_slow    = 0.30;       % begin afremmen [m]  (F2.1: >= 30 cm vrij = vol door)
p.obst.d_stop    = 0.22;       % stopdrempel [m]     (F2.3: uiterlijk bij 20 cm)
p.obst.d_vrij    = 0.60;       % verder rijden bij >= 60 cm vrij [m]         [EIS F2.4]
p.obst.w_draai   = 2.0;        % draaisnelheid op de plaats [rad/s]
p.obst.phi_min   = deg2rad(15); % draai minstens zo ver voor je weer vertrekt
p.obst.strategie = 'draai';    % 'draai' (draai tot vrij) of 'scan' (kijk links+rechts)
p.obst.phi_scan  = deg2rad(80); % scan-hoek naar elke kant [rad]
p.obst.rem       = 'kortsluit'; % 'kortsluit' (IN1=IN2), 'tegenstroom', 'uitrollen'
p.obst.s_achter  = 0.25;       % achteruit als draaien niets oplevert [m]    [EIS F2.4: <= 30]
p.obst.v_achter  = 0.15;       % [m/s]
p.obst.t_kies_max = 3.5;       % na zoveel s draaien: achteruit (F2.4: 4 s)
p.obst.encoders  = false;      % true: koers uit wielencoders i.p.v. uit de tijd
p.obst.K_koers   = 3.0;        % koershouden bij rechtdoor rijden (alleen met encoders) [1/s]
p.obst.echo_nodig = false;     % KIEZEN: time-out (geen echo) telt niet als vrij
p.obst.phi_extra = 0;          % 'draai': na eerste vrije meting nog zo ver door [rad]
p.obst.scan_min  = deg2rad(10); % 'scan': smalste vrije sector die telt [rad]
p.obst.k_zij     = 0;          % zijwaarts ontwijken met schuine sensoren [rad/s], 0 = uit
p.obst.d_zij     = 0.35;       % vanaf deze afstand stuurt een schuine sensor weg [m]

%% Onzekerheid (domain randomization, zie LEESLIJST A2-A4)
p.var.delta_motor = 0.05;      % verschil motor L/R, +/- fractie
p.var.bat         = [0.85 1.05]; % accuspanning t.o.v. nominaal
p.var.beta        = [7.5 15];    % halve openingshoek HC-SR04 [graden]      [DB, FA]
p.var.gamma       = [30 50];     % grootste invalshoek met echo [graden]    [AANN]

%% Encoders (optie; zie Frame-analyse, odometrie)
p.enc.N  = 20;                 % pulsen per wielomwenteling, schijf op de wielas [FA]
p.enc.ds = 2 * pi * p.auto.r_w / p.enc.N;   % wielweg per puls [m]

%% Doel en eisen
p.doel.e_toel   = 0.030;       % F3.1 zegt 5 cm; 2 cm marge voor de reality gap
p.doel.lambda_e = 5;           % straf per 100 % overschrijding van e_toel
p.doel.lambda_k = 2;           % straf op tijdfractie 'lijn kwijt'
p.doel.lambda_j = 5;           % straf op PWM-gejitter (slijtage, geluid)
p.sim.e_uit     = 0.15;        % verder dan dit van de lijn = run mislukt [m]
end
