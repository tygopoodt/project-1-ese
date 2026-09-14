function pp = obst_zet(p, th, opstelling)
%OBST_ZET  Genormaliseerde parametervector (0..1) naar instellingen voor F2.
%
%   pp = obst_zet(p, th, opstelling)       th: 1 x 13, elk element in [0, 1]
%     1  v_cruise   0,10 .. 0,47 m/s
%     2  d_stop     0,20 .. 0,30 m        (F2.3: uiterlijk bij 20 cm remmen)
%     3  d_slow     d_stop .. 0,30 m      (F2.1: vanaf 30 cm vrij vol door)
%     4  T_meas     60 ms .. 200/n ms     (DB: >= 60 ms; T8.2: >= 5 Hz per sensor)
%     5  N_f        1, 3 of 5             (mediaanfilter)
%     6  w_draai    0,8 .. 4,0 rad/s
%     7  toe        10 .. 50 graden       (niet bij 'recht')
%     8  k_zij      0 .. 5 rad/s          (zijwaarts ontwijken, niet bij 'recht')
%     9  d_zij      0,20 .. 0,60 m
%    10  strategie  'draai' (< 0,5) of 'scan'
%    11  rem        'kortsluit' (< 0,5) of 'tegenstroom'
%    12  echo_nodig time-out telt tijdens kiezen niet als vrij (>= 0,5)
%    13  phi_extra  0 .. 60 graden doordraaien na de eerste vrije meting

pp = p;
o = p.obst;
o.opstelling = opstelling;
n = size(sonar_opstelling(opstelling, 0, p), 1);
o.v_cruise = 0.10 + 0.37 * th(1);
o.d_stop   = 0.20 + 0.10 * th(2);
o.d_slow   = o.d_stop + (0.30 - o.d_stop) * th(3);
T_hi       = max(0.06, 0.20 / n);
o.T_meas   = 0.06 + (T_hi - 0.06) * th(4);
nf = [1 3 5];
o.N_f      = nf(min(3, floor(3 * th(5)) + 1));
o.w_draai  = 0.8 + 3.2 * th(6);
o.toe      = deg2rad(10 + 40 * th(7));
o.k_zij    = 5 * th(8);
o.d_zij    = 0.20 + 0.40 * th(9);
if th(10) < 0.5, o.strategie = 'draai'; else, o.strategie = 'scan'; end
if th(11) < 0.5, o.rem = 'kortsluit'; else, o.rem = 'tegenstroom'; end
o.echo_nodig = th(12) >= 0.5;
o.phi_extra  = deg2rad(60 * th(13));
if strcmp(opstelling, 'recht'), o.k_zij = 0; end
pp.obst = o;
end
