function res = sim_lijnvolgen(p, baan, var, bewaar_log)
%SIM_LIJNVOLGEN  Eén ronde lijnvolgen, zonder graphics (snel genoeg voor parfor).
%
%   res = sim_lijnvolgen(p, baan, var)
%   res = sim_lijnvolgen(p, baan, var, true)    ook tijdreeksen in res.log
%
%   Per regelstap (dt = 1/f_s):
%     1. positie van elk sensorkopje:  [x;y] + R(theta) [L; y_i]
%     2. afstand d_i tot de tapehartlijn (dichtstbijzijnde punt van de baan)
%     3. sensormodel: dekkingsgraad c(d_i) + ruis, dan comparator of ADC
%     4. positie van de lijn:  e = sum(y_i s_i) / sum(s_i)
%     5. PD-regelaar -> omega, snelheidswet -> v, stuurprioriteit bij
%        verzadiging, omzetten naar PWM
%     6. motor- en voertuigmodel (aandrijving_stap)
%   Uitvoer: rondetijd, grootste en rms-afwijking van de aandrijfas t.o.v. de
%   lijn (eis F3.1), tijdfractie lijn kwijt, PWM-gejitter.

if nargin < 4, bewaar_log = false; end
r   = p.regel;
dt  = 1 / r.f_s;
n   = numel(baan.x);
ds  = baan.ds;
bx  = baan.x;  by = baan.y;

Ls  = p.lijn.L;
ys  = p.lijn.y(:);
N   = numel(ys);
wt2 = var.lijn.tape_w / 2;
sq  = sqrt(2) * var.lijn.sigma;
gain = var.lijn.gain(:);  off = var.lijn.offset(:);  drem = var.lijn.drempel(:);
analoog = strcmp(p.lijn.modus, 'analoog');
lsb = 1 / (2^p.lijn.adc_bits - 1);

y_buiten = max(abs(ys));
e_kwijt  = r.e_kwijt_factor * y_buiten;
e_bereik = y_buiten + p.lijn.tape_w / 2;
v0   = p.motor.v0;
bnom = p.auto.b;
snelheid_omega = strcmp(r.snelheidswet, 'omega');

rs = RandStream('mt19937ar', 'Seed', var.seed);

th0 = atan2(by(3) - by(1), bx(3) - bx(1));
st  = [bx(1) by(1) th0 0 0];
ic  = 1;
voortgang = 0;
t_max = baan.L / 0.08 + 5;
nstap = ceil(t_max / dt);
venster_as = -25:100;
venster_s  = -60:round((Ls + 0.12) / ds);

e_prev = 0;  D = 0;  laatst = 1;  u_prev = [0 0];  ehat = 0;
e_max = 0;  e2 = 0;  n_kwijt = 0;  jit = 0;
voltooid = false;

if bewaar_log
    lg.t = zeros(nstap, 1); lg.x = lg.t; lg.y = lg.t; lg.th = lg.t;
    lg.e = lg.t; lg.ehat = lg.t; lg.v = lg.t; lg.w = lg.t;
    lg.uL = lg.t; lg.uR = lg.t; lg.kwijt = false(nstap, 1);
    lg.s = zeros(nstap, N);
end

for k = 1:nstap
    c = cos(st(3));  s = sin(st(3));
    xs = st(1) + Ls * c - ys * s;
    yw = st(2) + Ls * s + ys * c;
    idx = mod(ic - 1 + venster_s, n) + 1;
    d = sqrt(min((xs - bx(idx)).^2 + (yw - by(idx)).^2, [], 2));

    % sensormodel: dekkingsgraad van de lichtvlek door de tape
    a = gain .* 0.5 .* (erf((wt2 - d) / sq) + erf((wt2 + d) / sq)) ...
        + off + p.lijn.ruis * rs.randn(N, 1);
    if analoog
        a = round(min(max(a, 0), 1) / lsb) * lsb;
        g = max(a - r.a_basis, 0);
        zicht = max(a) > p.lijn.a_det && sum(g) > 0;
        if zicht, ehat = sum(ys .* g) / sum(g); end
        sig_log = a;
    else
        bits = a > drem;
        zicht = any(bits);
        if zicht, ehat = sum(ys(bits)) / nnz(bits); end
        sig_log = bits;
    end
    if zicht
        if ehat ~= 0, laatst = sign(ehat); end
    else
        ehat = laatst * e_kwijt;
        n_kwijt = n_kwijt + 1;
    end

    % PD-regelaar met gefilterde D-term
    de = (ehat - e_prev) / dt;
    e_prev = ehat;
    if r.tau_d > 0
        D = D + (de - D) * dt / (r.tau_d + dt);
    else
        D = de;
    end
    w_cmd = r.Kp * ehat + r.Kd * D;
    w_cmd = min(max(w_cmd, -r.w_max), r.w_max);

    if snelheid_omega
        v_cmd = max(r.v_min, r.v_max / (1 + r.k_v * abs(w_cmd)));
    else
        v_cmd = max(r.v_min, r.v_max * (1 - r.k_v * abs(ehat) / e_bereik));
    end
    if ~zicht, v_cmd = r.v_min; end

    % stuurprioriteit: sturen gaat voor snelheid als een wiel verzadigt
    half = w_cmd * bnom / 2;
    if abs(half) > v0
        half = sign(half) * v0;  v_cmd = 0;
    else
        v_cmd = min(v_cmd, v0 - abs(half));
    end
    u = pwm_omzetten([v_cmd - half, v_cmd + half] / v0, p.motor.u_dz, p.motor.pwm_bits);

    st = aandrijving_stap(st, u, [false false], var, p, dt);

    % afwijking van de aandrijfas en voortgang langs de baan
    idx = mod(ic - 1 + venster_as, n) + 1;
    [d2, j] = min((bx(idx) - st(1)).^2 + (by(idx) - st(2)).^2);
    voortgang = voortgang + venster_as(j) * ds;
    ic = idx(j);
    e_as = sqrt(d2);

    e_max = max(e_max, e_as);
    e2  = e2 + d2;
    jit = jit + abs(u(1) - u_prev(1)) + abs(u(2) - u_prev(2));
    u_prev = u;

    if bewaar_log
        i2 = mod(ic, n) + 1;
        tx = bx(i2) - bx(ic);  ty = by(i2) - by(ic);
        lg.t(k) = k * dt;  lg.x(k) = st(1);  lg.y(k) = st(2);  lg.th(k) = st(3);
        lg.e(k) = e_as * sign(tx * (st(2) - by(ic)) - ty * (st(1) - bx(ic)));
        lg.ehat(k) = ehat;  lg.v(k) = (st(4) + st(5)) / 2;
        lg.w(k) = (st(5) - st(4)) / var.b;  lg.uL(k) = u(1);  lg.uR(k) = u(2);
        lg.kwijt(k) = ~zicht;  lg.s(k, :) = sig_log(:)';
    end

    if e_as > p.sim.e_uit || voortgang < -0.2, break; end    % van de lijn of omgekeerd
    if voortgang >= baan.L, voltooid = true; break; end
end

res.voltooid   = voltooid;
res.T          = k * dt;
res.voortgang  = min(voortgang / baan.L, 1);
res.e_max      = e_max;
res.e_rms      = sqrt(e2 / k);
res.kwijt_frac = n_kwijt / k;
res.jitter     = jit / (2 * k);
res.v_gem      = voortgang / (k * dt);
if bewaar_log
    f = fieldnames(lg);
    for i = 1:numel(f), lg.(f{i}) = lg.(f{i})(1:k, :); end
    res.log = lg;
end
end
