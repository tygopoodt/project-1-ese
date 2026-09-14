function var = variatie_trekken(p, rs)
%VARIATIE_TREKKEN  Eén set 'echte-wereld'-afwijkingen voor één simulatierun.
%
%   var = variatie_trekken(p)       nominaal: alles precies zoals in p
%   var = variatie_trekken(p, rs)   willekeurig getrokken met RandStream rs
%
%   Idee (Jakobi 1995/1997, Tobin 2017; zie LEESLIJST blok A): varieer bewust
%   alles wat je niet met een schuifmaat kunt narekenen, zodat de optimizer
%   niet kan leunen op toevalligheden van één perfecte simulatie. Wat je wél
%   vertrouwt (spoorbreedte, wielstraal, sensorposities) blijft vrijwel vast.

N = numel(p.lijn.y);
if nargin < 2 || isempty(rs)
    var.k_L = 1;  var.k_R = 1;
    var.u_dz = p.motor.u_dz;
    var.b    = p.auto.b;
    var.tau  = p.motor.tau;
    var.lijn.gain    = ones(N, 1);
    var.lijn.offset  = zeros(N, 1);
    var.lijn.drempel = p.lijn.drempel * ones(N, 1);
    var.lijn.tape_w  = p.lijn.tape_w;
    var.lijn.sigma   = p.lijn.sigma;
    var.sonar.beta      = p.sonar.beta;
    var.sonar.gamma_max = p.sonar.gamma_max;
    var.sonar.p_uitval  = p.sonar.p_uitval;
    var.seed = 1;
    return
end

kbat  = p.var.bat(1) + diff(p.var.bat) * rs.rand();
delta = p.var.delta_motor * (2 * rs.rand() - 1);
var.k_L  = kbat * (1 - delta);
var.k_R  = kbat * (1 + delta);
var.u_dz = 0.10 + 0.12 * rs.rand();                 % 10 .. 22 % PWM
var.b    = p.auto.b * (1 + 0.03 * (2 * rs.rand() - 1));
var.tau  = p.motor.tau * (0.8 + 0.5 * rs.rand());

var.lijn.gain    = min(max(1 + 0.08 * rs.randn(N, 1), 0.75), 1.25);
var.lijn.offset  = 0.04 * rs.randn(N, 1);
var.lijn.drempel = p.lijn.drempel + 0.08 * (2 * rs.rand(N, 1) - 1);
var.lijn.tape_w  = p.lijn.tape_w + 0.001 * (2 * rs.rand() - 1);
var.lijn.sigma   = p.lijn.sigma * (0.8 + 0.5 * rs.rand());

var.sonar.beta      = deg2rad(p.var.beta(1)  + diff(p.var.beta)  * rs.rand());
var.sonar.gamma_max = deg2rad(p.var.gamma(1) + diff(p.var.gamma) * rs.rand());
var.sonar.p_uitval  = 0.04 * rs.rand();

var.seed = rs.randi(2^31 - 1);
end
