function st = aandrijving_stap(st, u, vrij, var, p, dt)
%AANDRIJVING_STAP  Eén tijdstap van motoren + differentiële aandrijving.
%
%   st   = [x y theta vL vR]  (positie [m], koers [rad], wielsnelheden [m/s])
%   u    = [uL uR]  PWM-fractie -1..1 zoals de microcontroller hem uitstuurt
%   vrij = [vrijL vrijR]  true = motor los (EN laag): wiel rolt uit
%   var  = variatie van deze run (variatie_trekken)
%
%   Motor (per wiel), met dode zone u_dz en versterking k (accu, L/R-verschil):
%       u_eff = sign(u) max(|u| - u_dz, 0) / (1 - u_dz)
%       dv/dt = (k u_eff v0 - v) / tau_m,     |dv/dt| <= a_slip
%   Uitrollen:  dv/dt = -a_rol sign(v)
%   Kinematica (Siegwart et al. 2011, hfst. 3):
%       v = (vR + vL) / 2,   omega = (vR - vL) / b
%   Integratie met de koers halverwege de stap (exact voor constante v, omega
%   tot op O(dt^3)).

vw  = st(4:5);
k   = [var.k_L var.k_R];
ue  = sign(u) .* max(abs(u) - var.u_dz, 0) / (1 - var.u_dz);
vss = k .* ue * p.motor.v0;
a   = (vss - vw) * (1 - exp(-dt / var.tau)) / dt;
a   = min(max(a, -p.motor.a_slip), p.motor.a_slip);
if any(vrij)
    a(vrij) = -sign(vw(vrij)) * p.motor.a_rol;
    stopt = vrij & abs(vw) <= p.motor.a_rol * dt;
    a(stopt) = -vw(stopt) / dt;
end
vw = vw + a * dt;

v  = (vw(1) + vw(2)) / 2;
w  = (vw(2) - vw(1)) / var.b;
tm = st(3) + w * dt / 2;
st = [st(1) + v * cos(tm) * dt, st(2) + v * sin(tm) * dt, st(3) + w * dt, vw];
end
