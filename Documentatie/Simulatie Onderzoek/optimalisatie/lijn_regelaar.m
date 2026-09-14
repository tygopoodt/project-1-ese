function pp = lijn_regelaar(p, th)
%LIJN_REGELAAR  Zet een parametervector van de optimizer in de regelaar.
%
%   pp = lijn_regelaar(p, th)
%   th = [log10(Kp), log10(Kd), log10(tau_d), v_max, k_v]
%   Kp, Kd en tau_d worden logaritmisch gezocht: hun zinvolle waarden liggen
%   over meerdere decaden verspreid.

pp = p;
pp.regel.Kp    = 10^th(1);
pp.regel.Kd    = 10^th(2);
pp.regel.tau_d = 10^th(3);
pp.regel.v_max = th(4);
pp.regel.k_v   = th(5);
end
