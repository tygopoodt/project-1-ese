function u = pwm_omzetten(u, u_dz_schat, bits)
%PWM_OMZETTEN  Gewenste wielsnelheid (als fractie van v0) naar PWM-waarde.
%
%   De regelaar compenseert de dode zone met zijn eigen schatting u_dz_schat
%   (de echte dode zone verschilt per motor en per run), begrenst op +/-1 en
%   rondt af op de PWM-resolutie van de ATmega328P:
%       u_pwm = round( (u_dz + (1 - u_dz) |u|) * (2^bits - 1) ) / (2^bits - 1)

aan = abs(u) > 1e-3;
u   = sign(u) .* (u_dz_schat + (1 - u_dz_schat) * abs(u)) .* aan;
u   = min(max(u, -1), 1);
n   = 2^bits - 1;
u   = round(u * n) / n;
end
