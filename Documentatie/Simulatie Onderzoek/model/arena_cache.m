function w = arena_cache(seed, soort)
%ARENA_CACHE  arena_maken met geheugen per MATLAB-proces (ook per parfor-worker).
%
%   w = arena_cache(seed)            willekeurige arena ('rommel')
%   w = arena_cache(seed, 'rustig')  alleen kisten van >= 15 cm, geen dunne palen
%   Een afstandskaart is een paar MB; zo wordt elke arena per worker één keer
%   gemaakt in plaats van bij elke parfor-aanroep opnieuw verstuurd.

persistent C
if isempty(C), C = containers.Map('KeyType', 'char', 'ValueType', 'any'); end
if nargin < 2, soort = 'rommel'; end
sleutel = sprintf('%s_%d', soort, seed);
if isKey(C, sleutel)
    w = C(sleutel);
    return
end
switch soort
    case 'rustig', w = arena_maken(seed, struct('rustig', true));
    otherwise,     w = arena_maken(seed);
end
if C.Count > 400, remove(C, keys(C)); end
C(sleutel) = w;
end
