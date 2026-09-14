function mnt = sonar_opstelling(naam, alpha, p)
%SONAR_OPSTELLING  Plaats en kijkrichting van de HC-SR04's op de auto.
%
%   mnt = sonar_opstelling(naam, alpha, p)   ->  n x 3, per sensor [x y richting]
%       'recht'   één sensor, recht vooruit (zoals nu gebouwd)
%       'uit2'    twee sensoren naast elkaar (y = +/-25 mm), elk alpha naar buiten
%       'kruis2'  twee sensoren 'scheel': de linker kijkt naar rechts en omgekeerd;
%                 de bundels kruisen elkaar op x = 25 mm / tan(alpha) voor de auto
%       'drie'    één recht vooruit plus twee op y = +/-50 mm, alpha naar buiten
%   Een HC-SR04-printje is 45 mm breed; bij twee of drie naast elkaar staan de
%   buitenste iets terug (x - 4 of 8 mm) zodat ze na het draaien nog passen.

xs = p.sonar.x;
switch naam
    case 'recht'
        mnt = [xs 0 0];
    case 'uit2'
        mnt = [xs - 0.004,  0.025,  alpha;
               xs - 0.004, -0.025, -alpha];
    case 'kruis2'
        mnt = [xs - 0.004, -0.025,  alpha;
               xs - 0.004,  0.025, -alpha];
    case 'drie'
        mnt = [xs,          0,      0;
               xs - 0.008,  0.050,  alpha;
               xs - 0.008, -0.050, -alpha];
    otherwise
        error('sonar_opstelling: onbekende opstelling ''%s''.', naam);
end
end
