
% Moves an angle to the -pi,pi range.
% 
% Ensures that the given angles are all in the -pi, pi range.
function angle=pi2pi(angle)

dp=2*pi;
ang=(angle<=-dp)||(angle>=dp);
angle(ang)=mod(angle(ang),dp);

ang=angle>=pi;
angle(ang)=angle(ang)-2*pi;

ang=angle<=-pi;
angle(ang)=angle(ang)+2*pi;