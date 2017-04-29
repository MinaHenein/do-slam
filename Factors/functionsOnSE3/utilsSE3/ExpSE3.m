
function T = ExpSE3(a)

% This function computes the exponantial of an se(3) and returns homogeneous
% coordinates

v = a(1:3);
w = a(4:6);
I3 = eye(3);

th = sqrt(w' * w);
%rd = rd/th;
cd =cos(th);
sd = sin(th);
wx=skew_symmetric(w);

%Rd = I3  + rx * sd + (1 - cd) *  rx^2;
if th == 0
    Rd = I3  +  wx + (1/2) *  wx^2;
    V = I3 - (1/2) * wx +  (1/6)  * wx^2;
else
    Rd = I3  + (sd/th) * wx + ((1 - cd)/th^2) *  wx^2;
    V = (I3 - ((1 - cd)/(th^2)) * wx +  ((th - sd) / (th^3) )  * wx^2);
end
v = V * v;

T = [Rd v; 0 0 0 1];



