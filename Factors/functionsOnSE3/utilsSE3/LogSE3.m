
function Ln = LogSE3(T)


R = T(1:3,1:3);
t = T(1:3,4);
I3 = eye(3);

%tr = rd11 + rd22 + rd33;
tr = trace(R);
cd = (tr-1)/2;
if cd < -1
    cd = -1;
elseif cd > 1
    cd = 1;
end
sd = sqrt(1 - cd^2);
th = acos(cd);

%% Rotation

r=[R(3,2) - R(2,3);
   R(1,3) - R(3,1);
   R(2,1) - R(1,2)];  % w can be calculated as R - R'

% o = simplify(b*r);
if (th ~= 0)
    % normalize axis
    b = th/(2*sd);
    w = b*r;
    
    %wn = w/th;
    wx = skew_symmetric(w);
    V = (I3 - ((1 - cd)/(th^2)) * wx +  ((th - sd) / (th^3) )  * wx^2);     
else
    % in the limit
    b = 1/2.;
    w = b*r;
    %wn = w/th;
    wx = skew_symmetric(w);
    V = I3 - (1/2) * wx + (1/6) * wx^2; %TODO check if this is ok
end 

v= inv(V)*t; 
Ln = [v;w];



