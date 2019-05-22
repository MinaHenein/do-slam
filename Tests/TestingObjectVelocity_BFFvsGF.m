R1 = rot(rand(3,1));
Rk = rot(rand(3,1));
R2 = Rk*R1*Rk';

th1 = acos((trace(R1)-1)/2);
th2 = acos((trace(R2)-1)/2);

aa1 = arot(R1);
aa2 = arot(R2);

w1 = skew_symmetric(aa1);
w2 = skew_symmetric(aa2);

V1 = eye(3) + ((1-cos(th1))/th1^2)*w1 + ((th1-sin(th1))/th1^3)*w1^2;
V2 = eye(3) + ((1-cos(th2))/th2^2)*w2 + ((th2-sin(th2))/th2^3)*w2^2;

t1 = rand(3,1);
tk = rand(3,1);
t2 = -Rk*R1*Rk'*tk + Rk*t1 + tk;

v1 = V1\t1;
v2 = V2\t2;

norm(v1)
norm(v2)
%--------------------------------------------------------------------------
X1 = [R1 t1; 0 0 0 1];
X2 = [R2 t2; 0 0 0 1];

vw1 = LogSE3(X1);
vw2 = LogSE3(X2);

v1 = vw1(1:3);
v2 = vw2(1:3);

norm(v1)
norm(v2)
