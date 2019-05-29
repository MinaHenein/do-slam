quat = [-0.9935149550437927, -0.09849041700363159, -0.054612502455711365, -0.015662116929888725];
axisAngle = q2a(quat);
axisRot = axisAngle/norm(axisAngle);
angle = norm(axisAngle);

xEast = axisRot(1); yNorth = axisRot(2); zUp = axisRot(3);
lat0 = 149.1287224; lon0 = -35.3064297; h0 = 585573;
spheroid = referenceSphere('Earth');
[X,Y,Z] = enu2ecef(xEast,yNorth,zUp,lat0,lon0,h0,spheroid);
plotCoordinates([0;0;0],rot([X;Y;Z]),'mkc')
xlabel('x')
ylabel('y')
zlabel('z')
axis equal

r = 6371000 + h0;
x = r*cosd(lat0)*cosd(lon0);
y = r*cosd(lat0)*sind(lon0);
z = r*sind(lat0);