clear all
% close all
 
%% Trajectory Initialisation
% 1. Trajectory can be initialised from waypoints, full pose/position data, or
%    some model (see constructor in PoseTrajectory/PositionTrajectory)
% 2. 'model' construction method not yet implemented
% 3. Only R3 waypoints currently implemented.

%% 1. Generate Trajectories
% waypoints (each column = [t,x,y,z]')
waypoints = [0.1 0.15 0.25 0.4;
             0 2 4 8;
             0 3 5 2;
             0 4 6 8];
         
% fitting
nPoses = 51;
t0 = 0;
t1 = 0.5;
fitType = 'smoothingspline';
tFit = linspace(t0,t1,nPoses);

% construct trajectories
poseTrajectory1 = PoseTrajectory('waypoints','R3',waypoints,tFit,'poly1');
poseTrajectory2 = PoseTrajectory('waypoints','R3',waypoints,tFit,'poly2');
poseTrajectory3 = PoseTrajectory('waypoints','R3',waypoints,tFit,'linearinterp');
%Example - constructing from full pose data
dataPoints = [poseTrajectory1.get('t');
              poseTrajectory1.get('poses').get('R3xso3Pose')];
poseTrajectory4 = PoseTrajectory('discrete','R3xso3',dataPoints);
poseTrajectory5 = PoseTrajectory('waypoints','R3',waypoints,tFit,'smoothingspline');
poseTrajectory6 = PoseTrajectory('waypoints','R3',waypoints,tFit,'smoothingspline');

pointTrajectory1 = PointTrajectory('waypoints','R3',waypoints,tFit,'poly1');
pointTrajectory2 = PointTrajectory('waypoints','R3',waypoints,tFit,'poly2');
pointTrajectory3 = PointTrajectory('waypoints','R3',waypoints,tFit,'linearinterp');
pointTrajectory4 = PointTrajectory('waypoints','R3',waypoints,tFit,'cubicinterp');
pointTrajectory5 = PointTrajectory('waypoints','R3',waypoints,tFit,'smoothingspline');

%% 2. Plot
% pose trajectory
figure
viewPoint = [-30,60];
axisLimits = [-5,10,-5,8,-5,10];

subplot(2,3,1)
title('poly1')
axis equal
xlabel('x')
ylabel('y')
zlabel('z')
view(viewPoint)
axis(axisLimits)
hold on
plot3(waypoints(2,:),waypoints(3,:),waypoints(4,:),'r*')
poseTrajectory1.plot()

subplot(2,3,2)
title('poly2')
axis equal
xlabel('x')
ylabel('y')
zlabel('z')
view(viewPoint)
axis(axisLimits)
hold on
plot3(waypoints(2,:),waypoints(3,:),waypoints(4,:),'r*')
poseTrajectory2.plot()

subplot(2,3,3)
title('linearinterp')
axis equal
xlabel('x')
ylabel('y')
zlabel('z')
view(viewPoint)
axis(axisLimits)
hold on
plot3(waypoints(2,:),waypoints(3,:),waypoints(4,:),'r*')
poseTrajectory3.plot()

subplot(2,3,4)
title('cubicinterp')
axis equal
xlabel('x')
ylabel('y')
zlabel('z')
view(viewPoint)
axis(axisLimits)
hold on
plot3(waypoints(2,:),waypoints(3,:),waypoints(4,:),'r*')
poseTrajectory4.plot()

subplot(2,3,5)
title('smoothingspline (logSE3)')
axis equal
xlabel('x')
ylabel('y')
zlabel('z')
view(viewPoint)
axis(axisLimits)
hold on
plot3(waypoints(2,:),waypoints(3,:),waypoints(4,:),'r*')
poseTrajectory5.plot()

subplot(2,3,6)
title('smoothingspline (R3xso3)')
axis equal
xlabel('x')
ylabel('y')
zlabel('z')
view(viewPoint)
axis(axisLimits)
hold on
plot3(waypoints(2,:),waypoints(3,:),waypoints(4,:),'r*')
poseTrajectory6.plot()

suptitle('Comparison of pose trajectory fitting methods')

% position trajectory
figure
viewPoint = [-30,60];
axisLimits = [-5,10,-5,8,-5,10];

subplot(2,3,1)
title('poly1')
axis equal
xlabel('x')
ylabel('y')
zlabel('z')
view(viewPoint)
axis(axisLimits)
hold on
plot3(waypoints(2,:),waypoints(3,:),waypoints(4,:),'r*')
pointTrajectory1.plot()

subplot(2,3,2)
title('poly2')
axis equal
xlabel('x')
ylabel('y')
zlabel('z')
view(viewPoint)
axis(axisLimits)
hold on
plot3(waypoints(2,:),waypoints(3,:),waypoints(4,:),'r*')
pointTrajectory2.plot()

subplot(2,3,3)
title('linearinterp')
axis equal
xlabel('x')
ylabel('y')
zlabel('z')
view(viewPoint)
axis(axisLimits)
hold on
plot3(waypoints(2,:),waypoints(3,:),waypoints(4,:),'r*')
pointTrajectory3.plot()

subplot(2,3,4)
title('cubicinterp')
axis equal
xlabel('x')
ylabel('y')
zlabel('z')
view(viewPoint)
axis(axisLimits)
hold on
plot3(waypoints(2,:),waypoints(3,:),waypoints(4,:),'r*')
pointTrajectory4.plot()

subplot(2,3,5)
title('smoothingspline')
axis equal
xlabel('x')
ylabel('y')
zlabel('z')
view(viewPoint)
axis(axisLimits)
hold on
plot3(waypoints(2,:),waypoints(3,:),waypoints(4,:),'r*')
pointTrajectory5.plot()

suptitle('Comparison of position trajectory fitting methods')