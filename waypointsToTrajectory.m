clear all
% close all

%% 1. Generate Trajectories
% waypoints (each column = [t,x,y,z]')
waypoints1 = [0.1 0.15 0.25 0.5;
             0 2 4 8;
             0 3 5 2;
             0 4 6 8];
waypoints2 = waypoints1;
waypoints2(2:4,:) = fliplr(waypoints2(2:4,:));    
         
         
% fitting
nPoses = 21;
t0 = 0;
t1 = 0.5;
fitType = 'smoothingspline';
tFit = linspace(t0,t1,nPoses);

% construct trajectories
poseTrajectory1 = PoseTrajectory('waypoints','R3',waypoints1,tFit,fitType);
poseTrajectory2 = PoseTrajectory('waypoints','R3',waypoints2,tFit,fitType);
dataPoints = [poseTrajectory1.get('t');
              poseTrajectory1.get('poses').get('R3xso3Pose')];
poseTrajectory3 = PoseTrajectory('discrete','R3xso3',dataPoints);
pointTrajectory1 = PointTrajectory('waypoints','R3',waypoints1,tFit,fitType);
pointTrajectory2 = PointTrajectory('waypoints','R3',waypoints2,tFit,fitType);

%% 2. Plot
% pose trajectory
figure
viewPoint = [-30,60];
axisLimits = [-5,10,-5,10,-5,10];

subplot(1,3,1)
axis equal
xlabel('x')
ylabel('y')
zlabel('z')
view(viewPoint)
axis(axisLimits)
hold on
plot3(waypoints1(2,:),waypoints1(3,:),waypoints1(4,:),'r*')
plot3(waypoints1(2,1),waypoints1(3,1),waypoints1(4,1),'g*')
poseTrajectory1.plot()

subplot(1,3,2)
axis equal
xlabel('x')
ylabel('y')
zlabel('z')
view(viewPoint)
axis(axisLimits)
hold on
plot3(waypoints2(2,:),waypoints2(3,:),waypoints2(4,:),'r*')
plot3(waypoints2(2,1),waypoints2(3,1),waypoints2(4,1),'g*')
poseTrajectory2.plot()

subplot(1,3,3)
axis equal
xlabel('x')
ylabel('y')
zlabel('z')
view(viewPoint)
axis(axisLimits)
hold on
plot3(waypoints2(2,:),waypoints2(3,:),waypoints2(4,:),'r*')
plot3(waypoints2(2,1),waypoints2(3,1),waypoints2(4,1),'g*')
pointTrajectory1.plot()