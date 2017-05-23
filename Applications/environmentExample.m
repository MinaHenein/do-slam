%--------------------------------------------------------------------------
% Author: Montiel Abello - montiel.abello@gmail.com - 23/05/17
% Contributors:
%--------------------------------------------------------------------------

clear all 
% close all 

%% 1. Config
% time
nSteps = 51;
t0 = 0;
tN = 10;
t  = linspace(t0,tN,nSteps);
% waypoints
staticPose1  = [5,8,0,0,0,0]';
staticPose2  = [4 16 4 pi/2 0 0]';
dynamicWaypoints = [0:2:tN; 4 5 7 10 7 3; 12 11 10 5 8 14; 8 5 2 3 6 4];
%construct trajectories
staticTrajectory1 = StaticPoseTrajectory(staticPose1,'R3xso3');
staticTrajectory2 = StaticPoseTrajectory(staticPose2,'R3xso3');
dynamicTrajectory = PositionModelPoseTrajectory(dynamicWaypoints,'R3','smoothingspline');

%% 2. Generate Environment
% initialise environment
environment = Environment();
% % add primitives to environment
environment.addRectangle([10,15],100,'mixed',staticTrajectory1);
environment.addRectangle([8,6],50,'mixed',staticTrajectory2);
environment.addPrimitive(3*rand(3,50)-1.5,'R3',dynamicTrajectory);

%% 3. Plot
figure
viewPoint = [-50,25];
axisLimits = [-1,15,-1,20,-1,10];
title('Environment')
axis equal
xlabel('x')
ylabel('y')
zlabel('z')
view(viewPoint)
axis(axisLimits)
hold on
staticTrajectory1.plot()
staticTrajectory2.plot()
dynamicTrajectory.plot(t)
environment.plot(t)
