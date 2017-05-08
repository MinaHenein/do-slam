clear all 
% close all 

%% 1. Config
% *TODO: create config class
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
% environment.addRectangle([10,15],100,'mixed',staticTrajectory1);
% environment.addRectangle([8,6],50,'mixed',staticTrajectory2);
% 
% 
% %% 3. Initialise Sensor
% fieldOfView = [-pi/3,pi/3,-pi/6,pi/6];
% maxRange    = 10;
% cameraPoseRelativeToRobot = GP_Pose([0,0,0,0,0,-pi/8]');
% %*TODO: write trajectory methods to do this
% cameraTrajectory = dynamicTrajectory.copy();
% cameraTrajectory.set('poses',cameraPoseRelativeToRobot.RelativeToAbsolutePose(cameraTrajectory.get('poses')));
% camera = SimulatedEnvironmentSensor(fieldOfView,maxRange,cameraTrajectory);
% 
% %% 4. Create SensorObjects
% sensorEnvironment = SensorEnvironment(environment);
% 
% %% 5. Generate Measurements & Save to Graph File
% % camera.generateMeasurements(sensorEnvironment);

%% 6. Plot
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
% camera.get('trajectory').plot()
% for i = 1:nSteps
%     h = environment.plot(i);
%     drawnow
%     pause(0.01)
%     if i < nSteps
%         delete(h)
%     end
% end