%--------------------------------------------------------------------------
% Author: Yash Vyas - yjvyas@gmail.com - 26/08/2017
% Contributors:
%--------------------------------------------------------------------------
clear all 
% close all 

%% 1. Config
config = CameraConfig();
setAppConfig(config); % copy same settings for error Analysis
% time
nSteps = 1;
t  = [0];
config.set('t',t);
% set motion model in setAppConfig function
config.set('noiseModel','Off');
% config.set('groundTruthFileName','groundTruth_EnvironmentTest1.graph');
% config.set('measurementsFileName','measurements_EnvironmentTest1.graph');

%% 2. Manual Set-up of points
triangleCornersAbsolute = [3 -1 1; 3 0 -1; 3 1 1]'; % the triangle used to occlude
triangleCornersRelative = [0 -1 1; 0 0 -1; 0 1 1]';
triangleCornersGP_Point = GP_Point.empty();
for i=1:size(triangleCornersRelative,2)
    triangleCornersGP_Point(i) = GP_Point(triangleCornersRelative(:,i));
end
triangleLinks = [1 2 3];
staticPoints = [4 0 0; 3 -1.2 1.2]';

visibility = [];
for i=1:size(staticPoints,2)
    visibility(i,1) = SimulatedEnvironmentOcclusionSensor.checkOcclusion(staticPoints(:,i),reshape(triangleCornersAbsolute,[1 9]));
end

disp(visibility);
%% 3. Set up environment
if config.rngSeed
    rng(config.rngSeed); 
end

robotWaypoints = [0 1; 0 0; 0 0; 0 0]; % static sensor
primitiveWaypoints = [0 1; 3 3; 0 0; 0 0];

robotTrajectory = PositionModelPoseTrajectory(robotWaypoints,'R3','linearinterp');
primitiveTrajectory = PositionModelPoseTrajectory(primitiveWaypoints,'R3','linearinterp');

environment = Environment();
environment.addEP_Default(triangleCornersRelative,'R3',primitiveTrajectory);
environment.get('environmentPrimitives',1).set('meshLinks',triangleLinks);
environment.get('environmentPrimitives',1).set('meshPoints',triangleCornersGP_Point);
environment.addStaticPoints(staticPoints);
cameraTrajectory = RelativePoseTrajectory(robotTrajectory,config.cameraRelativePose);

%% 3. Test sensor
sensor = SimulatedEnvironmentOcclusionSensor();
sensor.addEnvironment(environment);
sensor.addCamera(config.fieldOfView,cameraTrajectory);
sensor.setVisibility(config,environment);

sensor.get('pointVisibility')
