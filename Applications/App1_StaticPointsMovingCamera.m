%--------------------------------------------------------------------------
% Author: Mina Henein - mina.henein@anu.edu.au - 30/06/17
% Contributors:
%--------------------------------------------------------------------------
% App1_StaticPointsMovingCamera

clear all;

%% 1. Config
% time
nSteps = 51;
t0 = 0;
tN = 10;
t  = linspace(t0,tN,nSteps);

% CameraConfig is subclass of Config with properties specific to
% applications with a camera
config = CameraConfig();
% set properties of Config
config.set('t',t);
config.set('rngSeed',1);
config.set('noiseModel','Gaussian');
% config.set('noiseModel','Off');
config.set('poseParameterisation','R3xso3');
% config.set('poseParameterisation','logSE3');

% temporarily changed function handles to public for setting
config.absoluteToRelativePoseHandle = @AbsoluteToRelativePoseR3xso3;
config.absoluteToRelativePointHandle = @AbsoluteToRelativePositionR3xso3;
config.relativeToAbsolutePoseHandle = @RelativeToAbsolutePoseR3xso3;
config.relativeToAbsolutePointHandle = @RelativeToAbsolutePositionR3xso3;

config.set('cameraPointParameterisation','euclidean');
config.set('cameraControlInput','relativePose');
config.set('poseVertexLabel'     ,'VERTEX_POSE_LOG_SE3');
config.set('pointVertexLabel'    ,'VERTEX_POINT_3D');
config.set('planeVertexLabel'    ,'VERTEX_PLANE_4D');
config.set('posePoseEdgeLabel'   ,'EDGE_LOG_SE3');
config.set('posePointEdgeLabel'  ,'EDGE_3D');
config.set('pointPlaneEdgeLabel' ,'EDGE_1D');
config.set('posePriorEdgeLabel','EDGE_6D');
config.set('graphFileFolderName' ,'GraphFiles');
config.set('groundTruthFileName' ,'groundTruthFrontEnd.graph');
config.set('measurementsFileName','measurementsFrontEnd.graph');
config.set('stdPosePrior' ,[0.001,0.001,0.001,pi/600,pi/600,pi/600]');
config.set('stdPointPrior',[0.001,0.001,0.001]');
config.set('stdPosePose'  ,[0.01,0.01,0.01,pi/90,pi/90,pi/90]');
config.set('stdPosePoint' ,[0.02,0.02,0.02]');
config.set('stdPointPlane',0.001);
% set properties of CameraConfig
config.set('fieldOfView',[-pi/3,pi/3,-pi/6,pi/6,1,10]); %az,el,r limits
config.set('cameraRelativePose',GP_Pose([0,0,0,0,0,-pi/8]'));
% set properties of solverConfig
%   dimensions
config.set('dimPose',6);
config.set('dimPoint',3);
%   plane parameterisation'
config.set('planeNormalParameterisation','S2');
%   constraints
config.set('applyAngleConstraints',0);
config.set('automaticAngleConstraints',0);
%   first linearisation point
config.set('startPose','initial');
%   static assumption
config.set('staticAssumption',0);
%   solver settings
config.set('sortVertices',0);
config.set('sortEdges',0);
config.set('processing','batch');
config.set('nVerticesThreshold',100);
config.set('nEdgesThreshold',200);
config.set('solveRate',5);
config.set('solverType','Levenberg-Marquardt');
config.set('threshold',10e-4);
config.set('maxNormDX',1e10);
config.set('maxIterations',1000);
config.set('displayProgress',1);
config.set('savePath',pwd);
config.set('plotPlanes',1);

% random number seed
if config.rngSeed; rng(config.rngSeed); end;

%% environment setup
% waypoints
staticPose1  = [5,8,0,0,0,0]';
staticPose2  = [4 16 4 pi/2 0 0]';
dynamicWaypoints = [0:2:tN; linspace(0,10,6); sqrt(linspace(0,5,6)); linspace(0,3,6)];

% static point locations - corridoor
pointPositions = 50*rand([3 100]);

% construct trajectories
staticTrajectory1 = StaticPoseTrajectory(staticPose1,'R3xso3');
% staticTrajectory2 = StaticPoseTrajectory(staticPose2,'R3xso3');
robotTrajectory   = PositionModelPoseTrajectory(dynamicWaypoints,'R3','smoothingspline');

environment = Environment();
environment.addRectangle([10,15],100,'mixed',staticTrajectory1);
environment.addStaticPoints(pointPositions);

%% 6.Environment Plot
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
robotTrajectory.plot(t)
environment.plot(t)