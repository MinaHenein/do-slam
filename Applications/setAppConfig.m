function config = setAppConfig(config)
%SETUNITTESTCONFIG - makes it easier to write common configs for all the
%unit tests, required as new solver features (such as edges) are added.

% set properties of Config
config.set('rngSeed',1);
config.set('noiseModel','Gaussian');
% config.set('noiseModel','Off');
config.set('poseParameterisation','R3xso3');

%% select type of motion model - comment out the correct lines
config.set('pointMotionMeasurement','Off');
% 2 point edge
% config.set('pointMotionMeasurement','point2Edge')
% config.set('stdPointPoint',[0.01 0.01 0.01]');

% point 3 edge
% config.set('pointMotionMeasurement','point3Edge');

%     % speed version
%     config.set('motionModel','constantSpeed')
%     config.set('std3Points',0.01);

    % velocity version
%     config.set('motionModel','constantVelocity');
%     config.set('std3Points',[0.01 0.01 0.01]');

% 2 point velocity vertex
% config.set('pointMotionMeasurement','velocity')

    % speed version
%     config.set('motionModel','constantSpeed')
%     config.set('std2PointsVelocity',0.01);

%     % velocity version
%     config.set('motionModel','constantVelocity')
%     config.set('std2PointsVelocity',[0.01 0.01 0.01]');

% SE3 Motion
% config.set('pointMotionMeasurement','point2DataAssociation');
% config.set('motionModel','constantSE3MotionDA');

%% pose handles
% temporarily changed function handles to public for setting
config.absoluteToRelativePoseHandle = @AbsoluteToRelativePoseR3xso3;
config.absoluteToRelativePointHandle = @AbsoluteToRelativePositionR3xso3;
config.relativeToAbsolutePoseHandle = @RelativeToAbsolutePoseR3xso3;
config.relativeToAbsolutePointHandle = @RelativeToAbsolutePositionR3xso3;

%% other settings
config.set('cameraPointParameterisation','euclidean');
config.set('cameraControlInput','relativePose');
config.set('poseVertexLabel'     ,'VERTEX_POSE_R3_SO3');
config.set('pointVertexLabel'    ,'VERTEX_POINT_3D');
config.set('planeVertexLabel'    ,'VERTEX_PLANE_4D');
config.set('posePoseEdgeLabel'   ,'EDGE_R3_SO3');
config.set('posePointEdgeLabel'  ,'EDGE_3D');
config.set('pointPlaneEdgeLabel' ,'EDGE_1D');
config.set('pointPointEdgeLabel' ,'EDGE_2POINTS');
config.set('pointDataAssociationLabel','2POINTS_DataAssociation')
config.set('point3EdgeLabel','EDGE_3POINTS');
config.set('velocityVertexLabel','VERTEX_VELOCITY');
config.set('pointVelocityEdgeLabel','EDGE_2POINTS_VELOCITY');
config.set('SE3MotionVertexLabel','VERTEX_SE3Motion');
config.set('pointSE3MotionEdgeLabel','EDGE_2POINTS_SE3Motion');
config.set('posePriorEdgeLabel','EDGE_6D');
config.set('graphFileFolderName' ,'GraphFiles');

% set pose prior
rot = eul2rot([pi/180,pi/180,pi/180]); % 1 degree position error
orientation = arot(rot);
config.set('stdPosePrior',[0.01,0.01,0.01,orientation(1),orientation(2),orientation(3)]');

% set point prior error
config.set('stdPointPrior',[0.01,0.01,0.1]');

% set odometry error
rot = eul2rot([pi/180,pi/180,pi/180]); % 1 degree position error
orientation = arot(rot);
config.set('stdPosePose'  ,[0.04,0.04,0.04,orientation']');

% measurement errors
config.set('stdPosePoint' ,[0.04,0.04,0.04]');
config.set('stdPointPlane',0.001);
% set properties of CameraConfig
config.set('fieldOfView',[-pi/3,pi/3,-pi/3,pi/3,1,40]); %az,el,r limits
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
% data association
config.set('staticDataAssociation','consistent');
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
config.set('plotPlanes',0);

end

