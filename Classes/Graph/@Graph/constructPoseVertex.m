function [obj] = constructPoseVertex(obj,config,edgeRow)
%CONSTRUCTPOSEVERTEX Constructs vertex representing a pose. Pose is
%estimated from previous pose and odometry measurement

%% 1. load vars from edge row
edgeLabel = edgeRow{1};
edgeIndex = edgeRow{2};
inputVertices = edgeRow{3};
outputVertices = edgeRow{4};
edgeValue = edgeRow{5};
edgeCovariance = edgeRow{6};

%% 2. compute pose
switch edgeLabel
    case config.posePriorEdgeLabel %prior
        pose2 = edgeValue;     
    case config.posePoseEdgeLabel %odometry
        pose1 = obj.vertices(inputVertices).value;
        controlInput = edgeValue;
        switch config.cameraControlInput
            case 'relativePose'
                pose2 = config.relativeToAbsolutePoseHandle(pose1,controlInput);
%                 pose2(4:6) = wrapAxisAngle(pose2(4:6),'pi');
%                 pose2 = RelativeToAbsolutePose(pose1,controlInput);
%                 pose2 = Relative2AbsoluteSE3(pose1,controlInput);
            case 'relativeVelocity'
                pose2 = config.relativeToAbsolutePoseHandle(pose1,controlInput*config.dt);
%                 pose2(4:6) = wrapAxisAngle(pose2(4:6),'pi');
%                 pose2 = RelativeToAbsolutePose(pose1,controlInput*config.dt);
%                 pose2 = Relative2AbsoluteSE3(pose1,controlInput*config.dt);
        end
    otherwise; error('error, wrong edgeLabel')
end

%% 3. vertex properties
value = pose2;
covariance = []; %not using this property yet
type = 'pose';
iEdges = [edgeIndex];
index = outputVertices; 

%% 4. construct vertex
obj.vertices(index) = Vertex(value,covariance,type,iEdges,index);

end

