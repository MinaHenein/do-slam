function [obj] = constructPointVertex(obj,config,edgeRow)
%CONSTRUCTPOINTVERTEX constructs vertex representing point. Position in
%absolute coordinates is estimated from input pose vertex and measurement

%% 1. load vars from edge row
edgeLabel = edgeRow{1};
edgeIndex = edgeRow{2};
poseVertex = edgeRow{3};
pointVertex = edgeRow{4};
switch edgeLabel
    case config.labelPosePointEdge
        edgeValue = edgeRow{5};
        pointColour = [];
    case config.labelPosePointRGBEdge
        edgeValue = edgeRow{5}(1:3);
        pointColour = edgeRow{5}(4:6);
end
edgeCovariance = edgeRow{6};

%% 2. compute point position
pose = obj.vertices(poseVertex).value;
positionRelative = edgeValue;
switch config.cameraPointParameterisation
    case 'euclidean'
        positionAbsolute = config.relativeToAbsolutePointHandle(pose,positionRelative);
%         positionAbsolute = RelativeToAbsolutePosition(pose,positionRelative);
%         positionAbsolute = RelativePoint2AbsolutePoint3D(pose,positionRelative);
    otherwise
        error('%d point parameterisation not implemented',config.cameraPointParameterisation)
end

%% 3. vertex properties
value = positionAbsolute;
covariance = []; %not using this property yet
type = 'point';
iEdges = [edgeIndex];
index = pointVertex;  

%% 4. construct vertex
obj.vertices(index) = Vertex(value,covariance,type,iEdges,index);

%% 5. add colour
obj.vertices(index).colour = pointColour;
end

