function [obj] = constructPointVertex(obj,config,edgeRow)
%CONSTRUCTPOINTVERTEX constructs vertex representing point. Position in
%absolute coordinates is estimated from input pose vertex and measurement

%% 1. load vars from edge row
edgeLabel = edgeRow{1};
edgeIndex = edgeRow{2};
if strcmp(config.landmarkErrorToMinimize,'reprojection')
    posePointVertexes = edgeRow{3};
    poseVertex = posePointVertexes(1);
    pointVertex = posePointVertexes(2);
    intrinsicVertex = edgeRow{4};
else
    poseVertex = edgeRow{3};
    pointVertex = edgeRow{4};
end
switch edgeLabel
    case {config.posePointEdgeLabel,config.posePointIntrinsicEdgeLabel}
        edgeValue = edgeRow{5};
        pointColour = [];
    case config.posePointRGBEdgeLabel
        edgeValue = edgeRow{5}(1:3);
        pointColour = edgeRow{5}(4:6);
    otherwise
        error('%s type invalid',edgeLabel)
end
edgeCovariance = edgeRow{6};

%% 2. compute point position
pose = obj.vertices(poseVertex).value;
if strcmp(config.landmarkErrorToMinimize,'reprojection')
    intrinsics = obj.vertices(intrinsicVertex).value;
end
positionRelative = edgeValue;
switch config.cameraPointParameterisation
    case 'euclidean'
          if strcmp(config.landmarkErrorToMinimize,'reprojection') 
            % should be called only if positionRelative is in image frame
            % would give wrong results if positonRelative is in camera frame
            positionAbsolute = config.relativeToAbsolutePointHandle(pose,positionRelative,intrinsics);
          else
            positionAbsolute = config.relativeToAbsolutePointHandle(pose,positionRelative);
%         positionAbsolute = RelativeToAbsolutePosition(pose,positionRelative);
%         positionAbsolute = RelativePoint2AbsolutePoint3D(pose,positionRelative);
          end
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

