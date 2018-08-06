function [obj] = construct2PointsSE3MotionEdge(obj,config,edgeRow)
%CONSTRUCT2POINTSSE3MOTIONEDGE constructs edge representing measurement
%between two points and their respective SE3 motion vertex

%% 1. load vars from edge row
edgeLabel = edgeRow{1};
edgeIndex = edgeRow{2};
pointVertices = edgeRow{3};
SE3MotionVertexIndices = edgeRow{4};
SE3MotionVertex = SE3MotionVertexIndices(end);
edgeValue = edgeRow{5};
edgeCovariance = edgeRow{6};

%% 2. properties
value       = edgeValue;
covariance  = upperTriVecToCov(edgeCovariance);
jacobians   = [];
type        = '2points-SE3Motion';
iVertices   = [pointVertices SE3MotionVertex];
index       = edgeIndex;

%% 3. construct edge
obj.edges(edgeIndex) = Edge(value,covariance,jacobians,type,iVertices,index);

%% 4. update edge
%computes value & jacobians
obj = obj.update2PointsSE3MotionEdge(config,edgeIndex);

%% 5. add index to point and motion vertices
obj.vertices(pointVertices(1)).iEdges = [obj.vertices(pointVertices(1)).iEdges edgeIndex];
obj.vertices(pointVertices(2)).iEdges = [obj.vertices(pointVertices(2)).iEdges edgeIndex];
obj.vertices(SE3MotionVertex).iEdges = unique([obj.vertices(SE3MotionVertex).iEdges edgeIndex]);

%% 6. check window size and delete old edges
%% this is for landmarks window size
nLandmarks = numel(obj.vertices(SE3MotionVertex).iEdges) + 1;
if nLandmarks > config.landmarksSlidingWindowSize
    % delete observation edge index from pose vertices
    pose1Vertex = obj.vertices(obj.edges(obj.vertices(pointVertices(1))...
        .iEdges(1)).iVertices(1));
    pose1Vertex.iEdges(pose1Vertex.iEdges==obj.vertices(pointVertices(1)).iEdges(1))=[];
    % delete ternary edge index from SE3 motion vertex 
    firstEdgeIndex = obj.vertices(SE3MotionVertex).iEdges(1);
    obj.vertices(SE3MotionVertex).iEdges...
        (obj.vertices(SE3MotionVertex).iEdges==firstEdgeIndex) =[];
    % deactivate landmark edges
    edgesIndices = [obj.vertices(pointVertices(1)).iEdges];
    for i =1:length(edgesIndices)
        obj.edges(edgesIndices(i)).active = 0;
    end
    % delete first landmark vertex content
    obj.vertices(pointVertices(1)) = Vertex();
end


%% this is for object poses window size
posesVertices = [obj.vertices(obj.identifyVertices('pose')).index];
edgesConnectedToMotionVertex = [obj.vertices(SE3MotionVertex).iEdges];

pointVerticesConnectedToMotionVertex = ...
    [obj.edges(edgesConnectedToMotionVertex).iVertices];
pointVerticesConnectedToMotionVertex = ...
    pointVerticesConnectedToMotionVertex...
    (pointVerticesConnectedToMotionVertex~=SE3MotionVertex);
if length(pointVerticesConnectedToMotionVertex)>2
    pointVerticesConnectedToMotionVertex = reshape(pointVerticesConnectedToMotionVertex,...
        [2,length(pointVerticesConnectedToMotionVertex)/2])';
end

uniquePoseIndex = [];
for i=1:size(pointVerticesConnectedToMotionVertex,1)
    closestPose1index = posesVertices(sum((pointVerticesConnectedToMotionVertex(i,1)-posesVertices)>0));
    closestPose2index = posesVertices(sum((pointVerticesConnectedToMotionVertex(i,2)-posesVertices)>0));     
    if isempty(uniquePoseIndex)
        uniquePoseIndex = [closestPose1index, closestPose2index];
    else
        if ~ismember(closestPose1index,uniquePoseIndex)
            uniquePoseIndex = [uniquePoseIndex closestPose1index];
        end
        if ~ismember(closestPose2index,uniquePoseIndex)
            uniquePoseIndex = [uniquePoseIndex closestPose2index];
        end
    end
end

nObjectPoses = numel(uniquePoseIndex); 
if nObjectPoses > config.objectPosesSlidingWindowSize
    firstEdgeIndex = obj.vertices(SE3MotionVertex).iEdges(1);
    firstEdgeVertices = [obj.edges(firstEdgeIndex).iVertices];
    firstLandmarkPoseindex = posesVertices(sum((firstEdgeVertices(1)-posesVertices)>0));
    for j=1:size(pointVerticesConnectedToMotionVertex,1)
        edgeVertex = pointVerticesConnectedToMotionVertex(j,1);
        landmarkPoseIndex = posesVertices(sum((edgeVertex-posesVertices)>0));
        toBeDeleted = [];
        if landmarkPoseIndex==firstLandmarkPoseindex
            toBeDeleted = [toBeDeleted, edgeVertex];
        end
    end
    for k=1:length(toBeDeleted)
        landmarkPoseIndex = posesVertices(sum((toBeDeleted(k)-posesVertices)>0));
        % delete observation edge index from pose vertices
        obj.vertices(landmarkPoseIndex).iEdges...
            (obj.vertices(landmarkPoseIndex).iEdges== ...
            obj.vertices(toBeDeleted(k)).iEdges(1))=[];
        % delete ternary edge index from SE3 motion vertex
        obj.vertices(SE3MotionVertex).iEdges...
            (obj.vertices(SE3MotionVertex).iEdges ==...
            obj.vertices(toBeDeleted(k)).iEdges(2)) = [];
        % delete landmark edges
        edgesIndices = [obj.vertices(toBeDeleted(k)).iEdges];
        for i =1:length(edgesIndices)
            obj.edges(edgesIndices(i)).active = 0;
        end
        % delete landmark vertices
        obj.vertices(toBeDeleted(k)) = Vertex();
    end
    
end

end

