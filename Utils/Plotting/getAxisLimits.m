function [axisLimits] = getAxisLimits(config,graphCell)
%PLOTGRAPHFILE Summary of this function goes here
%   Detailed explanation goes here

%% 1. get vertices and labels
rowLengths = cellfun('length',graphCell);
vertexRows = (rowLengths==3);
verticesCellTemp = graphCell(vertexRows,:);
verticesCell = cell(sum(vertexRows),3);
for i = 1:sum(vertexRows)
    verticesCell(i,:) = verticesCellTemp{i,:};
end
vertexLabels = verticesCell(:,1);

%% 2. get edges and labels
edgeRows = (rowLengths==6);
edgesCellTemp = graphCell(edgeRows,:);
edgesCell = cell(sum(edgeRows),6);
for i = 1:sum(edgeRows)
    edgesCell(i,:) = edgesCellTemp{i,:};
end
edgeLabels = edgesCell(:,1);

%% 3. identify poses, points, planes, point
poseVertices  = strcmp(vertexLabels,config.labelPoseVertex);
pointVertices = strcmp(vertexLabels,config.labelPointVertex);
planeVertices = strcmp(vertexLabels,config.labelPlaneVertex);
pointPlaneEdges = strcmp(edgeLabels,config.labelPointPlaneEdge);

%% 4. get values
poses  = [verticesCell{poseVertices,3}];
points = [verticesCell{pointVertices,3}];
planes = [verticesCell{planeVertices,3}];

%% 5. min/max

% points = [points poses(1:3,:)];
axisLimits = [min(points(1,:)) max(points(1,:)) min(points(2,:)) max(points(2,:)) min(points(3,:)) max(points(3,:))];

end

