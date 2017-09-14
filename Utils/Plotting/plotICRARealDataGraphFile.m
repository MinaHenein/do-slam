function plotICRARealDataGraphFile(config,resultsCell,graphColour)
%PLOTGRAPHFILE Summary of this function goes here
%   Detailed explanation goes here

%% 1. get vertices and labels
rowLengths = cellfun('length',resultsCell);
vertexRows = (rowLengths==3);
verticesCellTemp = resultsCell(vertexRows,:);
verticesCell = cell(sum(vertexRows),3);
for i = 1:sum(vertexRows)
    verticesCell(i,:) = verticesCellTemp{i,:};
end
vertexLabels = verticesCell(:,1);

%% 3. identify poses, points, planes, point
poseVertices  = strcmp(vertexLabels,config.poseVertexLabel);
pointVertices = strcmp(vertexLabels,config.pointVertexLabel);

%% 4. get values
poses  = [verticesCell{poseVertices,3}];
points = [verticesCell{pointVertices,3}];

%% 5. plot poses
plot3(poses(1,:),poses(2,:),poses(3,:),'Color',graphColour,'Marker','o','LineStyle','none');
for i = 1:sum(poseVertices)
    iPose = poses(:,i);
    if strcmp(config.poseParameterisation,'SE3')
        iPose = LogSE3_Rxt(iPose);
    end 
    scale = 0.01;
    plotCoordinates(iPose(1:3,:),scale*rot(iPose(4:6,1)))
end

%% 6. plot points

obj1Pt1 = [22,27,30,33];
obj1Pt2 = [29,32,35,37,39,41,43,45,47,49,51];
obj2Pt1 = [9,12,15,18];
obj2Pt2 = [3,5,7,11,14,17,20,23,55,57,59,61];

ptIDs = find(pointVertices);
colour = 'c';

for i=1:size(ptIDs,1)
if ismember(ptIDs(i),obj1Pt1)
    colour = 'r';
elseif ismember(ptIDs(i),obj1Pt2)
    colour = 'm';
elseif ismember(ptIDs(i),obj2Pt1)
    colour = 'b';
elseif ismember(ptIDs(i),obj2Pt2)
    colour = 'k';    
end
plotPoints = plot3(points(1,i),points(2,i),points(3,i),'.');
set(plotPoints,'MarkerEdgeColor',colour,'MarkerSize',10)
axis equal
hold on
end

end