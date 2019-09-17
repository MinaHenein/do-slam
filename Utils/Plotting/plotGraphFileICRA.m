function plotGraphFileICRA(config,groundTruthCell,setting,varargin)
%PLOTGRAPHFILE Summary of this function goes here
%   Detailed explanation goes here

%% 1. get vertices and labels
rowLengths = cellfun('length',groundTruthCell);
vertexRows = (rowLengths==3);
verticesCellTemp = groundTruthCell(vertexRows,:);
verticesCell = cell(sum(vertexRows),3);
for i = 1:sum(vertexRows)
    verticesCell(i,:) = verticesCellTemp{i,:};
end
vertexLabels = verticesCell(:,1);

%% 2. get edges and labels
edgeRows = (rowLengths==6);
edgesCellTemp = groundTruthCell(edgeRows,:);
edgesCell = cell(sum(edgeRows),6);
for i = 1:sum(edgeRows)
    edgesCell(i,:) = edgesCellTemp{i,:};
end
edgeLabels = edgesCell(:,1);

%% 3. identify poses, points, planes, point
poseVertices  = strcmp(vertexLabels,config.poseVertexLabel);
pointVertices = strcmp(vertexLabels,config.pointVertexLabel);
planeVertices = strcmp(vertexLabels,config.planeVertexLabel);
pointPlaneEdges = strcmp(edgeLabels,config.pointPlaneEdgeLabel);

%% 4. get values
poses  = [verticesCell{poseVertices,3}];
points = [verticesCell{pointVertices,3}];
planes = [verticesCell{planeVertices,3}];

%% 5. plot poses and points

switch setting
    case 'groundTruth'
        plotPoses = plot3(poses(1,:),poses(2,:),poses(3,:),'Color','g','Marker','.','LineStyle','none');
        set(plotPoses,'MarkerSize',7);
%         for i = 1:sum(poseVertices)
%             iPose = poses(:,i);            
%             plotiCamera = plotCamera('Location',iPose(1:3),'Orientation',rot(-iPose(4:6))); %LHS invert pose
%             scale = 0.5;
%             %plotCoordinates(iPose(1:3),scale*rot(iPose(4:6)))
%             plotiCamera.Opacity = 0.1;
%             plotiCamera.Size = 0.1;
%             plotiCamera.Color = 'g';
%         end

        % uncomment to plot ground truth poses as coordinates
%         for i = 1:sum(poseVertices)
%         iPose = poses(:,i);
%         plotCoordinates(iPose(1:3,:),rot(iPose(4:6,1)))
%         end   
%         if ~isempty(points)
%             plotPoints = plot3(points(1,:),points(2,:),points(3,:),'g.');
%             set(plotPoints,'MarkerSize',5)
%         end
    case 'solverResults'
        relPose = varargin{1};
        posePoints = varargin{2};
        graphN = varargin{3};
        staticPointIndices = varargin{4};
        dynamicPointIndicesPerObject = varargin{5};
        pointVertices = [graphN.vertices(graphN.identifyVertices('point'))];   
        
        colors = {'red','magenta','radioactive green','leather','red','green','black','sapphire','swamp','light bluish green',...
    'butterscotch','cinnamon','chartreuse'}; 
        
        for i = 1:sum(poseVertices)
            iPose = poses(:,i);
            if strcmp(config.poseParameterisation,'SE3')
                iPose = LogSE3_Rxt(iPose);
            end
            iPose = RelativeToAbsolutePoseR3xso3(iPose,relPose);
            scale = 1;
            poses(:,i) = iPose;
            plotCoordinates(iPose(1:3,:),scale*rot(iPose(4:6,1))) % plots the trajectory as axes
        end
        
%         if ~isempty(points)
%         for i=1:size(points,2)
%             points(:,i) = RelativeToAbsolutePositionR3xso3(posePoints,points(:,i));
%         end
%         end
%         plotPoses = plot3(poses(1,:),poses(2,:),poses(3,:),'Color','b','Marker','.','LineStyle','none');
%         set(plotPoses,'MarkerSize',8);
        if ~isempty(staticPointIndices)      
         for i=1:size(points,2)
            if ismember(pointVertices(i).index,staticPointIndices)
                plot3(points(1,i),points(2,i),points(3,i),'b.','MarkerSize',5);
             end
         end   
        end
        nDynamicObjects = size(dynamicPointIndicesPerObject,1); % first cell are static points
        for i=1:size(points,2)
            for j=1:nDynamicObjects
                if ismember(pointVertices(i).index,dynamicPointIndicesPerObject{j,1})
                    plot3(points(1,i),points(2,i),points(3,i),'.','Color',rgb(colors(j)),'MarkerSize',5);
                    break
                end
            end
        end
        
    case 'initial'
        relPose = varargin{1};
        posePoints = varargin{2};
        
        for i = 1:sum(poseVertices)
            iPose = poses(:,i);
            if strcmp(config.poseParameterisation,'SE3')
                iPose = LogSE3_Rxt(iPose);
            end
            iPose = RelativeToAbsolutePoseR3xso3(iPose,relPose);
            poses(:,i) = iPose;
            %plotCoordinates(iPose(1:3,:),rot(iPose(4:6,1)))
        end
        
        for i=1:size(points,2)
            points(:,i) = RelativeToAbsolutePositionR3xso3(posePoints,points(:,i));
        end
        plotPoses = plot3(poses(1,:),poses(2,:),poses(3,:),'Color','r','Marker','.','LineStyle','none');
        set(plotPoses,'MarkerSize',7);
        plotPoints = plot3(points(1,:),points(2,:),points(3,:),'r.');
        set(plotPoints,'MarkerSize',5)
end

%% 7. plot planes
%need edges to figure out which points are on which planes
iPlaneVertices = [verticesCell{planeVertices,2}];
pointPlaneVertices = [edgesCell{pointPlaneEdges,3}; %[point plane]
                      edgesCell{pointPlaneEdges,4}]';
for i = 1:sum(planeVertices)
    %plane vertex index
    iPlaneVertex = iPlaneVertices(i);
    
    %identify plane points
    iPlanePointVertices = unique(pointPlaneVertices(pointPlaneVertices(:,2)==iPlaneVertex,1));
    iPlanePointPositions = [verticesCell{iPlanePointVertices,3}];
    
    %plot plane points
%     plotPlanePoints(i) = plot3(iPlanePointPositions(1,:),iPlanePointPositions(2,:),iPlanePointPositions(3,:),'*');
%     set(plotPlanePoints(i),'MarkerEdgeColor',graphColour)
%     set(plotPlanePoints(i),'MarkerSize',5)
    
    %plot plane
    if config.plotPlanes
        planeParameters = planes(:,i);
        normal = planeParameters(1:3);
        distance = planeParameters(4);
        plotPlane(normal,distance,iPlanePointPositions,graphColour);
    end
end

end

