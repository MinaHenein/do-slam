function [graphCell] = graphFileToCell(config,fileName,varargin)
%GRAPHFILETOCELL Parses graph file into cell array
%   This is largely for convenience. ie edge indexes are added here.
%   In some cases however, additional edges are added to the graph - but
%   these are specific to the solution method and are not required in the 
%   graph file: eg prior on the pose, edge n'n-1=0 constraining plane
%   normals to be unit vectors.

%open file
fileID = fopen(strcat(config.folderPath,config.sep,'Data',...
    config.sep,config.graphFileFolderName,config.sep,fileName),'r');

graphCell = cell(0);

%counts
noVertices = 1;
planeVertices = [];

%dont write angle constraints if desired
includeAngleConstraints = 1;
if nargin==3 && strcmp(varargin{1},'noAngleConstraints')
    includeAngleConstraints = 0;
end

%read lines
tline = fgets(fileID);
while ischar(tline)
    lineSplit = strsplit(tline);
    label = lineSplit{1};
    values = cellfun(@str2num,{lineSplit{2:end-1}});
    switch label
        case config.poseVertexLabel
            noVertices = 0;
            lineCell = {label,values(1),values(2:7)'};
            %store
            graphCell{end+1,1} = lineCell;
        case config.pointVertexLabel
            noVertices = 0;
            if strcmp(config.motionModel,'constantSE3Rob')||...
                    strcmp(config.motionModel,'constantSE3Mina')||...
                    strcmp(config.motionModel,'constantSE3')
                lineCell = {label,values(1),values(2:5)'};
            else
                lineCell = {label,values(1),values(2:4)'};
            end
            %store
            graphCell{end+1,1} = lineCell;
%         case config.pointRGBVertexLabel
%             noVertices = 0;
%             lineCell = {label,values(1),values(2:7)'};
%             %store
%             graphCell{end+1,1} = lineCell;
        case config.planeVertexLabel
            noVertices = 0;
            lineCell = {label,values(1),values(2:5)'};
            %store
            graphCell{end+1,1} = lineCell;
        case config.velocityVertexLabel
            noVertices = 0;
            if strcmp(config.motionModel,'constantSpeed')
                lineCell = {label,values(1),values(2)'};
            elseif strcmp(config.motionModel,'constantVelocity')
                lineCell = {label,values(1),values(1:3)'};
            end
            %store
            graphCell{end+1,1} = lineCell;
        case config.SE3MotionVertexLabel
            noVertices = 0;
            lineCell = {label,values(1),values(2:7)'};
            %store
            graphCell{end+1,1} = lineCell;
%         case config.angleVertexLabel 
%             noVertices = 0;
%             lineCell = {label,values(1),values(2)'};
%             %store
%             graphCell{end+1,1} = lineCell;
%         case config.distanceVertexLabel
%             noVertices = 0;
%             lineCell = {label,values(1),values(2)'};
%             %store
%             graphCell{end+1,1} = lineCell;
        case config.posePoseEdgeLabel
            lineCell = {label,[],values(1),values(2),values(3:8)',values(9:29)};
            %store
            graphCell{end+1,1} = lineCell;
        case config.posePointEdgeLabel
            if strcmp(config.motionModel,'constantSE3Rob')||...
                    strcmp(config.motionModel,'constantSE3Mina')||...
                    strcmp(config.motionModel,'constantSE3')
                lineCell = {label,[],values(1),values(2),values(3:6)',values(7:16)};
            else
                lineCell = {label,[],values(1),values(2),values(3:5)',values(6:11)};
            end
            %store
            graphCell{end+1,1} = lineCell;
%         case config.pointPointRGBEdgeLabel
%             lineCell = {label,[],values(1),values(2),values(3:8)',values(9:14)};
%             %store
%             graphCell{end+1,1} = lineCell;
        case config.pointPointEdgeLabel
            lineCell = {label,[],values(1),values(2),values(3:5)',values(6:11)};
             %store
            graphCell{end+1,1} = lineCell;
        case config.pointPointEdgeSE3Label
            lineCell = {label,[],values(1),values(2),values(3:6)',values(7:16)};
             %store
            graphCell{end+1,1} = lineCell;
        case config.point3EdgeLabel
            if strcmp(config.motionModel,'constantSpeed') 
                lineCell = {label,[],values(1:3),[],values(4),values(5)};
            elseif strcmp(config.motionModel,'constantVelocity') 
                lineCell = {label,[],values(1:3),[],values(4:6)',values(7:12)};
            end
            %store
            graphCell{end+1,1} = lineCell;
        case config.pointVelocityEdgeLabel
            if strcmp(config.motionModel,'constantSpeed')
                lineCell = {label,[],values(1:2),values(3),values(4),values(5)};
            elseif strcmp(config.motionModel,'constantVelocity')
                lineCell = {label,[],values(1:2),values(3),values(4:6)',values(7:12)'};
            end
             %store
            graphCell{end+1,1} = lineCell;
        case config.pointSE3MotionEdgeLabel
            if strcmp(config.motionModel,'constantSE3')
                lineCell = {label,[],values(1:2),values(3),values(4:7)',values(8:17)'};
            elseif strcmp(config.motionModel,'constantSE3Motion') || ...
                    strcmp(config.motionModel,'constantSE3MotionDA') || ...
                    strcmp(config.motionModel,'constantAccelerationSE3MotionDA')
                lineCell = {label,[],values(1:2),values(3),values(4:6)',values(7:12)'};
            end
            %store
            graphCell{end+1,1} = lineCell;
        case config.pointPlaneEdgeLabel 
            %need to add plane prior edge if first time plane vertex seen
            if strcmp(config.planeNormalParameterisation,'R3') && noVertices
                if ~any(planeVertices==values(2)) %haven't seen this vertex
                    planeVertices = [planeVertices values(2)];
                    %no value or covariance - will be computed later
                    lineCell = {config.labelPlanePriorEdge,[],values(2),[],[],[]};
                    graphCell{end+1,1} = lineCell;
                end
            end
            lineCell = {label,[],values(1),values(2),values(3)',values(4)};
            %store
            graphCell{end+1,1} = lineCell;
        case config.angleEdgeLabel
            if includeAngleConstraints
                lineCell = {label,[],values(1:2),values(3),values(4)',values(5)};
                %store
                graphCell{end+1,1} = lineCell;
            end
        case config.fixedAngleEdgeLabel
            if includeAngleConstraints
                lineCell = {label,[],values(1:2),[],values(3)',values(4)};
                %store
                graphCell{end+1,1} = lineCell;
            end
        case config.distanceEdgeLabel
            lineCell = {label,[],values(1:2),values(3),values(4)',values(5)};
            %store
            graphCell{end+1,1} = lineCell;
        case config.fixedDistanceEdgeLabel
            lineCell = {label,[],values(1:2),[],values(3)',values(4)};
            %store
            graphCell{end+1,1} = lineCell;
        otherwise; error('%s type invalid',label)
    end
     
    %get next line
    tline = fgets(fileID);
end

%% PROCESS - for no constraints mode (only edges in graph file)
if nargin==3 && strcmp(varargin{1},'noConstraints') && noVertices
    %adjust size
    nLines = numel(graphCell);
    graphCell = reshapeCell(graphCell,'array');
    
    %get no. vertices
    iVertices = unique([graphCell{:,3:4}])';
    nVertices = iVertices(end);
    
    %get vertexTypes
    vertexTypes = cell(nVertices,1);
    %loop over edges
    for i = 1:nLines
         switch graphCell{i,1}
            case config.posePoseEdgeLabel
                iPose1Vertex = graphCell{i,3};
                iPose2Vertex = graphCell{i,4};
                if isempty(vertexTypes{iPose1Vertex})
                    vertexTypes{iPose1Vertex} = config.poseVertexLabel;
                end
                if isempty(vertexTypes{iPose2Vertex})
                    vertexTypes{iPose2Vertex} = config.poseVertexLabel;
                end
            case config.posePointEdgeLabel
                iPoseVertex = graphCell{i,3};
                iPointVertex = graphCell{i,4};
                if isempty(vertexTypes{iPoseVertex})
                    vertexTypes{iPoseVertex} = config.poseVertexLabel;
                end
                if isempty(vertexTypes{iPointVertex})
                    vertexTypes{iPointVertex} = config.pointVertexLabel;
                end
            case config.pointPlaneEdgeLabel
                iPointVertex = graphCell{i,3};
                iPlaneVertex = graphCell{i,4};
                if isempty(vertexTypes{iPointVertex})
                    vertexTypes{iPointVertex} = config.pointVertexLabel;
                end
                if isempty(vertexTypes{iPlaneVertex})
                    vertexTypes{iPlaneVertex} = config.planeVertexLabel;
                end
            case config.angleEdgeLabel
                iAngleVertex = graphCell{i,4};
                if isempty(vertexTypes{iAngleVertex})
                    vertexTypes{iAngleVertex} = config.angleVertexLabel;
                end
            case config.fixedAngleEdgeLabel
            case config.distanceEdgeLabel
                iDistanceVertex = graphCell{i,4};
                if isempty(vertexTypes{iDistanceVertex})
                    vertexTypes{iDistanceVertex} = config.distanceVertexLabel;
                end
            case config.fixedDistanceEdgeLabel
         end
    end
    
    %identify valid vertices
    validVertices = strcmp(vertexTypes,config.poseVertexLabel)|...
                    strcmp(vertexTypes,config.pointVertexLabel);
    iVerticesAdjusted = cumsum(validVertices);
    iVerticesAdjusted(~validVertices) = 0;
    
    %get valid edges
    validEdges = strcmp(graphCell(:,1),config.posePoseEdgeLabel)|...
                 strcmp(graphCell(:,1),config.posePointEdgeLabel);
    graphCell = graphCell(validEdges,:);  
    
    %remap edge index vertices in graphCell
    for i = 1:size(graphCell,1)
        graphCell{i,2} = i;
        graphCell{i,3} = iVerticesAdjusted(graphCell{i,3});
        graphCell{i,4} = iVerticesAdjusted(graphCell{i,4});
    end
    
    %reshape to column
    graphCell = reshapeCell(graphCell,'column');
    
end

%% process ground truth graph file - remove constraints
if nargin==3 && strcmp(varargin{1},'noConstraints') && ~noVertices
    %split graphCell into verticesCell and edgesCell
    nLines = size(graphCell,1);
    lineLengths = cellfun(@length,graphCell);
    vertexLines = (lineLengths==3);
    edgeLines = (lineLengths==6);
    verticesCell = graphCell(vertexLines);
    edgesCell = graphCell(edgeLines);
    
    %reshape
    verticesCell = reshapeCell(verticesCell,'array');
    edgesCell = reshapeCell(edgesCell,'array');
    
    %get vertexTypes
    vertexTypes = verticesCell(:,1);
    
    %identify valid vertices
    validVertices = strcmp(vertexTypes,config.poseVertexLabel)|...
                    strcmp(vertexTypes,config.pointVertexLabel);
    iVerticesAdjusted = cumsum(validVertices);
    iVerticesAdjusted(~validVertices) = 0;
    
    %get valid edges
    validEdges = strcmp(edgesCell(:,1),config.posePoseEdgeLabel)|...
                 strcmp(edgesCell(:,1),config.posePointEdgeLabel);
    
    %loop over elements of graph cell, fix vertex indexes
    validLines = zeros(size(graphCell,1),1);
    edgeCount = 0;
    for i = 1:size(graphCell,1)
        switch numel(graphCell{i})
            case 3 %vertex
                vertexIndex = graphCell{i}{2};
                vertexValid = validVertices(vertexIndex);
                %if valid, remap index
                if vertexValid
                    validLines(i) = 1;
                    graphCell{i}{2} = iVerticesAdjusted(vertexIndex);
                end
            case 6 %edge
                edgeIndex = graphCell{i}{2};
                edgeValid = validEdges(edgeIndex);
                %if valid, remap indexes
                if edgeValid
                    edgeCount = edgeCount + 1;
                    validLines(i) = 1;
                    graphCell{i}{2} = []; 
                    graphCell{i}{3} = iVerticesAdjusted(graphCell{i}{3});
                    graphCell{i}{4} = iVerticesAdjusted(graphCell{i}{4});
                end
            otherwise error('wrong length')
        end     
    end
    
    %get valid lines
    graphCell = graphCell(logical(validLines));
end

%% close file
fclose(fileID);

end

