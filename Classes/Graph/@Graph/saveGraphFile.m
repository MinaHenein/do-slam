function saveGraphFile(obj,config,fileName)
%SAVEGRAPHFILE saves information from Graph class instance in a graph file
%indicated by fileName input

if ispc
    fileID = fopen(strcat(config.savePath,'\Data\GraphFiles\',fileName),'w');
elseif isunix || ismac
    fileID = fopen(strcat(config.savePath,'/Data/GraphFiles/',fileName),'w');
end

%***ORDERING???

%% 1. write vertices
for i = 1:obj.nVertices
    switch obj.vertices(i).type
        case 'pose';        vertexLabel = config.poseVertexLabel;
        case 'point';       vertexLabel = config.pointVertexLabel;
        case 'intrinsics';  vertexLabel = config.intrinsicVertexLabel;
        case 'plane';       vertexLabel = config.planeVertexLabel;
        case 'angle';       vertexLabel = config.angleVertexLabel;
        case 'distance';    vertexLabel = config.distanceVertexLabel;
        case 'velocity';    vertexLabel = config.velocityVertexLabel;
        case 'SE3Motion';   vertexLabel = config.SE3MotionVertexLabel;
        otherwise; error('type invalid')
    end
    vertexIndex = obj.vertices(i).index;
    vertexValue = obj.vertices(i).value;
    formatSpec = strcat('%s %d ',repmat(' %6.6f',1,numel(vertexValue)),'\n');
    fprintf(fileID,formatSpec,vertexLabel,vertexIndex,vertexValue);
end

%% 2. write edges
for i = 1:obj.nEdges
    writeEdge = 1;
    switch obj.edges(i).type
        case 'pose-pose'                  
            edgeLabel = config.posePoseEdgeLabel;
            verticesIn = obj.edges(i).iVertices(1);
            verticesOut = obj.edges(i).iVertices(2);
            edgeCovariance = covToUpperTriVec(obj.edges(i).covariance); 
        case 'pose-point'                  
            edgeLabel = config.posePointEdgeLabel;
            verticesIn = obj.edges(i).iVertices(1);
            verticesOut = obj.edges(i).iVertices(2);
            edgeCovariance = covToUpperTriVec(obj.edges(i).covariance);
        case 'pose-point-intrinsic'                  
            edgeLabel = config.posePointIntrinsicEdgeLabel;
            verticesIn = obj.edges(i).iVertices(1:2);
            verticesOut = obj.edges(i).iVertices(3);
            edgeCovariance = covToUpperTriVec(obj.edges(i).covariance);
        case 'point-point'
            edgeLabel = config.pointPointEdgeLabel;
            verticesIn = obj.edges(i).iVertices(1);
            verticesOut = obj.edges(i).iVertices(2);
            edgeCovariance = covToUpperTriVec(obj.edges(i).covariance);           
        case 'point-pointSE3'
            edgeLabel = config.pointPointEdgeSE3Label;
            verticesIn = obj.edges(i).iVertices(1);
            verticesOut = obj.edges(i).iVertices(2);
            edgeCovariance = covToUpperTriVec(obj.edges(i).covariance);
        case '3-points'
            edgeLabel = config.point3EdgeLabel;
            verticesIn = obj.edges(i).iVertices;
            verticesOut = [];
            edgeCovariance = obj.edges(i).covariance;
        case '2points-velocity'
            edgeLabel = config.pointVelocityEdgeLabel;
            verticesIn = obj.edges(i).iVertices(1:2);
            verticesOut = obj.edges(i).iVertices(3);
            edgeCovariance = obj.edges(i).covariance;
        case '2points-SE3Motion'
            edgeLabel = config.pointSE3MotionEdgeLabel;
            verticesIn = obj.edges(i).iVertices(1:2);
            verticesOut = obj.edges(i).iVertices(3);
            edgeCovariance = obj.edges(i).covariance;
        case 'point-plane'               
            edgeLabel = config.pointPlaneEdgeLabel;
            verticesIn = obj.edges(i).iVertices(1);
            verticesOut = obj.edges(i).iVertices(2);
            edgeCovariance = obj.edges(i).covariance;
        case 'plane-plane-angle'          
            edgeLabel = config.angleEdgeLabel;
            verticesIn = obj.edges(i).iVertices(1:2);
            verticesOut = obj.edges(i).iVertices(3);
            edgeCovariance = obj.edges(i).covariance;
        case 'plane-plane-fixedAngle'    
            edgeLabel = config.fixedAngleEdgeLabel;
            verticesIn = obj.edges(i).iVertices;
            verticesOut = [];
            edgeCovariance = obj.edges(i).covariance;
        case 'plane-plane-distance'       
            edgeLabel = config.distanceEdgeLabel;
            verticesIn = obj.edges(i).iVertices(1:2);
            verticesOut = obj.edges(i).iVertices(3);
            edgeCovariance = obj.edges(i).covariance;
        case 'plane-plane-fixedDistance'
            edgeLabel = config.fixedDistanceEdgeLabel;
            verticesIn = obj.edges(i).iVertices;
            verticesOut = [];
            edgeCovariance = obj.edges(i).covariance;
        case {'posePrior','planePrior'}                   
            writeEdge = 0;
        otherwise
            error('type invalid')
    end
    if writeEdge
        %edgeIndex = obj.edges(i).index;
        edgeValue = obj.edges(i).value;               
        if isempty(verticesOut)
            formatSpec = strcat('%s',...
                                repmat(' %d',1,numel(verticesIn)),...
                                repmat(' %.6f',1,numel(edgeValue)),...
                                repmat(' %.6f',1,numel(edgeCovariance)),'\n');
            fprintf(fileID,formatSpec,edgeLabel,verticesIn,edgeValue,edgeCovariance);
        else
            formatSpec = strcat('%s',...
                                repmat(' %d',1,numel(verticesIn)),...
                                repmat(' %d',1,numel(verticesOut)),...
                                repmat(' %.6f',1,numel(edgeValue)),...
                                repmat(' %.6f',1,numel(edgeCovariance)),'\n');
            fprintf(fileID,formatSpec,edgeLabel,verticesIn,verticesOut,edgeValue,edgeCovariance);
        end
    end
end


%% 
%close file
fclose(fileID);

end

