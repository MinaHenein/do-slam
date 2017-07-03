function [obj] = graphFileToGraph(obj,config,graphCell)
%GRAPHFILETOGRAPH Summary of this function goes here
%   Detailed explanation goes here

%no vertices & edges
lineLengths  = cellfun(@length,graphCell);
vertexLines  = (lineLengths==3);
edgeLines    = (lineLengths==6);
verticesCell = graphCell(vertexLines);
edgesCell = graphCell(edgeLines);
nVertices = size(verticesCell,1);
nEdges    = size(edgesCell,1);

obj.vertices = Vertex();
obj.vertices(1:nVertices) = Vertex();
obj.edges    = Edge();
obj.edges(1:nEdges) = Edge();

%construct vertices
for i = 1:nVertices
    switch verticesCell{i}{1}
        case config.poseVertexLabel
            type = 'pose';
            value = verticesCell{i}{3};
        case config.pointVertexLabel
            type = 'point';
            value = verticesCell{i}{3};
        case config.pointRGBVertexLabel    
            type = 'point';
            value = verticesCell{i}{3}(1:3);
            pointColour = verticesCell{i}{3}(4:6);
        case config.planeVertexLabel
            type = 'plane';
            value = verticesCell{i}{3};
        case config.angleVertexLabel
            type = 'angle';
            value = verticesCell{i}{3};
        case config.distanceVertexLabel
            type = 'distance';
            value = verticesCell{i}{3};
        otherwise; error('wrong type')
    end
    covariance = [];
    iEdges = [];
    index = verticesCell{i}{2};

    %construct
    obj.vertices(i) = Vertex(value,covariance,type,iEdges,index);
    
    %add colour
    if strcmp(verticesCell{i}{1},config.pointRGBVertexLabel)
        obj.vertices(i).colour = pointColour;
    end
end
% sort vertices just in case
[~, I] = sort([obj.vertices.index]);
vertices = obj.vertices(I);
obj.vertices = vertices;

%construct edges
for i = 1:nEdges
    switch edgesCell{i}{1}
        case config.posePoseEdgeLabel
            type = 'pose-pose';
            value = edgesCell{i}{5};
            covariance = upperTriVecToCov(edgesCell{i}{6});
        case config.posePointEdgeLabel
            type = 'pose-point';
            value = edgesCell{i}{5};
            covariance = upperTriVecToCov(edgesCell{i}{6});
%         case config.posePointRGBEdgeLabel
%             type = 'pose-point';
%             value = edgesCell{i}{5}(1:3);
%             pointColour = edgesCell{i}{5}(4:6);
%             covariance = upperTriVecToCov(edgesCell{i}{6});
        case config.pointPointEdgeLabel
            type = 'point-point';
            value = edgesCell{i}{5};
            covariance = upperTriVecToCov(edgesCell{i}{6});
        case config.pointPlaneEdgeLabel
            type = 'point-plane';
            value = edgesCell{i}{5};
            covariance = edgesCell{i}{6};
        case config.angleEdgeLabel
            type = 'plane-plane-angle';
            value = edgesCell{i}{5};
            covariance = edgesCell{i}{6};
        case config.fixedAngleEdgeLabel
            type = 'plane-plane-fixedAngle';
            value = edgesCell{i}{5};
            covariance = edgesCell{i}{6};
        case config.distanceEdgeLabel
            type = 'plane-plane-distance';
            value = edgesCell{i}{5};
            covariance = edgesCell{i}{6};
        case config.fixedDistanceEdgeLabel
            type = 'plane-plane-fixedDistance';
            value = edgesCell{i}{5};
            covariance = edgesCell{i}{6};
        case config.planePriorEdgeLabel
        otherwise; error('wrong type')
    end
    jacobians   = [];
    iVertices   = [edgesCell{i}{3:4}];
    index       = edgesCell{i}{2};
    obj.edges(i) = Edge(value,covariance,jacobians,type,iVertices,index);
    
    %add edge index to vertices
    for j = 1:numel(iVertices)
        obj.vertices(iVertices(j)).iEdges = ...
            unique([obj.vertices(iVertices(j)).iEdges index]); %order doesn't matter here
        %and colour
        if strcmp(edgesCell{i}{1},config.pointRGBVertexLabel)
            iPointVertex = iVertices(1);
            obj.vertices(iPointVertex).colour = pointColour;
        end
    end   
end

%update - gets jacobians
obj = obj.updateEdges(config);

end

