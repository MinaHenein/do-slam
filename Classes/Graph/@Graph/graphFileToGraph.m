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
        case config.labelPoseVertex
            type = 'pose';
            value = verticesCell{i}{3};
        case config.labelPointVertex
            type = 'point';
            value = verticesCell{i}{3};
        case config.labelPointRGBVertex    
            type = 'point';
            value = verticesCell{i}{3}(1:3);
            pointColour = verticesCell{i}{3}(4:6);
        case config.labelPlaneVertex
            type = 'plane';
            value = verticesCell{i}{3};
        case config.labelAngleVertex
            type = 'angle';
            value = verticesCell{i}{3};
        case config.labelDistanceVertex
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
    if strcmp(verticesCell{i}{1},config.labelPointRGBVertex)
        obj.vertices(i).colour = pointColour;
    end
end

%construct edges
for i = 1:nEdges
    switch edgesCell{i}{1}
        case config.labelPosePoseEdge
            type = 'pose-pose';
            value = edgesCell{i}{5};
            covariance = upperTriVecToCov(edgesCell{i}{6});
        case config.labelPosePointEdge
            type = 'pose-point';
            value = edgesCell{i}{5};
            covariance = upperTriVecToCov(edgesCell{i}{6});
        case config.labelPointPointRGBEdge
            type = 'pose-point';
            value = edgesCell{i}{5}(1:3);
            pointColour = edgesCell{i}{5}(4:6);
            covariance = upperTriVecToCov(edgesCell{i}{6});
        case config.labelPointPlaneEdge
            type = 'point-plane';
            value = edgesCell{i}{5};
            covariance = edgesCell{i}{6};
        case config.labelAngleEdge
            type = 'plane-plane-angle';
            value = edgesCell{i}{5};
            covariance = edgesCell{i}{6};
        case config.labelFixedAngleEdge
            type = 'plane-plane-fixedAngle';
            value = edgesCell{i}{5};
            covariance = edgesCell{i}{6};
        case config.labelDistanceEdge
            type = 'plane-plane-distance';
            value = edgesCell{i}{5};
            covariance = edgesCell{i}{6};
        case config.labelFixedDistanceEdge
            type = 'plane-plane-fixedDistance';
            value = edgesCell{i}{5};
            covariance = edgesCell{i}{6};
        case config.labelPlanePriorEdge
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
        if strcmp(edgesCell{i}{1},config.labelPointRGBVertex)
            iPointVertex = iVertices(1);
            obj.vertices(iPointVertex).colour = pointColour;
        end
    end   
end

%update - gets jacobians
obj = obj.updateEdges(config);

end

