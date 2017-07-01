function [obj] = graphFileToGraphVerticesOnly(obj,config,graphCell)
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

end

